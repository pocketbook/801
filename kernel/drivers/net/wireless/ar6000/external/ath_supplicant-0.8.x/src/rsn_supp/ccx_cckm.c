/*
 * WPA Supplicant - CCX Fast Roaming with CCKM Enhancements
 * Copyright (c) 2003-2008, Jouni Malinen <j@w1.fi>
 * Copyright (c) 2010-2011, embWiSe Technologies
 *
 * Licensed under a proprietary license. 
 * Notifications and licenses are retained for attribution purposes only.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name(s) of the above-listed copyright holder(s) nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 */
#include "includes.h"

#include "common.h"
#include "crypto/md5.h"
#include "crypto/crypto.h"
#include "crypto/sha1.h"
#include "crypto/sha256.h"
#include "crypto/aes_wrap.h"
#include "wpa.h"
#include "eloop.h"
#include "eapol_supp/eapol_supp_sm.h"
#include "preauth.h"
#include "pmksa_cache.h"
#ifdef CONFIG_CCX
#include "ccx_cckm.h"
#endif
#include "wpa_i.h"
#include "wpa_ie.h"
#include "peerkey.h"
#include "common/defs.h"
#include "common/ieee802_11_defs.h"
#include "../wpa_supplicant/wpa_supplicant_i.h"

/*
 * CCKM Keys for Initial Association
 * 	PRF_Data = (AP_ID | MN_ID | MN_Nonce | AP_Nonce)
 * 	CCKM_keys = PRF-384(NSK, "Fast-Roam Generate Base Key", PRF_DATA)
 *
 * Initial PTK
 * 	PRF_DATA = (RN | AP_ID)
 * 	PTK = PRF-512(BTK, "", PRF_DATA)
 *
 * MIC for Reassociation Request
 * 		MIC_DATA = (MN_ID | AP_ID | RSN_IE | TSF | RN):
 * 	WPA-TKIP / WPA-AES
 * 		MIC = HMAC-MD5(KRK, MIC_DATA)
 * 	WPA2-TKIP / WPA2-AES
 * 		MIC = HMAC-SHA1(KRK, MIC_DATA)
 *
 * MIC for Reassociation Response
 * 	MIC_DATA  = (MN_ID | RSN_IE | RN | Key_ID | Group_Key_ID |
 * 			RSC | GTK_Len | GTK_Data)
 *
 * WPA-TKIP / WPA-AES
 * 	MIC = HMAC-MD5(PTK[0..16], MIC_DATA)
 * WPA2-TKIP / WPA2-AES
 * 	MIC = HMAC-SHA1(PTK[0..16], MIC_DATA)
 *
 * WPA-TKIP / WPA2-TKIP
 * 	GTK = RC4(RN, PTK[16..31], GTK_Data)
 *
 * WPA-AES / WPA2-AES
 * 	GTK = AES-key-unwrap(PTK[16..31], GTK_Data)
 */

/* (Re)association Response IE */
struct cckm_ie_req {
	u8 elem_id;
	u8 len;
	u8 oui[4]; /* 24-bit OUI followed by 8-bit OUI type */
	u8 timestamp[8];
	u8 rn[4];
	u8 mn_mic[8];
} STRUCT_PACKED;

struct cckm_ie_rsp {
	u8 elem_id;
	u8 len;
	u8 oui[4]; /* 24-bit OUI followed by 8-bit OUI type */
	u8 rn[4];
	u8 unicast_keyid;
	u8 mcast_keyid;
	u8 rsc[8];
	u8 gtk_len[2];
	u8 ap_mic[8];
	u8 gtk[1];
} STRUCT_PACKED;

struct ccx_report {
	u8 aironet_snap[8];
	u8 length[2];
	u8 type;
	u8 function_type;
	u8 destination[ETH_ALEN];
	u8 source[ETH_ALEN];
	u8 tlv_tag[2];
	u8 tlv_len[2];
	u8 aironet_oui[4];
	u8 bssid[ETH_ALEN];
	u8 channel[2];
	u8 ssid_len[2];
	/* u16 disassoc_time */
} STRUCT_PACKED;

static u8 aironet_snap[8] = {0xaa, 0xaa, 0x03, 0x00, 0x40, 0x96, 0x00, 0x00};
static u8 aironet_oui[4] = {0x00, 0x40, 0x96, 0x00};

int wpa_sm_is_cckm_resp_ie_set (struct wpa_sm *sm)
{
	return (sm->cckm_resp_ie_len == 0) ? 0 : 1;
}

int wpa_sm_set_cckm_resp_ie(struct wpa_sm *sm, const u8 *ie, size_t len)
{
	if (sm == NULL)
		return -1;

	os_free(sm->cckm_resp_ie);
	if (ie == NULL || len == 0) {
		wpa_printf(MSG_DEBUG, "WPA: clearing own WPA/RSN IE");
		sm->cckm_resp_ie = NULL;
		sm->cckm_resp_ie_len = 0;
	} else {
		wpa_hexdump(MSG_DEBUG, "WPA: set own WPA/RSN IE", ie, len);
		sm->cckm_resp_ie = os_malloc(len);
		if (sm->cckm_resp_ie == NULL)
			return -1;

		os_memcpy(sm->cckm_resp_ie, ie, len);
		sm->cckm_resp_ie_len = len;
	}

	return 0;
}

int cckm_fr_is_completed(struct wpa_sm *sm)
{
	if (sm == NULL)
		return 0;

	if (sm->key_mgmt != WPA_KEY_MGMT_CCKM)
		return 0;

	return 1;
}

/**
 * cckm_egtk_to_gtk - Calculate GTK from Encrypted GTK
 * @cckm_rn: Association Request Numner
 * @gtk: Length of BTK
 * @gtk_len: Length of GTK
 *
 * GTK = RC4(RN | PTK-802.1X-Encrypt-Key, EGTK)
 * GTK = AES-Keywrap(PTK-802.1X-Encrypt-Key, EGTK)
 * - where PTK-802.1X-Encrypt-Key is the key as defined in Section 0
 * - Note that while the IV is not transmitted in the clear, its encrypted
 *   value is included in the EGTK. Thus, the EGTK length shall be the GTK
 *   key length plus 8 octets (e.g. the IV length). More explicitly, if the
 *   broadcast cipher is TKIP, while the GTK is 32 octets, the EGTK shall
 *   be 40 octets; similarly for AESCCMP, the GTK is 16 octets while the
 *   EGTK shall be 24 octets.
 */
static int cckm_install_gtk(struct wpa_sm *sm, u32 cckm_rn, u8 *gtk,
		int gtk_len, int keyidx, const u8 *key_rsc, int key_rsc_len)
{
	u8 gtk_buf[32];
	enum wpa_alg alg;

	switch (sm->group_cipher) {
		case WPA_CIPHER_CCMP:
			alg = WPA_ALG_CCMP;
			break;
		case WPA_CIPHER_TKIP:
			alg = WPA_ALG_TKIP;
			break;
		case WPA_CIPHER_NONE:
			wpa_printf(MSG_DEBUG, "WPA: Pairwise Cipher Suite: "
					"NONE - do not use pairwise keys");
			return 0;
		default:
			wpa_printf(MSG_WARNING, "WPA: Unsupported pairwise cipher %d",
					sm->pairwise_cipher);
			return -1;
	}

	if (sm->group_cipher == WPA_CIPHER_CCMP) {
		gtk_len -= 8;
		if (aes_unwrap(sm->ptk.kek, gtk_len / 8, gtk, gtk_buf)) {
			wpa_printf(MSG_WARNING, "WPA: AES unwrap "
					"failed - could not decrypt GTK");
			return -1;
		}
		gtk = gtk_buf;
	} else if (sm->group_cipher == WPA_CIPHER_TKIP) {
		u8 data[20];

		WPA_PUT_LE32(data, cckm_rn);
		os_memcpy(data + 4, &sm->ptk.kek, sizeof(sm->ptk.kek));
		wpa_hexdump_key(MSG_DEBUG, "EGTK-Data", data, sizeof(data));

		rc4_skip(data, sizeof(data), 256, gtk, gtk_len);

		/* Swap Tx/Rx keys for Michael MIC */
		os_memcpy(gtk_buf, gtk, 16);
		os_memcpy(gtk_buf + 16, gtk + 24, 8);
		os_memcpy(gtk_buf + 24, gtk + 16, 8);
		gtk = gtk_buf;
	}

	wpa_hexdump_key(MSG_DEBUG, "WPA: Group Key", gtk, gtk_len);
	wpa_printf(MSG_DEBUG, "WPA: Installing GTK to the driver "
			"(keyidx=%d tx=%d len=%d).", keyidx, 0, gtk_len);
	wpa_hexdump(MSG_DEBUG, "WPA: RSC", key_rsc, key_rsc_len);

	if (wpa_sm_set_key(sm, alg, (u8 *) "\xff\xff\xff\xff\xff\xff",
				keyidx, 0, key_rsc, key_rsc_len,
				gtk, gtk_len) < 0) {
		wpa_printf(MSG_WARNING, "WPA: Failed to set GTK to "
				"the driver.");
		return -1;
	}

	return 0;
}

static int cckm_install_ptk(struct wpa_sm *sm)
{
	int keylen, rsclen;
	enum wpa_alg alg;
	u8 null_rsc[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	wpa_printf(MSG_DEBUG, "WPA: Installing PTK to the driver.");

	switch (sm->pairwise_cipher) {
	case WPA_CIPHER_CCMP:
		alg = WPA_ALG_CCMP;
		keylen = 16;
		rsclen = 6;
		break;
	case WPA_CIPHER_TKIP:
		alg = WPA_ALG_TKIP;
		keylen = 32;
		rsclen = 6;
		break;
	case WPA_CIPHER_NONE:
		wpa_printf(MSG_DEBUG, "WPA: Pairwise Cipher Suite: "
				"NONE - do not use pairwise keys");
		return 0;
	default:
		wpa_printf(MSG_WARNING, "WPA: Unsupported pairwise cipher %d",
				sm->pairwise_cipher);
		return -1;
	}

	if (wpa_sm_set_key(sm, alg, sm->bssid, 0, 1, null_rsc, rsclen,
				(u8 *) sm->ptk.tk1, keylen) < 0) {
		wpa_printf(MSG_WARNING, "WPA: Failed to set PTK to the "
				"driver.");
		return -1;
	}

	return 0;
}

/*
 * MIC Calculation / Verification
 *
 * for WPA
 * 	MICAP = HMAC-MD5(PTK-802.1X-MIC-Key , STA-ID | RSNIEAP | RN |
 * 		KeyIDunicast | KeyIDmulticast | RSC | MulticastKeyLength | EGTK)
 *
 * for WPA2
 * 	MICAP = HMAC-SHA1(PTK-802.1X-MIC-Key , STA-ID | RSNIEAP | RN |
 * 		KeyIDunicast | KeyIDmulticast | RSC | MulticastKeyLength | EGTK)
 */
static int cckm_verify_mic(struct wpa_sm *sm, struct cckm_ie_rsp *cie, u8* kck)
{
	u8 data[128];
	u8 mic[32];
	u32 data_len = ETH_ALEN;

	os_memcpy(data, sm->own_addr, ETH_ALEN);

	if (sm->ap_rsn_ie && sm->ap_rsn_ie_len) {
		os_memcpy(data+data_len, sm->ap_rsn_ie, sm->ap_rsn_ie_len);
		data_len += sm->ap_rsn_ie_len;
	} else if (sm->ap_wpa_ie && sm->ap_wpa_ie_len) {
		os_memcpy(data+data_len, sm->ap_wpa_ie, sm->ap_wpa_ie_len);
		data_len += sm->ap_wpa_ie_len;
	} else if (sm->assoc_wpa_ie && sm->assoc_wpa_ie_len) {
		os_memcpy(data + data_len, sm->assoc_wpa_ie,
				sm->assoc_wpa_ie_len);
		data_len += sm->assoc_wpa_ie_len;
	}
	os_memcpy(data+data_len, cie->rn, 4);
	data_len += 4;
	os_memcpy(data+data_len, &cie->unicast_keyid, 12);
	data_len += 12;
	os_memcpy(data+data_len, &cie->gtk, WPA_GET_LE16(cie->gtk_len));
	data_len += WPA_GET_LE16(cie->gtk_len);

	if (sm->proto == WPA_PROTO_RSN) {
		hmac_sha1(kck, 16, data, data_len, mic);
	} else {
		hmac_md5(kck, 16, data, data_len, mic);
	}

	wpa_hexdump(MSG_DEBUG, "GTK MIC", mic, 8);
	wpa_hexdump(MSG_DEBUG, "RIE MIC", cie->ap_mic, 8);

	return os_memcmp(mic, cie->ap_mic, 8);
}


int cckm_build_request (struct wpa_sm *sm, u8 *bssid, u64 timestamp)
{
	struct cckm_ie_req *cie;
	u8 data[128];
	u32 data_len = ETH_ALEN;

	if (sm->cckm_req_ie) {
		os_free(sm->cckm_req_ie);
	}
	sm->cckm_req_ie = os_malloc(32);
	if (sm->cckm_req_ie == NULL)
		return -1;
	cie = (struct cckm_ie_req *)sm->cckm_req_ie;

	os_memcpy(data, sm->own_addr, ETH_ALEN);
	os_memcpy(data+data_len, bssid, ETH_ALEN);
	data_len += ETH_ALEN;

	if (sm->connect_wpa_ie && sm->connect_wpa_ie_len) {
		os_memcpy(data + data_len, sm->connect_wpa_ie,
				sm->connect_wpa_ie_len);
		data_len += sm->connect_wpa_ie_len;
	}

	sm->cckm_rn++;
	cie->elem_id = WLAN_EID_CCKM;
	cie->len = 24;
	os_memcpy(cie->oui, aironet_oui, sizeof(aironet_oui));
	WPA_PUT_LE64(cie->timestamp, timestamp);
	WPA_PUT_LE32(cie->rn, sm->cckm_rn);

	os_memcpy(data+data_len, cie->timestamp, 8);
	data_len += 8;
	os_memcpy(data+data_len, cie->rn, 4);
	data_len += 4;

	if (sm->proto == WPA_PROTO_RSN) {
		hmac_sha1(sm->cgk.krk, CCKM_KRK_LEN, data, data_len,
				cie->mn_mic);
	} else {
		hmac_md5(sm->cgk.krk, CCKM_KRK_LEN, data, data_len,
				cie->mn_mic);
	}

	sm->cckm_req_ie_len = sizeof(struct cckm_ie_req);

	wpa_hexdump(MSG_DEBUG, "req data ", data, data_len);
	wpa_hexdump(MSG_DEBUG, "cckm_req ", sm->cckm_req_ie,
			sm->cckm_req_ie_len);

	wpa_sm_update_cckm_ies(sm, sm->cckm_req_ie, sm->cckm_req_ie_len);
	return 0;
}


int cckm_process_response (struct wpa_sm *sm, u8 *bssid)
{
	struct wpa_ptk lptk;
	struct wpa_ptk *ptk = &lptk;
	u32 cckm_rn;
	struct cckm_ie_rsp *cie = (struct cckm_ie_rsp *)sm->cckm_resp_ie;
	u8 data[8];

	/* Reject Non-CCKM IEs */
	if ((cie->elem_id != WLAN_EID_CCKM) ||
			(os_memcmp(cie->oui, "\x00\x40\x96\x00",
				   sizeof(cie->oui)) != 0)) {
		return -1;
	}

	/* Derive PTK from BTK and RN */
	cckm_rn = WPA_GET_LE32(cie->rn);
	wpa_btk_to_ptk(sm->cgk.btk, sizeof(sm->cgk.btk), bssid,
			cckm_rn, (u8 *)ptk, sizeof(struct wpa_ptk));

	if (cckm_verify_mic(sm, cie, ptk->kck)) {
		wpa_printf(MSG_WARNING, "CCKM Reassociation Response - "
				"MIC Failure\n");
		return -1;
	}

	if (sm->pairwise_cipher == WPA_CIPHER_TKIP) {
		/* Supplicant: swap tx/rx Mic keys */
		os_memcpy(data, ptk->u.auth.tx_mic_key, 8);
		os_memcpy(ptk->u.auth.tx_mic_key, ptk->u.auth.rx_mic_key, 8);
		os_memcpy(ptk->u.auth.rx_mic_key, data, 8);
	}

	/* MIC verification successful, store and install PTK */
	os_memcpy (&sm->ptk, ptk, sizeof(struct wpa_ptk));
	cckm_install_ptk(sm);

	/* Install GTK */
	cckm_install_gtk(sm, cckm_rn, cie->gtk, WPA_GET_LE16(cie->gtk_len),
			cie->mcast_keyid, cie->rsc, 6);

	wpa_printf(MSG_INFO, "CCKM-FAST-ROAMING Connection to " MACSTR
			" Completed", MAC2STR(bssid));
	return 0;
}

int ccx_send_report (void *ctx)
{
	struct wpa_supplicant *wpa_s = ctx;
	u8 data[128];
	struct os_time t;
	u32 length;
	struct ccx_report *rep = (struct ccx_report *)data;
	struct ccx_apinfo *apinfo = &wpa_s->apinfo;

	os_memcpy(rep->aironet_snap, aironet_snap, sizeof(aironet_snap));
	rep->type = 0x30;		// REPORT FRAME
	rep->function_type = 0x00;
	os_memcpy(rep->destination, wpa_s->bssid, ETH_ALEN);
	os_memcpy(rep->source, wpa_s->own_addr, ETH_ALEN);
	WPA_PUT_BE16(rep->tlv_tag, 0x009b);
	os_memcpy(rep->aironet_oui, aironet_oui, sizeof(aironet_oui));
	os_memcpy(rep->bssid, apinfo->bssid, ETH_ALEN);
	WPA_PUT_BE16(rep->channel, apinfo->channel);
	WPA_PUT_BE16(rep->ssid_len, apinfo->ssid_len);

	length = sizeof(struct ccx_report);
	os_memcpy(&data[length], apinfo->ssid, apinfo->ssid_len);
	length += apinfo->ssid_len;
	os_get_time(&t);
	if (apinfo->disconnect_time)
		WPA_PUT_BE16(&data[length], (t.sec - apinfo->disconnect_time));
	else
		WPA_PUT_BE16(&data[length], 0);
	length += 2;

	WPA_PUT_BE16(rep->tlv_len, (apinfo->ssid_len + 16));
	WPA_PUT_BE16(rep->length, (length - sizeof(aironet_snap)));

	wpa_sm_ether_send(wpa_s->wpa, wpa_s->bssid, 0, data, length);
	wpa_hexdump(MSG_DEBUG, "CCX REPORT", data, length);
	return 0;
}

/**
 * wpa_nsk_to_cgk - Calculate CCMP Keys from NSK, addresses, and nonces
 * @nsk: Network Session Key
 * @nsk_len: Length of NSK
 * @addr1: AA
 * @addr2: SA
 * @nonce1: SNonce
 * @nonce2: ANonce
 * @cgk: Buffer for CCKM Common Keys
 * @cgk_len: Length of CGK
 *
 */
void wpa_nsk_to_cgk(const u8 *nsk, size_t nsk_len,
		const u8 *addr1, const u8 *addr2,
		const u8 *nonce1, const u8 *nonce2,
		u8 *cgk, size_t cgk_len)
{
	u8 data[2 * ETH_ALEN + 2 * WPA_NONCE_LEN];
	const char *label = "Fast-Roam Generate Base Key";

	os_memcpy(data, addr1, ETH_ALEN);
	os_memcpy(data + ETH_ALEN, addr2, ETH_ALEN);

	os_memcpy(data + 2 * ETH_ALEN, nonce1, WPA_NONCE_LEN);
	os_memcpy(data + 2 * ETH_ALEN + WPA_NONCE_LEN, nonce2, WPA_NONCE_LEN);
	sha1_prf(nsk, nsk_len, label, data, sizeof(data), cgk, cgk_len);

	wpa_hexdump_key(MSG_DEBUG, "WPA: NSK", nsk, nsk_len);
	wpa_hexdump_key(MSG_DEBUG, "WPA: CGK", cgk, cgk_len);
}


/**
 * wpa_btk_to_ptk - Calculate PTK from BTK, address, and Replay Counter
 * @btk: Base Transient Key
 * @btk_len: Length of BTK
 * @bssid: BSSID / AP ID
 * @ptk: Buffer for Pairwise Transient Key
 * @ptk_len: Length of PTK
 *
 * PTK = PRF-512(BTK, "", RN || AP_ID)
 */
void wpa_btk_to_ptk(const u8 *btk, size_t btk_len, const u8 *bssid,
		u32 cckm_rn, u8 *ptk, size_t ptk_len)
{
	u8 data[ETH_ALEN + 4];

	WPA_PUT_LE32(data, cckm_rn);
	os_memcpy(data + 4, bssid, ETH_ALEN);

	sha1_prf(btk, btk_len, NULL, data, sizeof(data), ptk, ptk_len);
	wpa_hexdump_key(MSG_DEBUG, "WPA: BTK-PTK", ptk, ptk_len);
}


int wpa_sm_set_cckm_req_ie(struct wpa_sm *sm, const u8 *ie, size_t len)
{
	if (sm == NULL)
		return -1;

	os_free(sm->cckm_req_ie);
	if (ie == NULL || len == 0) {
		wpa_printf(MSG_DEBUG, "WPA: clearing own CCKM IE");
		sm->cckm_req_ie = NULL;
		sm->cckm_req_ie_len = 0;
	} else {
		wpa_hexdump(MSG_DEBUG, "WPA: set own CCKM IE", ie, len);
		sm->cckm_req_ie = os_malloc(len);
		if (sm->cckm_req_ie == NULL)
			return -1;

		os_memcpy(sm->cckm_req_ie, ie, len);
		sm->cckm_req_ie_len = len;
	}

	return 0;
}

int wpa_sm_set_cckm_params(struct wpa_sm *sm, int reassociate)
{
	if (sm == NULL)
		return -1;
	if (reassociate)
		sm->cckm_rn++;
	else
		sm->cckm_rn = 1;

	return 0;
}


/**
 * wpa_sm_set_ccx_version_ie - Set own WPA/RSN IE from (Re)AssocReq
 * @sm: Pointer to WPA state machine data from wpa_sm_init()
 * @ie: Pointer to IE data (starting from id)
 * @len: IE length
 * Returns: 0 on success, -1 on failure
 *
 * Inform WPA state machine about the WPA/RSN IE used in (Re)Association
 * Request frame. The IE will be used to override the default value generated
 * with wpa_sm_set_assoc_wpa_ie_default().
 */
int wpa_sm_set_ccx_version_ie(struct wpa_sm *sm, const u8 *ie, size_t len)
{
	if (sm == NULL)
		return -1;

	os_free(sm->ccx_version_ie);
	if (ie == NULL || len == 0) {
		wpa_printf(MSG_DEBUG, "WPA: clearing own WPA/RSN IE");
		sm->ccx_version_ie = NULL;
		sm->ccx_version_ie_len = 0;
	} else {
		wpa_hexdump(MSG_DEBUG, "WPA: set own WPA/RSN IE", ie, len);
		sm->ccx_version_ie = os_malloc(len);
		if (sm->ccx_version_ie == NULL)
			return -1;

		os_memcpy(sm->ccx_version_ie, ie, len);
		sm->ccx_version_ie_len = len;
	}

	return 0;
}

