/*
 * WPA Supplicant - Basic AP mode support routines
 * Copyright (c) 2003-2009, Jouni Malinen <j@w1.fi>
 * Copyright (c) 2009, Atheros Communications
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 * See README and COPYING for more details.
 */

#include "utils/includes.h"

#include "utils/common.h"
#include "utils/eloop.h"
#include "utils/uuid.h"
#include "common/ieee802_11_defs.h"
#include "common/wpa_ctrl.h"
#include "ap/hostapd.h"
#include "ap/ap_config.h"
#include "ap/ap_drv_ops.h"
#ifdef NEED_AP_MLME
#include "ap/ieee802_11.h"
#endif /* NEED_AP_MLME */
#include "ap/beacon.h"
#include "ap/ieee802_1x.h"
#include "ap/wps_hostapd.h"
#include "ap/ctrl_iface_ap.h"
#include "eap_common/eap_defs.h"
#include "eap_server/eap_methods.h"
#include "eap_common/eap_wsc_common.h"
#include "wps/wps.h"
#include "common/ieee802_11_defs.h"
#include "config_ssid.h"
#include "config.h"
#include "wpa_supplicant_i.h"
#include "driver_i.h"
#include "p2p_supplicant.h"
#include "ap.h"
#include "ap/sta_info.h"
#include "notify.h"


#ifdef CONFIG_WPS
static void wpas_wps_ap_pin_timeout(void *eloop_data, void *user_ctx);
#endif /* CONFIG_WPS */

#if defined(CONFIG_AP) && defined(CONFIG_DRIVER_AR6003)
extern int ar6003_set_max_num_sta(void *priv, const u8 num_sta);
#endif

static int wpa_supplicant_conf_ap(struct wpa_supplicant *wpa_s,
				  struct wpa_ssid *ssid,
				  struct hostapd_config *conf)
{
	struct hostapd_bss_config *bss = &conf->bss[0];
	int pairwise;

	conf->driver = wpa_s->driver;

	os_strlcpy(bss->iface, wpa_s->ifname, sizeof(bss->iface));

	if (ssid->frequency == 0) {
		/* default channel 11 */
		conf->hw_mode = HOSTAPD_MODE_IEEE80211G;
		if(!(wpa_s->drv_flags & WPA_DRIVER_FLAGS_P2P_MGMT))
			conf->channel = 11;
	} else if (ssid->frequency >= 2412 && ssid->frequency <= 2472) {
		conf->hw_mode = HOSTAPD_MODE_IEEE80211G;
		conf->channel = (ssid->frequency - 2407) / 5;
	} else if ((ssid->frequency >= 5180 && ssid->frequency <= 5240) ||
		   (ssid->frequency >= 5745 && ssid->frequency <= 5825)) {
		conf->hw_mode = HOSTAPD_MODE_IEEE80211A;
		conf->channel = (ssid->frequency - 5000) / 5;
	} else {
		wpa_printf(MSG_ERROR, "Unsupported AP mode frequency: %d MHz",
			   ssid->frequency);
		return -1;
	}

	/* TODO: enable HT if driver supports it;
	 * drop to 11b if driver does not support 11g */

#ifdef CONFIG_P2P
	if (conf->hw_mode == HOSTAPD_MODE_IEEE80211G) {
		/* Remove 802.11b rates from supported and basic rate sets */
		int *list = os_malloc(4 * sizeof(int));
		if (list) {
			list[0] = 60;
			list[1] = 120;
			list[2] = 240;
			list[3] = -1;
		}
		conf->basic_rates = list;

		list = os_malloc(9 * sizeof(int));
		if (list) {
			list[0] = 60;
			list[1] = 90;
			list[2] = 120;
			list[3] = 180;
			list[4] = 240;
			list[5] = 360;
			list[6] = 480;
			list[7] = 540;
			list[8] = -1;
		}
		conf->supported_rates = list;
	}
#endif /* CONFIG_P2P */

	if (ssid->ssid_len == 0) {
		wpa_printf(MSG_ERROR, "No SSID configured for AP mode");
		return -1;
	}
	os_memcpy(bss->ssid.ssid, ssid->ssid, ssid->ssid_len);
	bss->ssid.ssid[ssid->ssid_len] = '\0';
	bss->ssid.ssid_len = ssid->ssid_len;
	bss->ssid.ssid_set = 1;

	if (wpa_key_mgmt_wpa_psk(ssid->key_mgmt))
		bss->wpa = ssid->proto;
	bss->wpa_key_mgmt = ssid->key_mgmt;
	bss->wpa_pairwise = ssid->pairwise_cipher;
	if (ssid->passphrase) {
		bss->ssid.wpa_passphrase = os_strdup(ssid->passphrase);
	} else if (ssid->psk_set) {
		os_free(bss->ssid.wpa_psk);
		bss->ssid.wpa_psk = os_zalloc(sizeof(struct hostapd_wpa_psk));
		if (bss->ssid.wpa_psk == NULL)
			return -1;
		os_memcpy(bss->ssid.wpa_psk->psk, ssid->psk, PMK_LEN);
		bss->ssid.wpa_psk->group = 1;
	}

	/* Select group cipher based on the enabled pairwise cipher suites */
	pairwise = 0;
	if (bss->wpa & 1)
		pairwise |= bss->wpa_pairwise;
	if (bss->wpa & 2) {
		if (bss->rsn_pairwise == 0)
			bss->rsn_pairwise = bss->wpa_pairwise;
		pairwise |= bss->rsn_pairwise;
	}
	if (pairwise & WPA_CIPHER_TKIP)
		bss->wpa_group = WPA_CIPHER_TKIP;
	else
		bss->wpa_group = WPA_CIPHER_CCMP;

	if (bss->wpa && bss->ieee802_1x)
		bss->ssid.security_policy = SECURITY_WPA;
	else if (bss->wpa)
		bss->ssid.security_policy = SECURITY_WPA_PSK;
	else if (bss->ieee802_1x) {
		bss->ssid.security_policy = SECURITY_IEEE_802_1X;
		bss->ssid.wep.default_len = bss->default_wep_key_len;
	} else if (bss->ssid.wep.keys_set)
		bss->ssid.security_policy = SECURITY_STATIC_WEP;
	else
		bss->ssid.security_policy = SECURITY_PLAINTEXT;

#ifdef CONFIG_WPS
	/*
	 * Enable WPS by default, but require user interaction to actually use
	 * it. Only the internal Registrar is supported.
	 */
	bss->eap_server = 1;
	bss->wps_state = 2;
	bss->ap_setup_locked = 2;
	if (wpa_s->conf->config_methods)
		bss->config_methods = os_strdup(wpa_s->conf->config_methods);
	os_memcpy(bss->device_type, wpa_s->conf->device_type,
		  WPS_DEV_TYPE_LEN);
	if (wpa_s->conf->device_name) {
		bss->device_name = os_strdup(wpa_s->conf->device_name);
		bss->friendly_name = os_strdup(wpa_s->conf->device_name);
	}
	if (wpa_s->conf->manufacturer)
		bss->manufacturer = os_strdup(wpa_s->conf->manufacturer);
	if (wpa_s->conf->model_name)
		bss->model_name = os_strdup(wpa_s->conf->model_name);
	if (wpa_s->conf->model_number)
		bss->model_number = os_strdup(wpa_s->conf->model_number);
	if (wpa_s->conf->serial_number)
		bss->serial_number = os_strdup(wpa_s->conf->serial_number);
	if (is_nil_uuid(wpa_s->conf->uuid))
		os_memcpy(bss->uuid, wpa_s->wps->uuid, WPS_UUID_LEN);
	else
		os_memcpy(bss->uuid, wpa_s->conf->uuid, WPS_UUID_LEN);
	os_memcpy(bss->os_version, wpa_s->conf->os_version, 4);
#endif /* CONFIG_WPS */

	if (wpa_s->max_stations &&
	    wpa_s->max_stations < wpa_s->conf->p2p_max_clients)
		bss->max_num_sta = wpa_s->max_stations;
	else
		bss->max_num_sta = wpa_s->conf->p2p_max_clients;

	bss->disassoc_low_ack = wpa_s->conf->disassoc_low_ack;

	return 0;
}


static void ap_public_action_rx(void *ctx, const u8 *buf, size_t len, int freq)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;
	const struct ieee80211_mgmt *mgmt;
	size_t hdr_len;

	mgmt = (const struct ieee80211_mgmt *) buf;
	hdr_len = (const u8 *) &mgmt->u.action.u.vs_public_action.action - buf;
	if (hdr_len > len)
		return;
	wpas_p2p_rx_action(wpa_s, mgmt->da, mgmt->sa, mgmt->bssid,
			   mgmt->u.action.category,
			   &mgmt->u.action.u.vs_public_action.action,
			   len - hdr_len, freq);
#endif /* CONFIG_P2P */
}


static void ap_wps_event_cb(void *ctx, enum wps_event event,
			    union wps_event_data *data)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;

	if (event == WPS_EV_FAIL && wpa_s->parent && wpa_s->parent != wpa_s &&
	    wpa_s == wpa_s->global->p2p_group_formation) {
		struct wps_event_fail *fail = &data->fail;

		/*
		 * src/ap/wps_hostapd.c has already sent this on the main
		 * interface, so only send on the parent interface here if
		 * needed.
		 */
		wpa_msg(wpa_s->parent, MSG_INFO, WPS_EVENT_FAIL
			"msg=%d config_error=%d",
			fail->msg, fail->config_error);
	}
#endif /* CONFIG_P2P */
}


static void ap_sta_authorized_cb(void *ctx, const u8 *mac_addr,
				 int authorized)
{
	wpas_notify_sta_authorized(ctx, mac_addr, authorized);
}


static int ap_vendor_action_rx(void *ctx, const u8 *buf, size_t len, int freq)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;
	const struct ieee80211_mgmt *mgmt;
	size_t hdr_len;

	mgmt = (const struct ieee80211_mgmt *) buf;
	hdr_len = (const u8 *) &mgmt->u.action.u.vs_public_action.action - buf;
	if (hdr_len > len)
		return -1;
	wpas_p2p_rx_action(wpa_s, mgmt->da, mgmt->sa, mgmt->bssid,
			   mgmt->u.action.category,
			   &mgmt->u.action.u.vs_public_action.action,
			   len - hdr_len, freq);
#endif /* CONFIG_P2P */
	return 0;
}


static int ap_probe_req_rx(void *ctx, const u8 *addr, const u8 *ie,
			   size_t ie_len)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;
	return wpas_p2p_probe_req_rx(wpa_s, addr, ie, ie_len);
#else /* CONFIG_P2P */
	return 0;
#endif /* CONFIG_P2P */
}


static void ap_wps_reg_success_cb(void *ctx, const u8 *mac_addr,
				  const u8 *uuid_e)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;
	wpas_p2p_wps_success(wpa_s, mac_addr, 1);
#endif /* CONFIG_P2P */
}


static void wpas_ap_configured_cb(void *ctx)
{
	struct wpa_supplicant *wpa_s = ctx;
	size_t i;

	wpa_supplicant_set_state(wpa_s, WPA_COMPLETED);
#if defined(CONFIG_AP) && defined(CONFIG_DRIVER_AR6003)
        if((wpa_s->current_ssid->frequency <= 0) && (wpa_s->drv_flags & WPA_DRIVER_FLAGS_P2P_MGMT)) {
            for(i =0; i < 10; i++){
                if(wpa_s->current_ssid->frequency > 0)
                   break;
                wpa_s->current_ssid->frequency = wpa_drv_get_freq(wpa_s);
                os_sleep(1, 0);
            } 
        }
#endif

	if (wpa_s->ap_configured_cb)
		wpa_s->ap_configured_cb(wpa_s->ap_configured_cb_ctx,
					wpa_s->ap_configured_cb_data);
}


int wpa_supplicant_create_ap(struct wpa_supplicant *wpa_s,
			     struct wpa_ssid *ssid)
{
	struct wpa_driver_associate_params params;
	struct hostapd_iface *hapd_iface;
	struct hostapd_config *conf;
	struct hostapd_data *hapd;
	size_t i;

	if (ssid->ssid == NULL || ssid->ssid_len == 0) {
		wpa_printf(MSG_ERROR, "No SSID configured for AP mode");
		return -1;
	}

	wpa_supplicant_ap_deinit(wpa_s);

	wpa_printf(MSG_DEBUG, "Setting up AP (SSID='%s')",
		   wpa_ssid_txt(ssid->ssid, ssid->ssid_len));

	os_memset(&params, 0, sizeof(params));
	params.ssid = ssid->ssid;
	params.ssid_len = ssid->ssid_len;
	switch (ssid->mode) {
	case WPAS_MODE_INFRA:
		params.mode = IEEE80211_MODE_INFRA;
		break;
	case WPAS_MODE_IBSS:
		params.mode = IEEE80211_MODE_IBSS;
		break;
	case WPAS_MODE_AP:
	case WPAS_MODE_P2P_GO:
	case WPAS_MODE_P2P_GROUP_FORMATION:
		params.mode = IEEE80211_MODE_AP;
		break;
	}
	params.freq = ssid->frequency;

	if (ssid->key_mgmt & WPA_KEY_MGMT_PSK)
		wpa_s->key_mgmt = WPA_KEY_MGMT_PSK;
	else
		wpa_s->key_mgmt = WPA_KEY_MGMT_NONE;
	params.key_mgmt_suite = key_mgmt2driver(wpa_s->key_mgmt);

	if (ssid->pairwise_cipher & WPA_CIPHER_CCMP)
		wpa_s->pairwise_cipher = WPA_CIPHER_CCMP;
	else if (ssid->pairwise_cipher & WPA_CIPHER_TKIP)
		wpa_s->pairwise_cipher = WPA_CIPHER_TKIP;
	else if (ssid->pairwise_cipher & WPA_CIPHER_NONE)
		wpa_s->pairwise_cipher = WPA_CIPHER_NONE;
	else {
		wpa_printf(MSG_WARNING, "WPA: Failed to select pairwise "
			   "cipher.");
		return -1;
	}
	params.pairwise_suite = cipher_suite2driver(wpa_s->pairwise_cipher);
	params.group_suite = params.pairwise_suite;

#ifdef CONFIG_P2P
	if (ssid->mode == WPAS_MODE_P2P_GO ||
	    ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION)
		params.p2p = 1;
	wpa_drv_set_intra_bss(wpa_s, wpa_s->conf->p2p_intra_bss);
#endif /* CONFIG_P2P */

	if (wpa_s->parent->set_ap_uapsd)
		params.uapsd = wpa_s->parent->ap_uapsd;
	else
		params.uapsd = -1;

	if (wpa_drv_associate(wpa_s, &params) < 0) {
		wpa_msg(wpa_s, MSG_INFO, "Failed to start AP functionality");
		return -1;
	}

	wpa_s->ap_iface = hapd_iface = os_zalloc(sizeof(*wpa_s->ap_iface));
	if (hapd_iface == NULL)
		return -1;
	hapd_iface->owner = wpa_s;

	wpa_s->ap_iface->conf = conf = hostapd_config_defaults();
	if (conf == NULL) {
		wpa_supplicant_ap_deinit(wpa_s);
		return -1;
	}

	if (wpa_supplicant_conf_ap(wpa_s, ssid, conf)) {
		wpa_printf(MSG_ERROR, "Failed to create AP configuration");
		wpa_supplicant_ap_deinit(wpa_s);
		return -1;
	}

#ifdef CONFIG_P2P
	if (ssid->mode == WPAS_MODE_P2P_GO)
		conf->bss[0].p2p = P2P_ENABLED | P2P_GROUP_OWNER;
	else if (ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION)
		conf->bss[0].p2p = P2P_ENABLED | P2P_GROUP_OWNER |
			P2P_GROUP_FORMATION;
#endif /* CONFIG_P2P */

	hapd_iface->num_bss = conf->num_bss;
	hapd_iface->bss = os_zalloc(conf->num_bss *
				    sizeof(struct hostapd_data *));
	if (hapd_iface->bss == NULL) {
		wpa_supplicant_ap_deinit(wpa_s);
		return -1;
	}

	for (i = 0; i < conf->num_bss; i++) {
		hapd_iface->bss[i] =
			hostapd_alloc_bss_data(hapd_iface, conf,
					       &conf->bss[i]);
		if (hapd_iface->bss[i] == NULL) {
			wpa_supplicant_ap_deinit(wpa_s);
			return -1;
		}

		hapd_iface->bss[i]->msg_ctx = wpa_s;
		hapd_iface->bss[i]->public_action_cb = ap_public_action_rx;
		hapd_iface->bss[i]->public_action_cb_ctx = wpa_s;
		hapd_iface->bss[i]->vendor_action_cb = ap_vendor_action_rx;
		hapd_iface->bss[i]->vendor_action_cb_ctx = wpa_s;
		hostapd_register_probereq_cb(hapd_iface->bss[i],
					     ap_probe_req_rx, wpa_s);
		hapd_iface->bss[i]->wps_reg_success_cb = ap_wps_reg_success_cb;
		hapd_iface->bss[i]->wps_reg_success_cb_ctx = wpa_s;
		hapd_iface->bss[i]->wps_event_cb = ap_wps_event_cb;
		hapd_iface->bss[i]->wps_event_cb_ctx = wpa_s;
		hapd_iface->bss[i]->sta_authorized_cb = ap_sta_authorized_cb;
		hapd_iface->bss[i]->sta_authorized_cb_ctx = wpa_s;
#ifdef CONFIG_P2P
		hapd_iface->bss[i]->p2p = wpa_s->global->p2p;
		hapd_iface->bss[i]->p2p_group = wpas_p2p_group_init(
			wpa_s, ssid->p2p_persistent_group,
			ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION);
#endif /* CONFIG_P2P */
		hapd_iface->bss[i]->setup_complete_cb = wpas_ap_configured_cb;
		hapd_iface->bss[i]->setup_complete_cb_ctx = wpa_s;
	}

	os_memcpy(hapd_iface->bss[0]->own_addr, wpa_s->own_addr, ETH_ALEN);
	hapd_iface->bss[0]->driver = wpa_s->driver;
	hapd_iface->bss[0]->drv_priv = wpa_s->drv_priv;

	wpa_s->current_ssid = ssid;
	os_memcpy(wpa_s->bssid, wpa_s->own_addr, ETH_ALEN);
	wpa_s->assoc_freq = ssid->frequency;
#if defined(CONFIG_AP) && defined(CONFIG_DRIVER_AR6003)
	if (wpa_s->conf->p2p_max_clients > 5) {
		wpa_printf(MSG_ERROR, "Cannot set max_num_sta to %d, maximum value is 5.  5 will be the value used.", wpa_s->conf->p2p_max_clients);
		wpa_s->conf->p2p_max_clients = 5;
	}
	wpa_printf(MSG_DEBUG, "Setting max_num_sta to %d", wpa_s->conf->p2p_max_clients);
	ar6003_set_max_num_sta(wpa_s->drv_priv, wpa_s->conf->p2p_max_clients);
#endif

#ifdef CONFIG_P2P
	if (wpa_s->conf->country[0] && wpa_s->conf->country[1]) {
                hapd = hapd_iface->bss[0];
		os_memcpy(hapd->iconf->country, wpa_s->conf->country, 2);
		hapd->iconf->country[2] ='\0';
        }
#endif

	if (hostapd_setup_interface(wpa_s->ap_iface)) {
		wpa_printf(MSG_ERROR, "Failed to initialize AP interface");
		wpa_supplicant_ap_deinit(wpa_s);
		return -1;
	}

	return 0;
}


void wpa_supplicant_ap_deinit(struct wpa_supplicant *wpa_s)
{
#ifdef CONFIG_WPS
	eloop_cancel_timeout(wpas_wps_ap_pin_timeout, wpa_s, NULL);
#endif /* CONFIG_WPS */

	if (wpa_s->ap_iface == NULL)
		return;

	wpa_s->current_ssid = NULL;
	wpa_s->assoc_freq = 0;
#ifdef CONFIG_P2P
	if (wpa_s->ap_iface->bss)
		wpa_s->ap_iface->bss[0]->p2p_group = NULL;
	wpas_p2p_group_deinit(wpa_s);
#endif /* CONFIG_P2P */
	hostapd_interface_deinit(wpa_s->ap_iface);
	hostapd_interface_free(wpa_s->ap_iface);
	wpa_s->ap_iface = NULL;
	wpa_drv_deinit_ap(wpa_s);
}


void ap_tx_status(void *ctx, const u8 *addr,
		  const u8 *buf, size_t len, int ack)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	hostapd_tx_status(wpa_s->ap_iface->bss[0], addr, buf, len, ack);
#endif /* NEED_AP_MLME */
}


void ap_rx_from_unknown_sta(void *ctx, const u8 *frame, size_t len)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	const struct ieee80211_hdr *hdr =
		(const struct ieee80211_hdr *) frame;
	u16 fc = le_to_host16(hdr->frame_control);
	ieee802_11_rx_from_unknown(wpa_s->ap_iface->bss[0], hdr->addr2,
				   (fc & (WLAN_FC_TODS | WLAN_FC_FROMDS)) ==
				   (WLAN_FC_TODS | WLAN_FC_FROMDS));
#endif /* NEED_AP_MLME */
}


void ap_mgmt_rx(void *ctx, struct rx_mgmt *rx_mgmt)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	struct hostapd_frame_info fi;
	os_memset(&fi, 0, sizeof(fi));
	fi.datarate = rx_mgmt->datarate;
	fi.ssi_signal = rx_mgmt->ssi_signal;
	ieee802_11_mgmt(wpa_s->ap_iface->bss[0], rx_mgmt->frame,
			rx_mgmt->frame_len, &fi);
#endif /* NEED_AP_MLME */
}


void ap_mgmt_tx_cb(void *ctx, const u8 *buf, size_t len, u16 stype, int ok)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	ieee802_11_mgmt_cb(wpa_s->ap_iface->bss[0], buf, len, stype, ok);
#endif /* NEED_AP_MLME */
}


void wpa_supplicant_ap_rx_eapol(struct wpa_supplicant *wpa_s,
				const u8 *src_addr, const u8 *buf, size_t len)
{
	ieee802_1x_receive(wpa_s->ap_iface->bss[0], src_addr, buf, len);
}


#ifdef CONFIG_WPS

int wpa_supplicant_ap_wps_pbc(struct wpa_supplicant *wpa_s, const u8 *bssid,
			      const u8 *p2p_dev_addr)
{
	if (!wpa_s->ap_iface)
		return -1;
	return hostapd_wps_button_pushed(wpa_s->ap_iface->bss[0],
					 p2p_dev_addr);
}


static int wpa_supplicant_ap_wps_sta_cancel(struct hostapd_data *hapd,
					    struct sta_info *sta, void *ctx)
{
	if (sta && (sta->flags & WLAN_STA_WPS)) {
		ap_sta_deauthenticate(hapd, sta,
				      WLAN_REASON_PREV_AUTH_NOT_VALID);
		wpa_printf(MSG_DEBUG, "WPS: %s: Deauth sta=" MACSTR,
			   __func__, MAC2STR(sta->addr));
		return 1;
	}

	return 0;
}


int wpa_supplicant_ap_wps_cancel(struct wpa_supplicant *wpa_s)
{
	struct wps_registrar *reg;
	int reg_sel = 0, wps_sta = 0;

	if (!wpa_s->ap_iface || !wpa_s->ap_iface->bss[0]->wps)
		return -1;

	reg = wpa_s->ap_iface->bss[0]->wps->registrar;
	reg_sel = wps_registrar_wps_cancel(reg);
	wps_sta = ap_for_each_sta(wpa_s->ap_iface->bss[0],
				  wpa_supplicant_ap_wps_sta_cancel, NULL);

	if (!reg_sel && !wps_sta) {
		wpa_printf(MSG_DEBUG, "No WPS operation in progress at this "
			   "time");
		return -1;
	}

	/*
	 * There are 2 cases to return wps cancel as success:
	 * 1. When wps cancel was initiated but no connection has been
	 *    established with client yet.
	 * 2. Client is in the middle of exchanging WPS messages.
	 */

	return 0;
}


int wpa_supplicant_ap_wps_pin(struct wpa_supplicant *wpa_s, const u8 *bssid,
			      const char *pin, char *buf, size_t buflen)
{
	int ret, ret_len = 0;

	if (!wpa_s->ap_iface)
		return -1;

	if (pin == NULL) {
		unsigned int rpin = wps_generate_pin();
		ret_len = os_snprintf(buf, buflen, "%d", rpin);
		pin = buf;
	} else
		ret_len = os_snprintf(buf, buflen, "%s", pin);

	ret = hostapd_wps_add_pin(wpa_s->ap_iface->bss[0], bssid, "any", pin,
				  0);
	if (ret)
		return -1;
	return ret_len;
}


static void wpas_wps_ap_pin_timeout(void *eloop_data, void *user_ctx)
{
	struct wpa_supplicant *wpa_s = eloop_data;
	wpa_printf(MSG_DEBUG, "WPS: AP PIN timed out");
	wpas_wps_ap_pin_disable(wpa_s);
}


static void wpas_wps_ap_pin_enable(struct wpa_supplicant *wpa_s, int timeout)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL)
		return;
	hapd = wpa_s->ap_iface->bss[0];
	wpa_printf(MSG_DEBUG, "WPS: Enabling AP PIN (timeout=%d)", timeout);
	hapd->ap_pin_failures = 0;
	eloop_cancel_timeout(wpas_wps_ap_pin_timeout, wpa_s, NULL);
	if (timeout > 0)
		eloop_register_timeout(timeout, 0,
				       wpas_wps_ap_pin_timeout, wpa_s, NULL);
}


void wpas_wps_ap_pin_disable(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL)
		return;
	wpa_printf(MSG_DEBUG, "WPS: Disabling AP PIN");
	hapd = wpa_s->ap_iface->bss[0];
	os_free(hapd->conf->ap_pin);
	hapd->conf->ap_pin = NULL;
	eloop_cancel_timeout(wpas_wps_ap_pin_timeout, wpa_s, NULL);
}


const char * wpas_wps_ap_pin_random(struct wpa_supplicant *wpa_s, int timeout)
{
	struct hostapd_data *hapd;
	unsigned int pin;
	char pin_txt[9];

	if (wpa_s->ap_iface == NULL)
		return NULL;
	hapd = wpa_s->ap_iface->bss[0];
	pin = wps_generate_pin();
	os_snprintf(pin_txt, sizeof(pin_txt), "%u", pin);
	os_free(hapd->conf->ap_pin);
	hapd->conf->ap_pin = os_strdup(pin_txt);
	if (hapd->conf->ap_pin == NULL)
		return NULL;
	wpas_wps_ap_pin_enable(wpa_s, timeout);

	return hapd->conf->ap_pin;
}


const char * wpas_wps_ap_pin_get(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;
	if (wpa_s->ap_iface == NULL)
		return NULL;
	hapd = wpa_s->ap_iface->bss[0];
	return hapd->conf->ap_pin;
}


int wpas_wps_ap_pin_set(struct wpa_supplicant *wpa_s, const char *pin,
			int timeout)
{
	struct hostapd_data *hapd;
	char pin_txt[9];
	int ret;

	if (wpa_s->ap_iface == NULL)
		return -1;
	hapd = wpa_s->ap_iface->bss[0];
	ret = os_snprintf(pin_txt, sizeof(pin_txt), "%s", pin);
	if (ret < 0 || ret >= (int) sizeof(pin_txt))
		return -1;
	os_free(hapd->conf->ap_pin);
	hapd->conf->ap_pin = os_strdup(pin_txt);
	if (hapd->conf->ap_pin == NULL)
		return -1;
	wpas_wps_ap_pin_enable(wpa_s, timeout);

	return 0;
}


void wpa_supplicant_ap_pwd_auth_fail(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL)
		return;
	hapd = wpa_s->ap_iface->bss[0];

	/*
	 * Registrar failed to prove its knowledge of the AP PIN. Disable AP
	 * PIN if this happens multiple times to slow down brute force attacks.
	 */
	hapd->ap_pin_failures++;
	wpa_printf(MSG_DEBUG, "WPS: AP PIN authentication failure number %u",
		   hapd->ap_pin_failures);
	if (hapd->ap_pin_failures < 3)
		return;

	wpa_printf(MSG_DEBUG, "WPS: Disable AP PIN");
	hapd->ap_pin_failures = 0;
	os_free(hapd->conf->ap_pin);
	hapd->conf->ap_pin = NULL;
}

#endif /* CONFIG_WPS */


#ifdef CONFIG_CTRL_IFACE

int ap_ctrl_iface_sta_first(struct wpa_supplicant *wpa_s,
			    char *buf, size_t buflen)
{
	if (wpa_s->ap_iface == NULL)
		return -1;
	return hostapd_ctrl_iface_sta_first(wpa_s->ap_iface->bss[0],
					    buf, buflen);
}


int ap_ctrl_iface_sta(struct wpa_supplicant *wpa_s, const char *txtaddr,
		      char *buf, size_t buflen)
{
	if (wpa_s->ap_iface == NULL)
		return -1;
	return hostapd_ctrl_iface_sta(wpa_s->ap_iface->bss[0], txtaddr,
				      buf, buflen);
}


int ap_ctrl_iface_sta_next(struct wpa_supplicant *wpa_s, const char *txtaddr,
			   char *buf, size_t buflen)
{
	if (wpa_s->ap_iface == NULL)
		return -1;
	return hostapd_ctrl_iface_sta_next(wpa_s->ap_iface->bss[0], txtaddr,
					   buf, buflen);
}


int ap_ctrl_iface_wpa_get_status(struct wpa_supplicant *wpa_s, char *buf,
				 size_t buflen, int verbose)
{
	char *pos = buf, *end = buf + buflen;
	int ret;
	struct hostapd_bss_config *conf;

	if (wpa_s->ap_iface == NULL)
		return -1;

	conf = wpa_s->ap_iface->bss[0]->conf;
	if (conf->wpa == 0)
		return 0;

	ret = os_snprintf(pos, end - pos,
			  "pairwise_cipher=%s\n"
			  "group_cipher=%s\n"
			  "key_mgmt=%s\n",
			  wpa_cipher_txt(conf->rsn_pairwise),
			  wpa_cipher_txt(conf->wpa_group),
			  wpa_key_mgmt_txt(conf->wpa_key_mgmt,
					   conf->wpa));
	if (ret < 0 || ret >= end - pos)
		return pos - buf;
	pos += ret;
	return pos - buf;
}

int ap_ctrl_iface_deauthenticate(struct wpa_supplicant *wpa_s,
                                             const char *txtaddr)
{
        u8 addr[ETH_ALEN];
        struct sta_info *sta;
        if (wpa_s->ap_iface == NULL)
		return -1;
        wpa_printf(MSG_DEBUG, "CTRL_IFACE DEAUTHENTICATE %s", txtaddr);
        if (hwaddr_aton(txtaddr, addr))
                return -1;
	hostapd_drv_sta_deauth(wpa_s->ap_iface->bss[0], addr, WLAN_REASON_PREV_AUTH_NOT_VALID);
	sta = ap_get_sta(wpa_s->ap_iface->bss[0], addr);
	if (sta)
		ap_sta_deauthenticate(wpa_s->ap_iface->bss[0], sta,
				      WLAN_REASON_PREV_AUTH_NOT_VALID);
        return 0;
}
int ap_ctrl_iface_disassociate(struct wpa_supplicant *wpa_s,
                                             const char *txtaddr)
{
        u8 addr[ETH_ALEN];
        struct sta_info *sta;
        if (wpa_s->ap_iface == NULL)
		return -1;
        wpa_printf(MSG_DEBUG, "CTRL_IFACE DISASSOCIATE %s", txtaddr);
        if (hwaddr_aton(txtaddr, addr))
                return -1;
	hostapd_drv_sta_disassoc(wpa_s->ap_iface->bss[0], addr, WLAN_REASON_PREV_AUTH_NOT_VALID);
	sta = ap_get_sta(wpa_s->ap_iface->bss[0], addr);
	if (sta)
		ap_sta_disassociate(wpa_s->ap_iface->bss[0], sta,
				      WLAN_REASON_PREV_AUTH_NOT_VALID);
        return 0;
}
#endif /* CONFIG_CTRL_IFACE */


int wpa_supplicant_ap_update_beacon(struct wpa_supplicant *wpa_s)
{
	struct hostapd_iface *iface = wpa_s->ap_iface;
	struct wpa_ssid *ssid = wpa_s->current_ssid;
	struct hostapd_data *hapd;

	if (ssid == NULL || wpa_s->ap_iface == NULL)
		return -1;

#ifdef CONFIG_P2P
	if (ssid->mode == WPAS_MODE_P2P_GO)
		iface->conf->bss[0].p2p = P2P_ENABLED | P2P_GROUP_OWNER;
	else if (ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION)
		iface->conf->bss[0].p2p = P2P_ENABLED | P2P_GROUP_OWNER |
			P2P_GROUP_FORMATION;
#endif /* CONFIG_P2P */

	ieee802_11_set_beacons(iface);
	hapd = iface->bss[0];
	hostapd_set_ap_wps_ie(hapd);

	return 0;
}


int wpa_supplicant_ap_mac_addr_filter(struct wpa_supplicant *wpa_s,
				      const u8 *addr)
{
	struct hostapd_data *hapd;
	struct hostapd_bss_config *conf;

	if (!wpa_s->ap_iface)
		return -1;

	if (addr)
		wpa_printf(MSG_DEBUG, "AP: Set MAC address filter: " MACSTR,
			   MAC2STR(addr));
	else
		wpa_printf(MSG_DEBUG, "AP: Clear MAC address filter");

	hapd = wpa_s->ap_iface->bss[0];
	conf = hapd->conf;

	os_free(conf->accept_mac);
	conf->accept_mac = NULL;
	conf->num_accept_mac = 0;
	os_free(conf->deny_mac);
	conf->deny_mac = NULL;
	conf->num_deny_mac = 0;

	if (addr == NULL) {
		conf->macaddr_acl = ACCEPT_UNLESS_DENIED;
		return 0;
	}

	conf->macaddr_acl = DENY_UNLESS_ACCEPTED;
	conf->accept_mac = os_zalloc(sizeof(struct mac_acl_entry));
	if (conf->accept_mac == NULL)
		return -1;
	os_memcpy(conf->accept_mac[0].addr, addr, ETH_ALEN);
	conf->num_accept_mac = 1;

	return 0;
}
