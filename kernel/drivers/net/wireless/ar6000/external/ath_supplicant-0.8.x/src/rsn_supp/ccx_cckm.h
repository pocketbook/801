/*
 * CCKM definitions for CCX
 * Copyright (c) 2009-2010, embWiSe Technologies
 *
 * Licensed under a proprietary license. 
 * Notifications and licenses are retained for attribution purposes only.
 *
 */

#ifndef CCX_CCKM_H
#define CCX_CCKM_H

void wpa_btk_to_ptk(const u8 *btk, size_t btk_len, const u8 *addr1, u32 rn,
		u8 *ptk, size_t ptk_len);
void wpa_nsk_to_cgk(const u8 *nsk, size_t nsk_len, const u8 *addr1,
		const u8 *addr2, const u8 *nonce1, const u8 *nonce2,
		u8 *cgk, size_t cgk_len);
int cckm_process_response (struct wpa_sm *sm, u8 *bssid);
int ccx_send_report (void *wpa_s);
int cckm_build_request (struct wpa_sm *sm, u8 *bssid, u64 timestamp);

#ifndef CONFIG_NO_WPA
int wpa_sm_set_cckm_params(struct wpa_sm *sm, int reassociate);
int wpa_sm_set_ccx_version_ie(struct wpa_sm *sm, const u8 *ie, size_t len);
int wpa_sm_set_cckm_resp_ie(struct wpa_sm *sm, const u8 *ie, size_t len);
int wpa_sm_set_cckm_req_ie(struct wpa_sm *sm, const u8 *ie, size_t len);
int cckm_fr_is_completed(struct wpa_sm *sm);
int wpa_sm_is_cckm_resp_ie_set (struct wpa_sm *sm);
#else
static inline int int wpa_sm_is_cckm_req_ie_set (struct wpa_sm *sm)
{
	return 0;
}

static inline int int wpa_sm_is_cckm_resp_ie_set (struct wpa_sm *sm)
{
	return 0;
}

static inline int wpa_sm_set_cckm_params(struct wpa_sm *sm, int reassociate)
{
	return -1;
}

static inline int wpa_sm_set_cckm_resp_ie(struct wpa_sm *sm, const u8 *ie,
		size_t len)
{
	return -1;
}

static inline int cckm_fr_is_completed(struct wpa_sm *sm)
{
	return 0;
}
#endif

#endif
