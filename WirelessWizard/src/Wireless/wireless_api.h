#ifndef WIRELESS_API_H_
#define WIRELESS_API_H_

#include <asf.h>
#include "tal.h"
#include "string.h"
#include "ieee_const.h"
#include "tal_helper.h"

#define IEEE802154_MTU		127

/* According to the IEEE 802.15.4 stadard the upper most significant bits of
 * the 32-bit channel bitmaps shall be used as an integer value to specify 32
 * possible channel pages. The lower 27 bits of the channel bit map shall be
 * used as a bit mask to specify channel numbers within a channel page.
 */
#define WPAN_NUM_CHANNELS	27
#define WPAN_NUM_PAGES		32

/*
 * WPAN device config
 */
typedef struct __attribute__((packed)) {
	/* basic IEEE 802.15.4 params */
	uint64_t	ieee_addr;
	uint16_t	short_addr;
	uint16_t	pan_id;
	bool		pan_coordinator;
	uint8_t		page;
	uint8_t		channel;
	uint8_t		tx_power;
	/* CCA params */
	uint8_t		cca_ed_level;
	uint8_t		cca_mode;
	uint8_t		max_frame_retries;
	/* CSMA params */
	uint8_t		csma_mode;
	uint8_t		csma_min_be;
	uint8_t		csma_max_be;
	uint8_t		csma_retries;
	/* Promiscous mode */
#ifdef PROMISCUOUS_MODE
	bool		promisc_mode;
#endif
	/* Hardware specific params */
	uint8_t		last_ed;
	bool		wpan_active;
	bool		lbt_mode;
	bool		pa_ext_sw_ctrl;
} wpan_dev_cfg;

extern wpan_dev_cfg *wdev;
extern uint32_t	channel_page_support[WPAN_NUM_PAGES];
/*
 * Setup requests wValue for device configure
 */
enum {
	REQ_WPAN_START,
	REQ_WPAN_STOP,
	REQ_WPAN_SET_CHANNEL,
	REQ_WPAN_SET_PAGE,
	REQ_WPAN_SET_SHORT_ADDR,
	REQ_WPAN_SET_PAN_COORD,
	REQ_WPAN_SET_PAN_ID,
	REQ_WPAN_SET_HW_ADDR,
	REQ_WPAN_SET_TXPOWER,
	REQ_WPAN_SET_LBT,
	REQ_WPAN_SET_CCA_MODE,
	REQ_WPAN_SET_CCA_ED_LEVEL,
	REQ_WPAN_SET_CSMA_MIN_BE,
	REQ_WPAN_SET_CSMA_MAX_BE,
	REQ_WPAN_SET_CSMA_RETRIES,
	REQ_WPAN_SET_FRAME_RETRIES,
	REQ_WPAN_GET_FEATURES,
	REQ_WPAN_GET_CHANNEL_LIST,
	REQ_WPAN_GET_ED,
#ifdef PROMISCUOUS_MODE
	REQ_WPAN_SET_PROMISC_MODE,
#endif
};

void wireless_init(void);
void init_default_pib(void);
void app_alert(void);

#endif /* WIRELESS_API_H_ */
