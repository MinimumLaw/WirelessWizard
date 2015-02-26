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
	/* Hardware specific params */
	uint32_t	flags; /* IEEE802154_HW_ later in this file */
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
	REQ_WPAN_GET_ED,
	REQ_WPAN_SET_CHANNEL,
	REQ_WPAN_SET_HWADDR_FILT,
	REQ_WPAN_SET_HWADDR,
	REQ_WPAN_SET_TXPOWER,
	REQ_WPAN_SET_LBT,
	REQ_WPAN_SET_CCA_MODE,
	REQ_WPAN_SET_CCA_ED_LEVEL,
	REQ_WPAN_SET_CSMA_PARAMS,
	REQ_WPAN_SET_FRAME_RETRIES,
	REQ_WPAN_GET_FEATURES,
	REQ_WPAN_GET_CHANNEL_LIST,
};

/*
 * Data structures used for set params of WPAN radio transmitter.
 * REQ_WPAN_SET_HWADDR send IEEE address (MAC address) directly in data section
 */

/* Dummy write data.
 * Used for REQ_WPAN_START, REQ_WPAN_STOP
 */
struct __attribute__((packed)) wpan_dummy_write {
	uint8_t	dummy;
};

/* Set current radio frequency
 * Used for REQ_WPAN_SET_CHANNEL
 */
struct __attribute__((packed)) wpan_channel_data {
	uint8_t	channel;
	uint8_t page;
};

/* Set or update hardware filters
 * Used for REQ_WPAN_SET_HWADDR_FILT
 */
struct __attribute__((packed)) wpan_hw_addr_filt {
	uint32_t	changed; /* IEEEE_AFILT_..._CHANGED combination */
	uint64_t	ieee_addr;
	uint16_t	short_addr;
	uint16_t	pan_id;
	uint8_t		pan_coordinator; /* true if PAN coordinator, else false */
};

/* Set current transmit power
 * Used for REQ_WPAN_SET_TXPOWER
 */
struct __attribute__((packed)) wpan_tx_power {
	uint8_t	tx_power;
};

/* Set listen before talk mode
 * Used for REQ_WPAN_SET_LBT
 */
struct __attribute__((packed)) wpan_lbt {
	uint8_t	mode;
};

/* Set CCA mode
 * Used for REQ_WPAN_SET_CCA_MODE
 */
struct __attribute__((packed)) wpan_cca {
	uint8_t	mode;
};

/* Set CCA energy detection threshold
 * Used for REQ_WPAN_SET_CCA_ED_LEVEL
 */
struct __attribute__((packed)) wpan_cca_threshold {
	uint8_t level;
};

/* Set frame retries count
 * Used for REQ_WPAN_SET_FRAME_RETRIES
 */
struct __attribute__((packed)) wpan_frame_retries {
	uint8_t count;
};

/* FixMe: M.B. change API and combine REQ_WPAN_SET_CCA_MODE,
 *	REQ_WPAN_SET_CCA_ED_LEVEL and REQ_WPAN_SET_RETRIES
 *	int single REQ_WPAN_CCA_PARAMS with
 * struct __attribute__((packed)) wpan_cca_params {
 *		uint8_t	mode;
 *		uint8_t	threshold;
 *		uint8_t	retries;
 *	};
 */

/* Set WPAN device CSMA params
 * Used for REQ_WPAN_SET_CSMA_PARAMS
 */
struct __attribute__((packed)) wpan_csma_params {
	uint8_t min_be;
	uint8_t max_be;
	uint8_t retries;
};

/* FixMe: M.B.
 * typedef union {
 *		struct wpan_tx_power		power;
 *		struct wpan_hw_addr_filt	filt;
 *		...
 *		struct wpan_csma_params		csma;
 * } wpan_control_request;
 * and use them in params code???
 */

/**************************************************
 * Got this from os linux include/net/mac802154.h *
 **************************************************/
/* The following flags are used to indicate changed address settings from
 * the stack to the hardware.
 */

/* indicates that the Short Address changed */
#define IEEE802515_AFILT_SADDR_CHANGED			0x00000001
/* indicates that the IEEE Address changed */
#define IEEE802515_AFILT_IEEEADDR_CHANGED		0x00000002
/* indicates that the PAN ID changed */
#define IEEE802515_AFILT_PANID_CHANGED			0x00000004
/* indicates that PAN Coordinator status changed */
#define IEEE802515_AFILT_PANC_CHANGED			0x00000008

/* Checksum is in hardware and is omitted from a packet
 *
 * These following flags are used to indicate hardware capabilities to
 * the stack. Generally, flags here should have their meaning
 * done in a way that the simplest hardware doesn't need setting
 * any particular flags. There are some exceptions to this rule,
 * however, so you are advised to review these flags carefully.
 */

/* Indicates that receiver omits FCS and xmitter will add FCS on it's own. */
#define	IEEE802154_HW_OMIT_CKSUM		0x00000001
/* Indicates that receiver will autorespond with ACK frames. */
#define	IEEE802154_HW_AACK				0x00000002
/* Indicates that transceiver will support transmit power setting. */
#define	IEEE802154_HW_TXPOWER			0x00000004
/* Indicates that transceiver will support listen before transmit. */
#define	IEEE802154_HW_LBT				0x00000008
/* Indicates that transceiver will support cca mode setting. */
#define	IEEE802154_HW_CCA_MODE			0x00000010
/* Indicates that transceiver will support cca ed level setting. */
#define	IEEE802154_HW_CCA_ED_LEVEL		0x00000020
/* Indicates that transceiver will support csma (max_be, min_be, csma retries)
 * settings. */
#define	IEEE802154_HW_CSMA_PARAMS		0x00000040
/* Indicates that transceiver will support ARET frame retries setting. */
#define	IEEE802154_HW_FRAME_RETRIES		0x00000080

/* This groups the most common CSMA support fields into one. */
#define IEEE802154_HW_CSMA		(IEEE802154_HW_CCA_MODE | \
					 IEEE802154_HW_CCA_ED_LEVEL | \
					 IEEE802154_HW_CSMA_PARAMS | \
					 IEEE802154_HW_FRAME_RETRIES)


void wireless_init(void);
void init_data_reception(void);
void app_alert(void);

#endif /* WIRELESS_API_H_ */