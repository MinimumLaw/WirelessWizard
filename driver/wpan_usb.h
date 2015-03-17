/*****************************************************************************
 * OAO Radioavionca Personal Radio Comminication Module based on IEEE802.15.4
 *			transmitter with USB interface
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Created by: NTK PIT, Schemotecnics NTC, Developers Laboratory
 *		Alex A. Mihaylov AKA MinimumLaw <minimumlaw@rambler.ru>
 ****************************************************************************/

#ifndef _INCLUDE_WPAN_USB_H_
#define _INCLUDE_WPAN_USB_H_

/*
 * USB WPAN device config and features
 */
struct __attribute__((packed)) wpan_usb_features {
	/* basic IEEE 802.15.4 params */
	uint64_t	extended_addr;
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
	uint32_t	flags;
	uint8_t		last_ed;
	bool		wpan_active;
	bool		lbt_mode;
	bool		pa_ext_sw_ctrl;
};

/*
 * Data structures used for set params of WPAN radio transmitter.
 * REQ_WPAN_SET_HWADDR send IEEE address (MAC address) directly
 * in data section
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
	uint32_t	changed; /* IEEEE_AFILT_CHANGED combination */
	uint64_t	extended_addr;
	uint16_t	short_addr;
	uint16_t	pan_id;
	/* true if PAN coordinator, else false */
	uint8_t		pan_coordinator; 
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

/*
 * Private device data
 */
struct pcrm_usb_dev {
	/* usb specific part */
	struct usb_device	*udev;
	struct usb_interface	*iface;
	/* data endpoints */
	struct usb_anchor	out_submitted;
	uint8_t			out_ep;
	struct urb		*out_urb;
	struct usb_anchor	inp_submitted;
	uint8_t			inp_ep;
	struct urb		*inp_urb;
	uint8_t			*inp_buff;
	/* device present */
	atomic_t		usb_active;
	/* IEEE802.15.4 specific part */
	struct ieee802154_hw	*wpan_hw;
};

#endif
