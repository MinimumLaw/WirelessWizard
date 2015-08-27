/*****************************************************************************
 * OAO Radioavionca Personal Radio Comminication Module based on IEEE802.15.4
 *			transmitter with USB interface
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Created by: NTK PIT, Schemotecnics NTC, Developers Laboratory
 *		Alex A. Mihaylov AKA MinimumLaw <minimumlaw@rambler.ru>
 ****************************************************************************/

#ifndef _INCLUDE_WPAN_USB_H_
#define _INCLUDE_WPAN_USB_H_

#define _DUMP_SKB_AND_URB_
#define _USE_PROMISC_MODE_

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
#ifdef _USE_PROMISC_MODE_
	/* Promiscuous mode */
	bool		promisc_mode;
#endif
	/* Hardware specific params */
	uint8_t		last_ed;
	bool		wpan_active;
	bool		lbt_mode;
	bool		pa_ext_sw_ctrl;
};

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
