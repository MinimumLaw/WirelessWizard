/*****************************************************************************
 * OAO Radioavionca Personal Radioc Comminication Module based on IEEE802.15.4
 *			transmitter with USB interface
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Created by: NTK PIT, Schemotecnics NTC, Developers Laboratory
 *		Alex A. Mihaylov AKA MinimumLaw <minimumlaw@rambler.ru>
 ****************************************************************************/

/* kernel module (driver) specific */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
/* usb specific */
#include <linux/usb.h>
#include <linux/errno.h>
/* network device specific */
#include <linux/skbuff.h>
/* IEEE802.15.4 specific */
#include <net/ieee802154.h>
#include <net/mac802154.h>
#include <net/wpan-phy.h>
/* local device data */
#include "wpan_usb.h"

/*
 * USB device driver specific constants and tables
 */
#define WPAN_USB_VID		0xADCA
#define WPAN_USB_PID		0x1001
#define WPAN_USB_CLASS		0xFF
#define WPAN_USB_SUBCLASS	0xFF
#define WPAN_USB_PROTO		0xFF

#define CTRL_VENDOR_GET	\
	(USB_TYPE_VENDOR | USB_RECIP_INTERFACE | USB_DIR_IN)
#define CTRL_VENDOR_SET	\
	(USB_TYPE_VENDOR | USB_RECIP_INTERFACE | USB_DIR_OUT)


/*
 * wValue for WPAN USB device
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
 * IEEE 802.15.4 specific functions
 */

static void wpan_usb_receive_callback(struct urb* urb)
{
	struct pcrm_usb_dev	*dev = urb->context;
	struct usb_interface	*iface = dev->iface;
	struct sk_buff		*skb;
	uint8_t	frame_len = *((uint8_t *)urb->transfer_buffer);
	uint8_t	*frame	= (uint8_t *)urb->transfer_buffer + 1;
	uint8_t lqi,ed;

	/* Dump received packet */
	dev_err(&iface->dev,"Receive USB %d bytes", urb->actual_length);
	print_hex_dump(KERN_ERR, "URB data:", DUMP_PREFIX_OFFSET,
		16,1, urb->transfer_buffer, urb->actual_length, 1);

	/* check for usb device present */
	if(!atomic_read(&dev->usb_active))
		return;

	if (urb->status == 0) { /* data received */

		if(frame_len < 2) {
			dev_err(&iface->dev,
				"to short frame recv (usb) - ignore\n");
			goto restart_transfer;
		}

		if(frame_len > IEEE802154_MTU) { /* to long */
			lqi = 0;
			frame_len = IEEE802154_MTU;
			dev_err(&iface->dev,
				"to long frame recv (usb) - trim!\n");
		} else {
			lqi = frame[frame_len];
			ed = frame[frame_len+1];
		}

		/* allocate skb */
		skb = alloc_skb(IEEE802154_MTU, GFP_ATOMIC);
		if(!skb) {
		    dev_err(&iface->dev,"SKB alloc error!\n");
		    goto restart_transfer;
		};

		/* fill skb */
		memcpy(skb_put(skb,frame_len), frame, frame_len);

		/* skip CRC in receive frame */
		skb_trim(skb, frame_len - 2);

		dev_err(&iface->dev,"SKB LQI = 0x%02X, ED = 0x%02X\n",
			lqi, ed);

		/* submitt skb to device */
		ieee802154_rx_irqsafe(dev->wpan_dev,skb,lqi);
	} else
		dev_err(&iface->dev,
			"Error receive wpan data frame USB!\n");
restart_transfer:
	/* restart data receive */
	usb_fill_bulk_urb(urb, dev->udev,
		usb_rcvbulkpipe(dev->udev, dev->inp_ep),
		urb->transfer_buffer, urb->transfer_buffer_length,
		wpan_usb_receive_callback, dev);
	dev->inp_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_anchor_urb(urb, &dev->inp_submitted);

	usb_submit_urb(urb, GFP_KERNEL);
}

/* start: Handler that 802.15.4 module calls for device initialization.
    This function is called before the first interface is attached. */
static int wpan_usb_start(struct ieee802154_dev *wpan_dev)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_dummy_write	dummy;
	int ret;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_START,	/* wValue */
		0,		/* wIndex */
		&dummy,		/* pData */
		sizeof(dummy),	/* wSize */
		HZ);		/* tOut */

	if (ret<sizeof(dummy)) {
		dev_err(&iface->dev,"Error send WPAN start "
			"command (%d)!\n", ret);
		return -ENOTSUPP;
	};

	/* allocate incommint urb */
	dev->inp_urb = usb_alloc_urb(0, GFP_KERNEL);

	if(!dev->inp_urb) {
		dev_err(&iface->dev,"Error alocate USB input "
			"urb!\n");
		return -ENOMEM;
	};

	/* allocate buffer for input transfer */
	dev->inp_buff = usb_alloc_coherent(dev->udev,
		IEEE802154_MTU + 2, GFP_KERNEL,
		&dev->inp_urb->transfer_dma);

	if(!dev->inp_buff) {
		dev_err(&iface->dev,"Error alocate USB input "
			"data buffer!\n");
		return -ENOMEM;
	};

	usb_fill_bulk_urb(dev->inp_urb, dev->udev,
		usb_rcvbulkpipe(dev->udev, dev->inp_ep),
		dev->inp_buff, IEEE802154_MTU + 2,
		wpan_usb_receive_callback, dev);
	dev->inp_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_anchor_urb(dev->inp_urb, &dev->inp_submitted);

	ret = usb_submit_urb(dev->inp_urb, GFP_KERNEL);

	return ret;
}

/* stop:Handler that 802.15.4 module calls for device cleanup.
    This function is called after the last interface is removed. */
static void wpan_usb_stop(struct ieee802154_dev *wpan_dev)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_dummy_write	dummy;
	int ret;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_STOP,	/* wValue */
		0,		/* wIndex */
		&dummy,		/* pData */
		sizeof(dummy),	/* wSize */
		HZ/* tOut */);

	if (ret<sizeof(dummy)) {
		dev_err(&iface->dev,"Error send WPAN stop "
		    "command (%d)!\n", ret);
	};

	/* free receive urb */
	usb_free_urb(dev->inp_urb);
}

static void usb_write_cb(struct urb *urb)
{
	struct pcrm_usb_dev	*dev = urb->context;
	struct usb_interface	*iface = dev->iface;

	if(urb->status)
		dev_err(&iface->dev,"Error transfer wpan data!\n");

	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);
}

/* xmit: Handler that 802.15.4 module calls for each transmitted frame.
	skb cntains the buffer starting from the IEEE 802.15.4 header.
	The low-level driver should send the frame based on available
	configuration.
	This function should return zero or negative errno. Called with
	pib_lock held. */
static int wpan_usb_xmit(struct ieee802154_dev *wpan_dev, struct sk_buff *skb)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	int 			retval;
	char* 			buff;

	/* check for usb device present */
	if(!atomic_read(&dev->usb_active))
		return -ENODEV;

	dev->out_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(!dev->out_urb) {
		dev_err(&iface->dev,"Failed to allocate URB!\n");
		return -ENOMEM;
	};

	/* skb->len data bytes + 1 byte len + 2 byte CRC  */
	buff = usb_alloc_coherent(dev->udev, skb->len + 1 + 2,
				GFP_KERNEL, &dev->out_urb->transfer_dma);
	if(!buff) {
		dev_err(&iface->dev,"Failed to allocate URB"
			" data buffer!\n");
		retval = -ENOMEM;
		goto buff_error;
	};

	/* fill len (with CRC bytes) and data, CRC calculated by transmitter */
	buff[0] = skb->len + 2;
	memcpy(buff + 1, skb->data, skb->len);

	/* Dump sending packet */
	dev_err(&iface->dev,"Send SKB %d bytes", skb->len + 1);
	print_hex_dump(KERN_ERR, "SKB data:", DUMP_PREFIX_OFFSET,
		16,1, buff, skb->len + 1, 1);

	/* Fill packet with additional len */
	usb_fill_bulk_urb(dev->out_urb, dev->udev,
			usb_sndbulkpipe(dev->udev, dev->out_ep),
			buff, skb->len + 1, usb_write_cb, dev);
	dev->out_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_anchor_urb(dev->out_urb, &dev->out_submitted);

	retval = usb_submit_urb(dev->out_urb, GFP_KERNEL);

	if(retval) {
		dev_err(&iface->dev, "Failed to submit URB!\n");
		goto submit_error;
	}

	usb_free_urb(dev->out_urb);

	return retval;

submit_error:
	if(dev->out_urb)
		usb_free_coherent(dev->udev, skb->len,
				buff, dev->out_urb->transfer_dma);
buff_error:
	if(dev->out_urb)
		usb_free_urb(dev->out_urb);

	return retval;
};

/* ed:	Handler that 802.15.4 module calls for Energy Detection.
	This function should place the value for detected energy
	(usually device-dependant) in the level pointer and return
	either zero or negative errno. Called with pib_lock held. */
static int wpan_usb_ed(struct ieee802154_dev *wpan_dev, u8* level)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	int ret;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_GET,/* bRequestType */
		REQ_WPAN_GET_ED,/* wValue */
		0,		/* wIndex */
		level,		/* pData */
		1,		/* wSize */
		HZ);		/* tOut */
	if(ret < 1) {
		dev_err(&iface->dev,"Error receive ED from WPAN!");
		return -ENOTSUPP;
	};
	return 0;
}

/* set_channel:
	Set radio for listening on specific channel.
	Set the device for listening on specified channel.
	Returns either zero, or negative errno. Called with pib_lock held. */
static int wpan_usb_set_channel(struct ieee802154_dev *wpan_dev,
				int page, int channel)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_channel_data set_channel;
	int ret;

	set_channel.page = page;
	set_channel.channel = channel;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_CHANNEL,/* wValue */
		0,		/* wIndex */
		&set_channel,	/* pData */
		sizeof(set_channel),/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(set_channel)) {
		dev_err(&iface->dev,"Error set WPAN device channel!");
		return -ENOTSUPP;
	};
	return 0;
}

/* set_hw_addr_filt:
	Set radio for listening on specific address.
	Set the device for listening on specified address.
	Returns either zero, or negative errno. */
static int wpan_usb_set_hw_addr_filt(struct ieee802154_dev *wpan_dev,
				struct ieee802154_hw_addr_filt *filt,
				unsigned long changed)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_hw_addr_filt filter;
	int ret;

	filter.changed = changed;
	filter.ieee_addr = filt->ieee_addr;
	filter.short_addr = filt->short_addr;
	filter.pan_id = filt->pan_id;
	filter.pan_coordinator = filt->pan_coord;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_HWADDR_FILT,/* wValue */
		0,		/* wIndex */
		&filter,	/* pData */
		sizeof(filter),	/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(filter)) {
		dev_err(&iface->dev,"Error send hardware addreess"
			" filter to WPAN device!\n");
		return -ENOTSUPP;
	};
	return 0;
}

/* ieee_addr:
	FixMe: more info about this helper. M.B. set hw (ieee) address???
	Returns either zero, or negative errno. */
static int wpan_usb_ieee_addr(struct ieee802154_dev *wpan_dev, __le64 addr)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	int ret;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_HWADDR,/* wValue */
		0,		/* wIndex */
		&addr,		/* pData */
		sizeof(__le64),	/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(__le64)) {
		dev_err(&iface->dev,"Error set WPAN device"
			" hardware addreess!\n");
		return -ENOTSUPP;
	};
	return 0;
}

/* set_txpower:
	Set radio transmit power in dB. Called with pib_lock held.
	Returns either zero, or negative errno. */
static int wpan_usb_set_txpower(struct ieee802154_dev *wpan_dev, int db)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_tx_power	power;
	int ret;

	power.tx_power = db;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_TXPOWER,/* wValue */
		0,		/* wIndex */
		&power,		/* pData */
		sizeof(power),	/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(power)) {
		dev_err(&iface->dev,"Error set WPAN device"
			" transmitt power!\n");
		return -ENOTSUPP;
	};
	return 0;
}

/* set_lbt:
	Enables or disables listen before talk on the device. Called with
	pib_lock held.
	Returns either zero, or negative errno. */
static int wpan_usb_set_lbt(struct ieee802154_dev *wpan_dev, bool on)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_lbt		lbt;
	int ret;

	if(on == true)
	    lbt.mode = 1;
	else
	    lbt.mode = 0;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_LBT,/* wValue */
		0,		/* wIndex */
		&lbt,		/* pData */
		sizeof(lbt),	/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(lbt)) {
		dev_err(&iface->dev,
			"Error set WPAN device listen before talk mode!\n");
		return -ENOTSUPP;
	};
	return 0;
}

/* set_cca_mode:
	Sets the CCA mode used by the device. Called with pib_lock held.
	Returns either zero, or negative errno. */
static int wpan_usb_set_cca_mode(struct ieee802154_dev *wpan_dev, u8 mode)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_cca		cca;
	int ret;

	cca.mode = mode;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_CCA_MODE,/* wValue */
		0,		/* wIndex */
		&cca,		/* pData */
		sizeof(cca),	/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(cca)) {
		dev_err(&iface->dev, "Error set WPAN device CCA mode!\n");
		return -ENOTSUPP;
	};
	return 0;
}

/* set_cca_ed_level:
	Sets the CCA energy detection threshold in dBm. Called with pib_lock
	held.
	Returns either zero, or negative errno. */
static int wpan_usb_set_cca_ed_level(struct ieee802154_dev *wpan_dev,
				s32 level)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_cca_threshold threshold;
	int ret;

	threshold.level = level;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_CCA_ED_LEVEL,/* wValue */
		0,		/* wIndex */
		&threshold,	/* pData */
		sizeof(threshold),/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(threshold)) {
		dev_err(&iface->dev, "Error set WPAN device CCA ED level!\n");
		return -ENOTSUPP;
	};
	return 0;
}

/* set_csma_params:
	Sets the CSMA parameter set for the PHY. Called with pib_lock held.
	Returns either zero, or negative errno. */
static int wpan_usb_set_csma_params(struct ieee802154_dev *wpan_dev,
				u8 min_be, u8 max_be, u8 retries)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_csma_params	csma;
	int ret;

	csma.min_be = min_be;
	csma.max_be = max_be;
	csma.retries = retries;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_CSMA_PARAMS,/* wValue */
		0,		/* wIndex */
		&csma,		/* pData */
		sizeof(csma),	/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(csma)) {
		dev_err(&iface->dev, "Error set WPAN device"
			" CSMA parameters!\n");
		return -ENOTSUPP;
	};
	return 0;
}

/* set_frame_retries:
	Sets the retransmission attempt limit. Called with pib_lock held.
	Returns either zero, or negative errno. */
static int wpan_usb_set_frame_retries(struct ieee802154_dev *wpan_dev,
				s8 retries)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_frame_retries cca_retries;
	int ret;

	cca_retries.count = retries;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0,		/* bReqiest */
		CTRL_VENDOR_SET,/* bRequestType */
		REQ_WPAN_SET_FRAME_RETRIES,/* wValue */
		0,		/* wIndex */
		&cca_retries,	/* pData */
		sizeof(cca_retries),/* wSize */
		HZ);		/* tOut */
	if(ret < sizeof(cca_retries)) {
		dev_err(&iface->dev, "Error set WPAN device"
			" frame retries number!\n");
		return -ENOTSUPP;
	};
	return 0;
}

static struct ieee802154_ops wpan_ops = {
	.owner = THIS_MODULE,
	.start = wpan_usb_start,
	.stop = wpan_usb_stop,
	.xmit = wpan_usb_xmit,
	.ed = wpan_usb_ed,
	.set_channel = wpan_usb_set_channel,
	.set_hw_addr_filt = wpan_usb_set_hw_addr_filt,
	.ieee_addr = wpan_usb_ieee_addr,
	.set_txpower = wpan_usb_set_txpower,
	.set_lbt = wpan_usb_set_lbt,
	.set_cca_mode = wpan_usb_set_cca_mode,
	.set_cca_ed_level = wpan_usb_set_cca_ed_level,
	.set_csma_params = wpan_usb_set_csma_params,
	.set_frame_retries = wpan_usb_set_frame_retries,
};

/*
 * USB WPAN device configuration and feature detect
 */
static int pcrm_usb_feature_detect(struct ieee802154_dev *wpan_dev)
{
	struct pcrm_usb_dev	*dev = wpan_dev->priv;
	struct usb_interface	*iface = dev->iface;
	struct usb_device	*udev = dev->udev;
	struct wpan_dev_features features;
	int ret;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
		0,			/* bReqiest */
		CTRL_VENDOR_GET,	/* bRequestType */
		REQ_WPAN_GET_FEATURES,	/* wValue */
		0,			/* wIndex */
		&features,		/* pData */
		sizeof(struct wpan_dev_features), /* wSize */
		HZ);			/* tOut */

	dev_dbg(&iface->dev,"Debug get_feature (%d) size=%d\n",
	    ret, (int)sizeof(struct wpan_dev_features));
	dev_dbg(&iface->dev,"Debug: short = 0x%04X, panid=0x%04X, coord=%d\n",
	    features.short_addr, features.pan_id, features.pan_coordinator);

	if (ret<0)
	    return ret;
	WARN_ON(ret != sizeof(struct wpan_dev_features));

	/* features analyze and device setup */
	wpan_dev->extra_tx_headroom = 0;
	/* FixMe: M.B. fix later... */
	wpan_dev->flags = features.flags;
	wpan_dev->hw_filt.ieee_addr = features.ieee_addr;
	wpan_dev->hw_filt.short_addr = features.short_addr;
	wpan_dev->hw_filt.pan_id = features.pan_id;
	wpan_dev->hw_filt.pan_coord = features.pan_coordinator;
	wpan_dev->phy->current_channel = features.channel;
	wpan_dev->phy->current_page = features.page;
	wpan_dev->phy->transmit_power = features.tx_power;
	wpan_dev->phy->frame_retries = features.max_frame_retries;
	wpan_dev->phy->lbt = features.lbt_mode;
	/* CCA */
	wpan_dev->phy->cca_mode = features.cca_mode;
	wpan_dev->phy->cca_ed_level = features.cca_ed_level;
	/* CSMA */
	wpan_dev->phy->min_be = features.csma_min_be;
	wpan_dev->phy->max_be = features.csma_max_be;
	wpan_dev->phy->csma_retries = features.csma_retries;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
		0,			/* bReqiest */
		CTRL_VENDOR_GET,	/* bRequestType */
		REQ_WPAN_GET_CHANNEL_LIST,/* wValue */
		0,			/* wIndex */
		wpan_dev->phy->channels_supported,/* pData */
		sizeof(wpan_dev->phy->channels_supported), /* wSize */
		HZ);			/* tOut */

	dev_dbg(&iface->dev,"Debug channel_supported (%d) size=%d\n",
	    ret, (int)sizeof(wpan_dev->phy->channels_supported));

	if (ret<0)
	    return ret;
	WARN_ON(ret != sizeof(wpan_dev->phy->channels_supported));

	return 0;
}

/*
 * USB device service functions (probe, remove, etc...)
 */

static int pcrm_usb_probe(struct usb_interface *interface,
				const struct usb_device_id *id)
{
	struct pcrm_usb_dev		*dev;
	struct ieee802154_dev		*wpan_dev;
	struct usb_host_interface	*iface_descr;
	struct usb_endpoint_descriptor	*endpoint;
	int i;

	/* allocate IEEE802.15.4 device */
	wpan_dev = ieee802154_alloc_device(sizeof(struct pcrm_usb_dev),
			&wpan_ops);

	if(!wpan_dev) {
		dev_err(&interface->dev,"Error allocate IEEE 802.15.4"
			" interface special device!\n");
		return -ENOMEM;
	};

	/* Fill private info with USB device data and allow usb trafic */
	dev = wpan_dev->priv;
	dev->wpan_dev = wpan_dev;
	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->iface = interface;
	usb_set_intfdata(interface, dev);
	atomic_set(&dev->usb_active, 1);

	/* switch interface 0 to alt 1 (Atmel ASF vendor specific) */
	usb_set_interface(dev->udev, 0, 1);

	/* Single alt setting */
	iface_descr = interface->cur_altsetting;

	/* find data in and out endpoint */
	for(i=0; i < iface_descr->desc.bNumEndpoints; ++i) {
		endpoint = &iface_descr->endpoint[i].desc;

		if(!dev->out_ep && usb_endpoint_is_bulk_out(endpoint)) {
			dev->out_ep = endpoint->bEndpointAddress;
		}
		if(!dev->inp_ep && usb_endpoint_is_bulk_in(endpoint)) {
			dev->inp_ep = endpoint->bEndpointAddress;
		}
	}

	if(!dev->out_ep) {
		dev_err(&interface->dev,"No output endpoint found!\n");
		/* FixMe: error handling!!! */
		return -ENODEV;
	}

	if(!dev->inp_ep) {
		dev_err(&interface->dev,"No input endpoint found!\n");
		/* FixMe: error handling!!! */
		return -ENODEV;
	}

	init_usb_anchor(&dev->out_submitted);
	init_usb_anchor(&dev->inp_submitted);

	/* Check USB WPAN device feature */
	if(pcrm_usb_feature_detect(wpan_dev)) {
		dev_err(&interface->dev,"Error get IEEE 802.15.4"
			" device features!\n");
		return -ENOTSUPP;
	}

	/* OK now register IEEE 802.15.4 device */
	if (ieee802154_register_device(wpan_dev)) {
		dev_err(&interface->dev,"Error register configured"
			" IEEE 802.15.4 device!\n");
		ieee802154_free_device(wpan_dev);
		return -ENOTSUPP;
	}

	/* Nice, let's exit success */
	dev_info(&interface->dev,
			"PCRM usb device attached!\n");
	return 0;
}

static void pcrm_usb_disconnect(struct usb_interface *interface)
{
	struct pcrm_usb_dev	*dev;
	struct ieee802154_dev	*wpan_dev;

	dev = usb_get_intfdata(interface);

	/* disable usb trafic */
	atomic_set(&dev->usb_active, 0);

	/* got wpan device and free them */
	wpan_dev = dev->wpan_dev;
	ieee802154_unregister_device(wpan_dev);
	ieee802154_free_device(wpan_dev);

	usb_set_intfdata(interface, NULL);

	dev_info(&interface->dev,
		"PCRM usb device disconnected!\n");
}

static const struct usb_device_id wpan_usb_table[] = {
	{USB_DEVICE_AND_INTERFACE_INFO(
		WPAN_USB_VID,
		WPAN_USB_PID,
		WPAN_USB_CLASS,
		WPAN_USB_SUBCLASS,
		WPAN_USB_PROTO)
	},
	{}
};

static struct usb_driver pcrm_driver = {
	.name = "pcrm_usb",
	.probe = pcrm_usb_probe,
	.disconnect = pcrm_usb_disconnect,
	.id_table = wpan_usb_table,
	.supports_autosuspend = 1,
};

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0))

module_usb_driver(pcrm_driver);

#else

static int __init pcrm_init(void)
{
	return usb_register(&pcrm_driver);
}

static void __exit pcrm_cleanup(void)
{
	usb_deregister(&pcrm_driver);
}

module_init(pcrm_init);
module_exit(pcrm_cleanup);

#endif

MODULE_DEVICE_TABLE(usb, wpan_usb_table);
MODULE_DESCRIPTION(
"OAO Radioavionica Personal Radio Communication Module device.");
MODULE_AUTHOR("Alex A. Mihaylov AKA MinimumLaw <minimumlaw@rambler.ru>");
MODULE_LICENSE("GPL");
