/**
 * \file main.c
 *
 * Created: 25.02.2015 12:56:53
 * Author: alex
 *
 * \brief  Main file of Wireless Project generated by Project Wizard
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 */

/**
 * \page license License
 * Copyright(c) 2014, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === INCLUDES ============================================================ */

#include "tal.h"
#include "wireless_api.h"

/* === PROTOTYPES ====================================================== */

/**
 * \brief Application init
 */
static void app_init(void);


/**
 * \brief Application task
 */
static void app_task(void);

void prcm_usb_data_sended(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep);
void prcm_usb_data_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep);


/* === IMPLEMENTATION ====================================================== */

/**
 * \brief Main function of the application
 */
int main(void)
{	
	/* Initialize the Wireless Module */
	wireless_init();    
	
	/*The Modules selected in the wizard are initialized here */
	modules_init();

	app_init();

	while (1)
	{
		/* These methods are called to perform the default tasks of the MAC Stack */
		pal_task();
		tal_task();
		/* Custom (User) tasks are performed here. */
		app_task();
	}
}

queue_t	qfUSBtWPAN,qfWPANtUSB;

/**
 * \brief Application init
 */
 void app_init(void)
{
	qmm_queue_init(&qfUSBtWPAN);
	qmm_queue_init(&qfWPANtUSB);
}


// TODO (Code Writing) USB data send callback write
void prcm_usb_data_sended(udd_ep_status_t status,
			iram_size_t nb_transfered, udd_ep_id_t ep)
{
	if(UDD_EP_TRANSFER_OK == status) {
		/* Handle success send frame to host */
	} else {
		/* Handle unsuccess send frame to host */
	}
}

/**
 * \brief Application task
 */
uint8_t receive_psdu[128 + 1];
 void app_task(void)
{
	buffer_t *usb_to_wpan;
	buffer_t *wpan_to_usb;
	
	usb_to_wpan = qmm_queue_remove(&qfUSBtWPAN, NULL);
	if(usb_to_wpan != NULL) {
		frame_info_t *frame;
		
		frame = malloc(sizeof(frame_info_t));
		
		if(frame != NULL) {
			frame->mpdu = usb_to_wpan->body;
			tal_tx_frame(frame, CSMA_SLOTTED, true);
		} else {
			/* handle no memory for RX frame - simple drop data buffer */
			bmm_buffer_free(usb_to_wpan);
		}
	}
	
	wpan_to_usb = qmm_queue_remove(&qfWPANtUSB, NULL);
	if(wpan_to_usb != NULL) {
		uint8_t usb_msg_size = wpan_to_usb->body[0] + 3; /* Frame + LQI */
		
		memcpy(receive_psdu, wpan_to_usb->body, usb_msg_size);
		udi_vendor_bulk_in_run(receive_psdu, usb_msg_size, prcm_usb_data_sended);
		bmm_buffer_free(wpan_to_usb);
	}
}

/**
 *
 * \brief This method (callback) is called when a frame has been transmitted by the transceiver
 * \param status  Status of frame transmission i.e MAC_SUCCESS,MAC_NO_ACK,CHANNEL_ACCESS_FAILURE etc
 * \param frame pointer to the transmitted frame
 */
void tal_tx_frame_done_cb(retval_t status, frame_info_t *frame)
{
	/*Perform application tasks when a frame is transmitted here*/
	
	// Free-up the buffer which was used for transmission once the frame is extracted.
	bmm_buffer_free(frame->buffer_header);
	// Free-up allocated frame_info_t* structure
	free(frame);
}

/**
 *
 * \brief This method (callback) is called when a frame is received by the transceiver
 * \param frame pointer to the received frame
 *
 */
void tal_rx_frame_cb(frame_info_t *frame)
{
	buffer_t *rcv_frame_buffer;
	uint16_t total_len = frame->mpdu[0] + 3; /* FCS in frame len, LQI after FCS */
	
	/* Allocate buffer for received frame */
	rcv_frame_buffer = bmm_buffer_alloc(total_len);
	
	if (rcv_frame_buffer != NULL) {
		/* copy data from incomming to outcomming buffer */
		memcpy(rcv_frame_buffer->body,frame->mpdu,total_len);
		/* and put them to USB output queue */
		qmm_queue_append(&qfWPANtUSB, rcv_frame_buffer);
	}
	
	// Free-up the buffer which was used for reception once the frame is extracted.
	bmm_buffer_free(frame->buffer_header);
}

uint8_t usb_out_buff[128];
void prcm_usb_data_received(udd_ep_status_t status,
				iram_size_t nb_transfered, udd_ep_id_t ep)
{
	buffer_t *bmm_buff;
	
	if (status == UDD_EP_TRANSFER_OK) {
		usb_out_buff[0] -= 2; /* FSC counted, but not received */
		uint8_t msg_size = usb_out_buff[0];
		bmm_buff = bmm_buffer_alloc(msg_size);
		if(bmm_buff != NULL) {
			memcpy(bmm_buff->body,usb_out_buff, msg_size);
			qmm_queue_append(&qfUSBtWPAN, bmm_buff);
		} else {
			/* Handle no memory */
		}
	} else {
		/* handle fail transfer - simple restart transfer 
		udi_vendor_bulk_out_run(usb_out_buff, sizeof(usb_out_buff),&prcm_usb_data_received); */
	}
	/* restart transfer 
	udi_vendor_bulk_out_run(usb_out_buff, sizeof(usb_out_buff),&prcm_usb_data_received); */
}

bool prcm_device_enable(void)
{
	/* start receive with default params */
	init_data_reception();
	
	udi_vendor_bulk_out_run(usb_out_buff, sizeof(usb_out_buff),&prcm_usb_data_received);
	return true;
}

// TODO (Code Writing) Disable USB device callback write
void prcm_device_disable(void)
{
	/* Noting at this time */
}

// TODO (Code Writing) PRCM setup out handling write
bool prcm_setup_out(void)
{
	bool ret = false;
	
	switch(udd_g_ctrlreq.req.wValue){
		case REQ_WPAN_START:
			break;
		case REQ_WPAN_STOP:
			break;
		case REQ_WPAN_SET_CHANNEL:
			break;
		case REQ_WPAN_SET_HWADDR_FILT:
			break;
		case REQ_WPAN_SET_HWADDR:
			break;
		case REQ_WPAN_SET_TXPOWER:
			break;
		case REQ_WPAN_SET_LBT:
			break;
		case REQ_WPAN_SET_CCA_MODE:
			break;
		case REQ_WPAN_SET_CCA_ED_LEVEL:
			break;
		case REQ_WPAN_SET_CSMA_PARAMS:
			break;
		case REQ_WPAN_SET_FRAME_RETRIES:
			break;
	}
	
	return ret;
}

bool prcm_setup_in(void)
{
	bool ret = false;
	
	switch(udd_g_ctrlreq.req.wValue) {
		case REQ_WPAN_GET_ED:
			udd_g_ctrlreq.payload = &wdev->last_ed;
			udd_g_ctrlreq.payload_size = sizeof(wdev->last_ed);
			ret = true;
			break;
		case REQ_WPAN_GET_FEATURES:
			udd_g_ctrlreq.payload = (uint8_t *)wdev;
			udd_g_ctrlreq.payload_size = sizeof(wpan_dev_cfg);
			ret = true;
			break;
		case REQ_WPAN_GET_CHANNEL_LIST:
			udd_g_ctrlreq.payload = (uint8_t *)&channel_page_support;
			udd_g_ctrlreq.payload_size = sizeof(channel_page_support);
			ret = true;
			break;
	}
	
	return ret;
}
