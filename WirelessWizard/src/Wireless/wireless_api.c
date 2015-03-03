#include "wireless_api.h"
#include "wireless_config.h"

wpan_dev_cfg wpan_dev = {
	/* default basic IEEE 802.15.4 params */
	.ieee_addr = 0xCAFEBABEDEADBEAF,
	.short_addr = 0x0001,
	.pan_id = 0x1234,
	.pan_coordinator = false,
	.page = 17,
	.channel = 1,
	.tx_power = 10,
	.wpan_active = false,
	/* default FLAGS */
	.flags = IEEE802154_HW_OMIT_CKSUM | IEEE802154_HW_AACK | 
			IEEE802154_HW_TXPOWER | IEEE802154_HW_CSMA |
			IEEE802154_HW_LBT,
	/* default CCA params */
	.cca_ed_level = 0x00,
	.cca_mode = CCA_MODE_0,
	.max_frame_retries = 0,
	/* default CSMA params */
	.csma_mode = CSMA_UNSLOTTED,
	.csma_min_be = 0,
	.csma_max_be = 0,
	.csma_retries = 0,
};

wpan_dev_cfg *wdev = &wpan_dev;

uint32_t channel_page_support[WPAN_NUM_PAGES];

void wireless_init(void)
{
	irq_initialize_vectors();
	sysclk_init();
	board_init();
	sw_timer_init();
	
	channel_page_support[0] = 0x003FF;
	channel_page_support[2] = 0x003FF;
	channel_page_support[5] = 0x003FF;
#ifdef HIGH_DATA_RATE_SUPPORT
	channel_page_support[16] = 0x003FF;
	channel_page_support[17] = 0x003FF;
	channel_page_support[18] = 0x003FF;
	channel_page_support[19] = 0x003FF;
#endif // HIGH_DATA_RATE_SUPPORT

	/*Initialize the TAL Layer*/
	if(tal_init()!= MAC_SUCCESS) {
		app_alert();
	}
	cpu_irq_enable();
}

void init_data_reception()
{
	tal_pib_set(macIeeeAddress, (pib_value_t *)&wdev->ieee_addr);
	tal_pib_set(macShortAddress, (pib_value_t *)&wdev->short_addr);
	tal_pib_set(macPANId, (pib_value_t *)&wdev->pan_id);

	tal_pib_set(phyCurrentPage,(pib_value_t *)&wdev->page);
	tal_pib_set(phyCurrentChannel, (pib_value_t *)&wdev->channel);
	
	uint8_t temp_var = CONV_DBM_TO_phyTransmitPower(wdev->tx_power);
	tal_pib_set(phyTransmitPower, (pib_value_t *)&temp_var);

	#if (ANTENNA_DIVERSITY == 1)
	if(ENABLE_ANTENNA_DIVERSITY)
	{
		tal_ant_div_config(ANT_DIVERSITY_ENABLE,ANTENNA_DEFAULT);
	}
	else
	{
		tal_ant_div_config(ANT_DIVERSITY_DISABLE,ANT_SELECT);
	}
	#endif

    /*Enable Promiscuous Mode pib attribute to put the transceiver in RX_ON mode.*/
    #ifdef PROMISCUOUS_MODE
		bool mode = true;
		tal_rxaack_prom_mode_ctrl(true);
		tal_pib_set(macPromiscuousMode, (pib_value_t *)&mode);
	#endif
    
	/*RX_AACK_ON Mode is enabled if Promiscuous Mode is not used,else RX is switched on in RX_ON Mode*/
    tal_rx_enable(PHY_RX_ON); 
}



/**
 *
 * \brief This method  is called when an error is encountered during Stack Initialization
 * or to alert the user on any undesired activity
 *
 */
void app_alert(void)
{
	/* Indicate error - flash an LED */
	
	while(1)
	{
		//led_toggle();
		//delay_ms(100);
	}
	
}