#include "wireless_api.h"
#include "wireless_config.h"

#include <tal_constants.h>

wpan_dev_cfg wpan_dev = {
	/* default basic IEEE 802.15.4 params */
	.ieee_addr = 0xCAFEBABEDEADBEAF,
	.short_addr = TAL_SHORT_ADDRESS_DEFAULT,
	.pan_id = TAL_PANID_BC_DEFAULT,
	.pan_coordinator = TAL_PAN_COORDINATOR_DEFAULT,
	.page = 17, /* Fixed value: see TODO list */
	.channel = 1, /* Fixed value: see TODO list */
	.tx_power = 10, /* Fixed value: see TODO list */
	.lbt_mode = true, /* Fixed value: see TODO list*/
	.wpan_active = false,
	/* default CCA params */
	.cca_ed_level = 0x00,
	.cca_mode = TAL_CCA_MODE_DEFAULT,
	.max_frame_retries = TAL_MAXFRAMERETRIES_DEFAULT,
	/* default CSMA params */
	.csma_mode = CSMA_UNSLOTTED,
	.csma_min_be = TAL_MINBE_DEFAULT,
	.csma_max_be = TAL_MAXBE_DEFAULT,
	.csma_retries = TAL_MAX_CSMA_BACKOFFS_DEFAULT,
	/* Promiscous mode */
#ifdef PROMISCUOUS_MODE
	.promisc_mode = false,
#endif
};

wpan_dev_cfg *wdev = &wpan_dev;

uint32_t channel_page_support[WPAN_NUM_PAGES];

void wireless_init(void)
{
	irq_initialize_vectors();
	sysclk_init();
	board_init();
	sw_timer_init();

	/*Initialize the TAL Layer*/
	if(tal_init()!= MAC_SUCCESS) {
		app_alert();
	}
	/* DIG3,4 - ext PA control */
	if(tal_ext_pa_ctrl(true)!= MAC_SUCCESS) {
		app_alert();
	}
	/*
	 * Lead time 8 uS, IO for DIG and CLKM - 8mA
	 */
	trx_bit_write(SR_PA_LT, 3);
	trx_bit_write(SR_PAD_IO, PAD_CLKM_8_MA);
	trx_bit_write(SR_PAD_IO_CLKM, PAD_CLKM_8_MA);
	cpu_irq_enable();
}

void init_default_pib(void)
{
	/* supported chanell/pages */
	channel_page_support[0] = 0x007FF;
	channel_page_support[2] = 0x007FF;
	channel_page_support[5] = 0x007FF;
#ifdef HIGH_DATA_RATE_SUPPORT
	channel_page_support[16] = 0x007FF;
	channel_page_support[17] = 0x007FF;
	channel_page_support[18] = 0x007FF;
	channel_page_support[19] = 0x007FF;
#endif // HIGH_DATA_RATE_SUPPORT

	tal_pib_set(macIeeeAddress, (pib_value_t *)&wdev->ieee_addr);
	tal_pib_set(macShortAddress, (pib_value_t *)&wdev->short_addr);
	tal_pib_set(macPANId, (pib_value_t *)&wdev->pan_id);

	tal_pib_set(phyCurrentPage,(pib_value_t *)&wdev->page);
	tal_pib_set(phyCurrentChannel, (pib_value_t *)&wdev->channel);
	
	tal_pib_set(phyCCAMode,(pib_value_t *)&wdev->cca_mode);
	tal_pib_set(macMaxFrameRetries, (pib_value_t *)&wdev->max_frame_retries);

	trx_bit_write(SR_CSMA_LBT_MODE, wdev->lbt_mode ? 1 : 0);

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
	tal_rxaack_prom_mode_ctrl(true);
	tal_pib_set(macPromiscuousMode, (pib_value_t *)&wdev->promisc_mode);
#endif
}

/**
 *
 * \brief This method  is called when an error is encountered during Stack Initialization
 * or to alert the user on any undesired activity
 *
 */
void app_alert(void)
{
	while(1) { /* freezee and wait for watchdog reset */
	}
	
}