/**
  ******************************************************************************
  * @file    mcp_can.c
  * @author  Avadh Patel, Sami Bouzid, FTEX
  * @brief   This module handles sending and receiving CAN frames using MCP25625
  *
	******************************************************************************
*/

#include "mcp_can.h"


#define MCP_QUEUE_LENGTH     15
#define MCP_SPI_INSTANCE_ID  0

NRF_SPI_MNGR_DEF(m_nrf_spi_mngr, MCP_QUEUE_LENGTH, MCP_SPI_INSTANCE_ID);

static uint8_t m_tx_buf[18];    /**< TX buffer. */
static uint8_t m_rx_buf[18];    /**< RX buffer. */

static mcp_can_t m_mcp_can;

// Function called before SPI command is sent to mcp
static void mcp_cb_before(void * p_user_data)
{
    nrf_gpio_pin_clear(m_mcp_can.spi_cs);
}

// Function called after SPI command is sent to mcp
static void mcp_cb_after(void * p_user_data)
{
    nrf_gpio_pin_clear(m_mcp_can.spi_cs);
}

static ret_code_t init_mcp_can(void)
{
//    // SPI0 (with transaction manager) initialization.
//    nrf_drv_spi_config_t const m_master0_config =
//    {
//        .sck_pin        = ARDUINO_13_PIN,
//        .mosi_pin       = ARDUINO_11_PIN,
//        .miso_pin       = ARDUINO_12_PIN,
//        .ss_pin         = ARDUINO_10_PIN,
//        .irq_priority   = APP_IRQ_PRIORITY_LOWEST,
//        .orc            = 0xFF,
//        .frequency      = NRF_DRV_SPI_FREQ_8M,
//        .mode           = NRF_DRV_SPI_MODE_0,
//        .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
//    };
//    return nrf_spi_mngr_init(&m_nrf_spi_mngr, &m_master0_config);
}


/**@brief Task function to manage Can bus
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
void TSK_CANBUS (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	init_mcp_can();
	
	while (true)
	{					
		
	}
}