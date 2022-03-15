/**
  ******************************************************************************
  * @file    board_hardware.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module defines the hardware (pins, peripherals, ...) used by the board. 
	* 				 It creates as global variables the nrf_drv peripheral instances.
  *
	******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_HARDWARE_H
#define __BOARD_HARDWARE_H

#include "stdint.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_spi.h"


extern nrf_drv_spi_t spi0_inst;
extern nrf_drv_uart_t uart1_inst;
extern nrf_drv_timer_t timer1_inst;


/* Communication with motor drive (STM32) -------------------------------------*/

#define MD_UART_INSTANCE_ADDR			&uart1_inst
#define MD_UART_TX_PIN						NRF_GPIO_PIN_MAP(0,27)
#define MD_UART_RX_PIN						NRF_GPIO_PIN_MAP(0,6)


/* Communication with MCP2515 -------------------------------------*/

#define MCP_SS_PIN								NRF_GPIO_PIN_MAP(0,29)
#define MCP_INT_PIN								NRF_GPIO_PIN_MAP(0,2)
#define MCP_RST_PIN								NRF_GPIO_PIN_MAP(1,13)
#define MCP_SLCT_120R							NRF_GPIO_PIN_MAP(1,14)
// Test Purpose only				
#define	MCP_LOCK_PIN							NRF_GPIO_PIN_MAP(1,01)

/* SPI1 Bus -------------------------------------*/

#define SPI0_INSTANCE_ADDR				&spi0_inst
#define SPI0_MISO_PIN							NRF_GPIO_PIN_MAP(1,11)
#define SPI0_MOSI_PIN							NRF_GPIO_PIN_MAP(0,5)
#define SPI0_SCK_PIN							NRF_GPIO_PIN_MAP(1,10)


/* Throttle analog input -------------------------------------*/

#define RCM_TIMER_INSTANCE_ADDR		&timer1_inst
#define THROTTLE_ANALOG_PIN 			NRF_SAADC_INPUT_AIN2


/* Brake input -------------------------------------*/

#define BRAKE_GPIO_PIN						NRF_GPIO_PIN_MAP(1,4)

/* Motor selection input -------------------------------------*/

#define M1SELECT_GPIO_PIN						NRF_GPIO_PIN_MAP(0,25)
#define M2SELECT_GPIO_PIN						NRF_GPIO_PIN_MAP(0,19)



#endif /*__BOARD_HARDWARE_H*/
