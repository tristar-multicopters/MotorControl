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
extern nrf_drv_uart_t uart0_inst;
extern nrf_drv_timer_t timer1_inst;
extern nrf_drv_timer_t timer2_inst;
extern nrf_drv_timer_t timer4_inst;

/* Board specific functions ---------------------------------*/

uint32_t GetChipID(uint8_t ID_Half);

/* Communication with motor drive (STM32) -------------------*/

#define MD_UART_INSTANCE_ADDR				&uart1_inst
#define MD_UART_TX_PIN							NRF_GPIO_PIN_MAP(0,12)
#define MD_UART_RX_PIN							NRF_GPIO_PIN_MAP(0,8)


/* CANbus communication -------------------------------------*/

#define MCP_SS_PIN									NRF_GPIO_PIN_MAP(0,22)
#define MCP_INT_PIN									NRF_GPIO_PIN_MAP(0,15)
#define MCP_RST_PIN									NRF_GPIO_PIN_MAP(1,13)
#define CAN_STBY_PIN								NRF_GPIO_PIN_MAP(0,1)
	
/* SPI1 Bus -------------------------------------------------*/

#define SPI0_INSTANCE_ADDR					&spi0_inst
#define SPI0_MISO_PIN								NRF_GPIO_PIN_MAP(0,6)
#define SPI0_MOSI_PIN								NRF_GPIO_PIN_MAP(0,4)
#define SPI0_SCK_PIN								NRF_GPIO_PIN_MAP(0,20)


/* Regular conversion manager (analog input scheduler)-------*/

#define RCM_TIMER_INSTANCE_ADDR			&timer1_inst


/* Throttle input -------------------------------------------*/

#define THROTTLE_ANALOG_PIN 				NRF_SAADC_INPUT_AIN7


/* Brake input ----------------------------------------------*/

#define BRAKE_GPIO_PIN							NRF_GPIO_PIN_MAP(1,11)


/* Reverse input --------------------------------------------*/

#define REVERSE_GPIO_PIN						NRF_GPIO_PIN_MAP(1,15)


/* Motor selection input ------------------------------------*/

#define M1SELECT_GPIO_PIN						NRF_GPIO_PIN_MAP(0,25)
#define M2SELECT_GPIO_PIN						NRF_GPIO_PIN_MAP(0,21)


/* Communication with external device using UART ------------*/

#define UART0_INSTANCE_ADDR					&uart0_inst
#define UART0_TX_PIN								NRF_GPIO_PIN_MAP(0,3)
#define UART0_RX_PIN								NRF_GPIO_PIN_MAP(0,30)

/* Pedal assist ---------------------------------------------*/

#define PAS_TIMER_INSTANCE_ADDR			&timer2_inst
#define PAS_SIN_GPIO_PIN						NRF_GPIO_PIN_MAP(0,14)
#define PAS_COS_GPIO_PIN						NRF_GPIO_PIN_MAP(0,17)
#define PAS_TORQUE_PIN							NRF_SAADC_INPUT_AIN0

/* Power enable ---------------------------------------------*/

#define PWREN_GPIO_PIN							NRF_GPIO_PIN_MAP(0,13)

/* Wheel speed  ---------------------------------------------*/

#define WH_TIMER_INSTANCE_ADDR		&timer4_inst
#define WH_PUL_GPIO_PIN						NRF_GPIO_PIN_MAP(1,15)

#endif /*__BOARD_HARDWARE_H*/
