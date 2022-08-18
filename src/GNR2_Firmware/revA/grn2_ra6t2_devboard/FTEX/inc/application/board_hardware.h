/**
  ******************************************************************************
  * @file    board_hardware.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module defines the hardware (pins, peripherals, ...) used by the board. 
  *
	******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_HARDWARE_H
#define __BOARD_HARDWARE_H

#include "stdint.h"

/* Board specific functions ---------------------------------*/

uint32_t GetChipID(uint8_t ID_Half);



/* Brake input ----------------------------------------------*/

#define BRAKE_GPIO_PIN													0xc9


/* Reverse input --------------------------------------------*/

#define REVERSE_GPIO_PIN												0


/* Motor selection input ------------------------------------*/

#define M1SELECT_GPIO_PIN												0
#define M2SELECT_GPIO_PIN												0


/* Power enable ---------------------------------------------*/

#define PWREN_GPIO_PIN													0


/* Throttle ---------------------------------------------*/

#define THROTTLE_ANALOG_CHANNEL									ADC_CHANNEL_18


/* Pedal torque sensor ---------------------------------------------*/

#define PEDAL_TORQUE_SENSOR_ANALOG_CHANNEL			ADC_CHANNEL_20


/* Bus voltage sensor ---------------------------------------------*/

#define BUS_VOLTAGE_ANALOG_CHANNEL							ADC_CHANNEL_1


/* Heatsink temperature sensor ---------------------------------------------*/

#define HEATSINK_TEMP_ANALOG_CHANNEL						ADC_CHANNEL_0



#endif /*__BOARD_HARDWARE_H*/
