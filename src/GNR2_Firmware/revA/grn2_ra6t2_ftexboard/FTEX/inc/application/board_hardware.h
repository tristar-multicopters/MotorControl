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



/* GPIOs ----------------------------------------------*/

#define CAN_STANDBY_N_GPIO_PIN								0xb4
#define CAN_ENABLE_N_GPIO_PIN								0xd7
#define BRAKE_GPIO_PIN										0xc0
#define REVERSE_GPIO_PIN									0x01
#define M2SELECT_GPIO_PIN									0xa9
#define M1SELECT_GPIO_PIN									0xdf
#define PWREN_GPIO_PIN									    0
#define PWRSTG_ENABLE_GPIO_PIN							    0xe9

#define FRONT_LIGHT_PIN                                     0xd4
#define BACK_LIGHT_PIN                                      0xda


/* Hall position sensor ---------------------------------------------*/

#define HALL_POSITION_TIMER_HANDLE_ADDRESS                  &g_timer0
#define HALL_POSITION_HU_GPIO_PIN                           M1_ENC_U
#define HALL_POSITION_HV_GPIO_PIN                           M1_ENC_V
#define HALL_POSITION_HW_GPIO_PIN                           M1_ENC_W


/* Current sensing & PWM ---------------------------------------------*/

#define CURRENT_SENSOR_ADC_HANDLE_ADDRESS                   &g_adc0
#define CURRENT_SENSOR_IA_ANALOG_CHANNEL                    ADC_CHANNEL_0
#define CURRENT_SENSOR_IB_ANALOG_CHANNEL                    ADC_CHANNEL_2
#define CURRENT_SENSOR_ADC_GROUP_MASK                       ADC_GROUP_MASK_0 // Current sensing pins must have their own ADC group, separated from other analog pins

#define PWM_THREE_PHASE_HANDLE_ADDRESS                      &g_three_phase0
#define PWM_POEG0_HANDLE_ADDRESS                            &g_poeg0

#define SOCP_IA_IIRFA_HANDLE_ADDRESS                        &g_iirfa0
#define SOCP_IB_IIRFA_HANDLE_ADDRESS                        &g_iirfa1


/* DAC for debugging ---------------------------------------------*/

#define DEBUG1_DAC_HANDLE_ADDRESS                           &g_dac2
#define DEBUG2_DAC_HANDLE_ADDRESS                           &g_dac3


/* ADC Regular conversion manager ---------------------------------------------*/

#define BUS_VOLTAGE_ANALOG_CHANNEL											ADC_CHANNEL_6
#define HEATSINK_TEMP_ANALOG_CHANNEL										ADC_CHANNEL_8
#define THROTTLE_ANALOG_CHANNEL							    				ADC_CHANNEL_9
#define PEDAL_TORQUE_SENSOR_ANALOG_CHANNEL			        ADC_CHANNEL_20

#define FIRST_REG_CONV_ADC_GROUP_MASK                   ADC_GROUP_MASK_1 // Regular ADC conversion manager will work with ADC pins contained in two designated groups
#define SECOND_REG_CONV_ADC_GROUP_MASK                  ADC_GROUP_MASK_2 // Regular ADC conversion manager will work with ADC pins contained in two designated groups

/* Pedal speed sensor ---------------------------------------------*/

#define PEDAL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS         &g_timer_a0 

/* Wheel speed sensor ---------------------------------------------*/

#define WHEEL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS         &g_timer9

#endif /*__BOARD_HARDWARE_H*/
