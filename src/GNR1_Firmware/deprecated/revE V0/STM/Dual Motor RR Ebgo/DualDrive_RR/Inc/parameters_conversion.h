
/**
  ******************************************************************************
  * @file    parameters_conversion.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_H
#define __PARAMETERS_CONVERSION_H

#include "mc_math.h"
#include "parameters_conversion_g4xx.h"
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"

#define ADC_REFERENCE_VOLTAGE  3.30

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE 	(uint32_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE))

/* TF_REGULATION_RATE_SCALED is TF_REGULATION_RATE divided by PWM_FREQ_SCALING to allow more dynamic */
#define TF_REGULATION_RATE_SCALED (uint16_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING))

/* DPP_CONV_FACTOR is introduce to compute the right DPP with TF_REGULATOR_SCALED  */
#define DPP_CONV_FACTOR (65536/PWM_FREQ_SCALING)

#define REP_COUNTER 			(uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

#define SYS_TICK_FREQUENCY          2000
#define UI_TASK_FREQUENCY_HZ        10
#define SERIAL_COM_TIMEOUT_INVERSE  25
#define SERIAL_COM_ATR_TIME_MS 20

#define MEDIUM_FREQUENCY_TASK_RATE	(uint16_t)SPEED_LOOP_FREQUENCY_HZ

#define INRUSH_CURRLIMIT_DELAY_COUNTS  (uint16_t)(INRUSH_CURRLIMIT_DELAY_MS * \
                                  ((uint16_t)SPEED_LOOP_FREQUENCY_HZ)/1000u -1u)

#define MF_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u

#define UI_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/UI_TASK_FREQUENCY_HZ)-1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY/SERIAL_COM_TIMEOUT_INVERSE)-1u
#define SERIALCOM_ATR_TIME_TICKS (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

#define TF_REGULATION_RATE2 	(uint32_t) ((uint32_t)(PWM_FREQUENCY2)/REGULATION_EXECUTION_RATE2)

/* TF_REGULATION_RATE_SCALED2 is TF_REGULATION_RATE2 divided by PWM_FREQ_SCALING2 to allow more dynamic */
#define TF_REGULATION_RATE_SCALED2 (uint16_t) ((uint32_t)(PWM_FREQUENCY2)/(REGULATION_EXECUTION_RATE2*PWM_FREQ_SCALING2))

/* DPP_CONV_FACTOR2 is introduce to compute the right DPP with TF_REGULATOR_SCALED2  */
#define DPP_CONV_FACTOR2 (65536/PWM_FREQ_SCALING2)

#define REP_COUNTER2 			(uint16_t) ((REGULATION_EXECUTION_RATE2 *2u)-1u)

#define MEDIUM_FREQUENCY_TASK_RATE2	(uint16_t)SPEED_LOOP_FREQUENCY_HZ2

#define INRUSH_CURRLIMIT_DELAY_COUNTS2  (uint16_t)(INRUSH_CURRLIMIT_DELAY_MS2 * \
                                  ((uint16_t)SPEED_LOOP_FREQUENCY_HZ2)/1000u -1u)

#define MF_TASK_OCCURENCE_TICKS2  (SYS_TICK_FREQUENCY/MEDIUM_FREQUENCY_TASK_RATE2)-1u

#define MAX_APPLICATION_SPEED_UNIT ((MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM)
#define MIN_APPLICATION_SPEED_UNIT ((MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM)

#define MAX_APPLICATION_SPEED_UNIT2 ((MAX_APPLICATION_SPEED_RPM2*SPEED_UNIT)/_RPM)
#define MIN_APPLICATION_SPEED_UNIT2 ((MIN_APPLICATION_SPEED_RPM2*SPEED_UNIT)/_RPM)

/**************************   VOLTAGE CONVERSIONS  Motor 1 *************************/
#define OVERVOLTAGE_THRESHOLD_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define UNDERVOLTAGE_THRESHOLD_d  (uint16_t)((UD_VOLTAGE_THRESHOLD_V*65535)/\
                                  ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           VBUS_PARTITIONING_FACTOR)))
#define INT_SUPPLY_VOLTAGE          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define DELTA_TEMP_THRESHOLD        (OV_TEMPERATURE_THRESHOLD_C- T0_C)
#define DELTA_V_THRESHOLD           (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d  ((V0_V + DELTA_V_THRESHOLD)*INT_SUPPLY_VOLTAGE)

#define DELTA_TEMP_HYSTERESIS        (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS           (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d  (DELTA_V_HYSTERESIS*INT_SUPPLY_VOLTAGE)
/**************************   VOLTAGE CONVERSIONS  Motor 2 *************************/
#define OVERVOLTAGE_THRESHOLD_d2   (uint16_t)(OV_VOLTAGE_THRESHOLD_V2*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR2))
#define UNDERVOLTAGE_THRESHOLD_d2  (uint16_t)((UD_VOLTAGE_THRESHOLD_V2*65535)/\
                                  ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           VBUS_PARTITIONING_FACTOR2)))
#define INT_SUPPLY_VOLTAGE2          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define DELTA_TEMP_THRESHOLD2        (OV_TEMPERATURE_THRESHOLD_C2- T0_C2)
#define DELTA_V_THRESHOLD2           (dV_dT2 * DELTA_TEMP_THRESHOLD2)
#define OV_TEMPERATURE_THRESHOLD_d2  ((V0_V2 + DELTA_V_THRESHOLD2)*INT_SUPPLY_VOLTAGE2)

#define DELTA_TEMP_HYSTERESIS2        (OV_TEMPERATURE_HYSTERESIS_C2)
#define DELTA_V_HYSTERESIS2           (dV_dT2 * DELTA_TEMP_HYSTERESIS2)
#define OV_TEMPERATURE_HYSTERESIS_d2  (DELTA_V_HYSTERESIS2*INT_SUPPLY_VOLTAGE2)

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)((ADV_TIM_CLK_MHz*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY)))&0xFFFE)

#define PWM_PERIOD_CYCLES2 (uint16_t)((ADV_TIM_CLK_MHz2*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY2)))&0xFFFE)

#define DEADTIME_NS  SW_DEADTIME_NS
#define DEADTIME_NS2  SW_DEADTIME_NS2

#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif
#define DEAD_TIME_ADV_TIM_CLK_MHz2 (ADV_TIM_CLK_MHz2 * TIM_CLOCK_DIVIDER2)
#define DEAD_TIME_COUNTS2_1  (DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/1000uL)

#if (DEAD_TIME_COUNTS2_1 <= 255)
#define DEAD_TIME_COUNTS2 (uint16_t) DEAD_TIME_COUNTS2_1
#elif (DEAD_TIME_COUNTS2_1 <= 508)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS2_1 <= 1008)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS2_1 <= 2015)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS2 510
#endif

#define DTCOMPCNT (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define TON_NS  500
#define TOFF_NS 500
#define TON  (uint16_t)((TON_NS * ADV_TIM_CLK_MHz)  / 2000)
#define TOFF (uint16_t)((TOFF_NS * ADV_TIM_CLK_MHz) / 2000)
#define DTCOMPCNT2 (uint16_t)((DEADTIME_NS2 * ADV_TIM_CLK_MHz2) / 2000)
#define TON_NS2  500
#define TOFF_NS2 500
#define TON2  (uint16_t)((TON_NS2 * ADV_TIM_CLK_MHz2)  / 2000)
#define TOFF2 (uint16_t)((TOFF_NS2 * ADV_TIM_CLK_MHz2) / 2000)
/**********************/
/* MOTOR 1 ADC Timing */
/**********************/
#define SAMPLING_TIME ((ADC_SAMPLING_CYCLES * ADV_TIM_CLK_MHz) / ADC_CLK_MHz) /* In ADV_TIMER CLK cycles*/
#define HTMIN 1 /* Required for main.c compilation only, CCR4 is overwritten at runtime */
#define TW_BEFORE ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES) * ADV_TIM_CLK_MHz) / ADC_CLK_MHz  + 1u)
#define TW_BEFORE_R3_1 ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES*2 + ADC_SAR_CYCLES) * ADV_TIM_CLK_MHz) / ADC_CLK_MHz  + 1u)
#define TW_AFTER ((uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*ADV_TIM_CLK_MHz)/1000ul))
#define MAX_TWAIT ((uint16_t)((TW_AFTER - SAMPLING_TIME)/2))

/**********************/
/* MOTOR 2 ADC Timing */
/**********************/
#define SAMPLING_TIME2 ((ADC_SAMPLING_CYCLES2 * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2) /* In ADV_TIMER2 CLK cycles*/
#define HTMIN2 0 /* Required for main.c compilation only, CCR4 is overwritten at runtime */
#define TW_BEFORE2 ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES2) * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2  + 1u)
#define TW_AFTER2 ((uint16_t)(((DEADTIME_NS2+MAX_TNTR_NS2)*ADV_TIM_CLK_MHz2)/1000ul))
#define MAX_TWAIT2 ((uint16_t)((TW_AFTER2 - SAMPLING_TIME2)/2))

/* USER CODE BEGIN temperature */

#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M1_TEMP_SW_FILTER_BW_FACTOR      250u

/* USER CODE END temperature */

/* Flux Weakening - Feed forward */
#define M1_VQD_SW_FILTER_BW_FACTOR       128u
#define M1_VQD_SW_FILTER_BW_FACTOR_LOG LOG2(M1_VQD_SW_FILTER_BW_FACTOR)

/* USER CODE BEGIN temperature M2*/

#define M2_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M2_TEMP_SW_FILTER_BW_FACTOR     250u

/* USER CODE END temperature M2*/
/* Flux Weakening - Feed forward Motor 2*/
#define M2_VQD_SW_FILTER_BW_FACTOR      128u
#define M2_VQD_SW_FILTER_BW_FACTOR_LOG LOG2(M2_VQD_SW_FILTER_BW_FACTOR)

#define PQD_CONVERSION_FACTOR (int32_t)(( 1000 * 3 * ADC_REFERENCE_VOLTAGE ) /\
             ( 1.732 * AMPLIFICATION_GAIN ))
#define PQD_CONVERSION_FACTOR2 (int32_t)(( 1000 * 3 * ADC_REFERENCE_VOLTAGE ) /\
             ( 1.732 * AMPLIFICATION_GAIN2 ))
#define PWM_FREQUENCY_CHECK_RATIO   (PWM_FREQUENCY*10000u/PWM_FREQUENCY2)

#define MAGN_FREQ_RATIO   (1*10000u)

#if (PWM_FREQUENCY_CHECK_RATIO != MAGN_FREQ_RATIO)
#error "The two motor PWM frequencies should be integer multiple"
#endif

#define USART_IRQHandler USART1_IRQHandler

/****** Prepares the UI configurations according the MCconfxx settings ********/
#define COM_ENABLE | OPT_COM

#define DAC_ENABLE | OPT_DAC
#define DAC_OP_ENABLE | UI_CFGOPT_DAC

/* Motor 1 settings */
#define FW_ENABLE | UI_CFGOPT_FW

#define DIFFTERM_ENABLE
/* Motor 2 settings */
#define FW_ENABLE2 | UI_CFGOPT_FW

#define DIFFTERM_ENABLE2

/* Sensors setting */

#define MAIN_SCFG UI_SCODE_HALL

#define AUX_SCFG 0x0

#define MAIN_SCFG2 UI_SCODE_HALL

#define AUX_SCFG2 0x0

#define PLLTUNING_ENABLE
#define PLLTUNING_ENABLE2

#define UI_CFGOPT_PFC_ENABLE

/*******************************************************************************
  * UI configurations settings. It can be manually overwritten if special
  * configuartion is required.
*******************************************************************************/

/* Specific options of UI */
#define UI_CONFIG_M1 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE \
  | (MAIN_SCFG << MAIN_SCFG_POS) | (AUX_SCFG << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PLLTUNING)

/* Specific options of UI, Motor 2 */
#define UI_CONFIG_M2 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE2 \
  | (MAIN_SCFG2 << MAIN_SCFG_POS) | (AUX_SCFG2 << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE2 )

#define DIN_ACTIVE_LOW Bit_RESET
#define DIN_ACTIVE_HIGH Bit_SET

#define DOUT_ACTIVE_HIGH   DOutputActiveHigh
#define DOUT_ACTIVE_LOW    DOutputActiveLow

/**********  AUXILIARY HALL TIMER MOTOR 1 *************/
#define M1_HALL_TIM_PERIOD 65535
#define M1_HALL_IC_FILTER  14
#define SPD_TIM_M1_IRQHandler TIM2_IRQHandler

/**********  AUXILIARY HALL TIMER MOTOR 2 *************/
#define M2_HALL_TIM_PERIOD 65535
#define M2_HALL_IC_FILTER  14
#define SPD_TIM_M2_IRQHandler TIM4_IRQHandler

/* MMI Table Motor 1 MAX_MODULATION_100_PER_CENT */
#define START_INDEX 63
#define MAX_MODULE 32767
#define MMITABLE  {\
32767,32390,32146,31907,31673,31444,31220,31001,30787,30577,30371,\
30169,29971,29777,29587,29400,29217,29037,28861,28687,28517,\
28350,28185,28024,27865,27709,27555,27404,27256,27110,26966,\
26824,26685,26548,26413,26280,26149,26019,25892,25767,25643,\
25521,25401,25283,25166,25051,24937,24825,24715,24606,24498,\
24392,24287,24183,24081,23980,23880,23782,23684,23588,23493,\
23400,23307,23215,23125\
}

/* MMI Table Motor 2 MAX_MODULATION_100_PER_CENT */
#define START_INDEX2     63
#define MAX_MODULE2      32767
#define MMITABLE2        {\
32767,32390,32146,31907,31673,31444,31220,31001,30787,30577,30371,\
30169,29971,29777,29587,29400,29217,29037,28861,28687,28517,\
28350,28185,28024,27865,27709,27555,27404,27256,27110,26966,\
26824,26685,26548,26413,26280,26149,26019,25892,25767,25643,\
25521,25401,25283,25166,25051,24937,24825,24715,24606,24498,\
24392,24287,24183,24081,23980,23880,23782,23684,23588,23493,\
23400,23307,23215,23125\
}

#define SAMPLING_CYCLE_CORRECTION 0.5 /* Add half cycle required by STM32G474QETx ADC */
#define LL_ADC_SAMPLINGTIME_1CYCLES_5 LL_ADC_SAMPLINGTIME_1CYCLE_5
#define LL_ADC_SAMPLING_CYCLE(CYCLE) LL_ADC_SAMPLINGTIME_ ## CYCLE ## CYCLES_5

#endif /*__PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
