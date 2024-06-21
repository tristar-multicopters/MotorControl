/**
  * @file    parameters_conversion.h
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
*/

/*
 __          __     _____  _   _ _____ _   _  _____ 
 \ \        / /\   |  __ \| \ | |_   _| \ | |/ ____|
  \ \  /\  / /  \  | |__) |  \| | | | |  \| | |  __ 
   \ \/  \/ / /\ \ |  _  /| . ` | | | | . ` | | |_ |
    \  /\  / ____ \| | \ \| |\  |_| |_| |\  | |__| |
     \/  \/_/    \_\_|  \_\_| \_|_____|_| \_|\_____|
                                                    
 Be VERY careful where you include this .h ESPECIALLY if this is done 
 outside of motor control. There has been an instance when going full 
 throttle and spamming the brake handle causes the motor control loop 
 to be seemingly lost, weird noises can be heard from the motor.
 
 If you need values from this file elsewhere and want to avoid including
 this file please use a similar solution that was put in place with 
 
 GAIN_TORQUE_IQREF
 
 */                                               


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_H
#define __PARAMETERS_CONVERSION_H

#include "mc_math.h"
#include "parameters_conversion_ra6t2.h"
#include "drive_parameters.h"
#include "foldback.h"
#include "pid_regulator.h"
#include "ntc_temperature_sensor.h"


/****** ADC Parameters ******/
#define ADC_REFERENCE_VOLTAGE           3.30                       // Reference voltage for the ADC (Analog-to-Digital Converter) in volts.
#define ADC_BYTE_TO_TICS                16                         // Number of tics in the ADC resolution.
#define ADC_MAXIMUM_VALUE               4096                       // Maximum digital value output by the ADC (for a 12-bit ADC, 2^12 = 4096).

/****** Temperature Parameters ******/
#define CELSIUS_TO_KELVIN               273.15                     // Conversion constant for converting Celsius to Kelvin.
#define TEMP_25_CELSIUS_IN_KELVIN       25 + CELSIUS_TO_KELVIN     // Conversion constant for 25 degrees Celsius to Kelvin.
#define MOTOR_NTC_RESISTANCE_COEF_X_100 (uint16_t)(100*MOTOR_NTC_RESISTANCE_COEF) // Since we cannot add a float parameter to the object dictionary, we multiply it by 100.
                                                                                  // In calculations, when we want to use it, we first divide it by 100.

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE     (uint32_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE))

/* TF_REGULATION_RATE_SCALED is TF_REGULATION_RATE divided by PWM_FREQ_SCALING to allow more dynamic */
#define TF_REGULATION_RATE_SCALED (uint16_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING))

/* DPP_CONV_FACTOR is introduce to compute the right DPP with TF_REGULATOR_SCALED  */
#define DPP_CONV_FACTOR (65536/PWM_FREQ_SCALING)

#define REP_COUNTER             (uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

#define SYS_TICK_FREQUENCY          2000
#define UI_TASK_FREQUENCY_HZ        10
#define SERIAL_COM_TIMEOUT_INVERSE  25
#define SERIAL_COM_ATR_TIME_MS 20

#define MEDIUM_FREQUENCY_TASK_RATE    (uint16_t)SPEED_LOOP_FREQUENCY_HZ

#define INRUSH_CURRLIMIT_DELAY_COUNTS  (uint16_t)(INRUSH_CURRLIMIT_DELAY_MS * \
                                  ((uint16_t)SPEED_LOOP_FREQUENCY_HZ)/1000u -1u)

#define MF_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u

#define UI_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/UI_TASK_FREQUENCY_HZ)-1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY/SERIAL_COM_TIMEOUT_INVERSE)-1u
#define SERIALCOM_ATR_TIME_TICKS (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

/************************* COMMON OBSERVER PARAMETERS **************************/
/*max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE (int16_t)((ADC_REFERENCE_VOLTAGE/SQRT_3)/VBUS_PARTITIONING_FACTOR)

/* The maximum current from Current Sensor with AMPLIFICATION_GAIN that we can measure using ADC with ADC_REFERENCE_VOLTAGE */
#define current_correction_factor   0.85  /* correction factor for the current sensor range */
#define MAX_MEASURABLE_CURRENT      (ADC_REFERENCE_VOLTAGE/(2*AMPLIFICATION_GAIN)* current_correction_factor)
#define ID_DEMAG                    (int16_t)(ID_DEMAG_amps  * 65535 / (2 * MAX_MEASURABLE_CURRENT))
#define IQ_REGEN                    (int16_t)(IQ_REGEN_AMPS  * 65535 / (2 * MAX_MEASURABLE_CURRENT))

#define OCSP_SAFETY_MARGIN                 (uint16_t)(OCSP_SAFETY_MARGIN_amps  * 65535 / (2 * MAX_MEASURABLE_CURRENT))    /* Measured current amplitude can be until OCSP_SAFETY_MARGIN higher
                                                than reference current before overcurrent software protection triggers */
#define OCSP_MAX_CURRENT                (uint16_t)(OCSP_MAX_CURRENT_amps  * 65535 / (2 * MAX_MEASURABLE_CURRENT))   /* Max current that can be reached before triggering software overcurrent */

#define OBS_MINIMUM_SPEED_UNIT    (uint16_t) ((OBS_MINIMUM_SPEED_RPM*SPEED_UNIT)/_RPM)

#define MIN_APPLICATION_SPEED_UNIT ((MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM)

/************************* PLL PARAMETERS **************************/
#define C2 (int32_t) GAIN1
#define C4 (int32_t) GAIN2

#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u)
#define HFI_MINIMUM_SPEED    (uint16_t) (HFI_MINIMUM_SPEED_RPM/6u)

/**************************   VOLTAGE CONVERSIONS  Motor 1 *************************/
#define OVERVOLTAGE_THRESHOLD_d            (uint16_t)(OV_VOLTAGE_THRESHOLD_V*65535/\
                                           (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define DEFAULT_UNDERVOLTAGE_THRESHOLD_d   (uint16_t)((UD_VOLTAGE_THRESHOLD_CONT_V*65535)/\
                                           ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           VBUS_PARTITIONING_FACTOR)))
#define INT_SUPPLY_VOLTAGE          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define M1_VBUS_SW_FILTER_BW_FACTOR     6

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)((PWM_TIM_CLK_MHz*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY)))&0xFFFE)

#define DEAD_TIME_ADV_TIM_CLK_MHz (PWM_TIM_CLK_MHz)
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

#define DTCOMPCNT (uint16_t)((DEADTIME_NS * PWM_TIM_CLK_MHz) / 2000)
#define TON_NS  500
#define TOFF_NS 500
#define TON  (uint16_t)((TON_NS * PWM_TIM_CLK_MHz)  / 2000)
#define TOFF (uint16_t)((TOFF_NS * PWM_TIM_CLK_MHz) / 2000)

/*************** Current vs torque ratio ******/

#define GAIN_TORQUE_IDREF              0


/* USER CODE BEGIN temperature */

#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M1_TEMP_SW_FILTER_BW_FACTOR      250u

/* USER CODE END temperature */

/* Flux Weakening - Feed forward */
#define M1_VQD_SW_FILTER_BW_FACTOR       128u
#define M1_VQD_SW_FILTER_BW_FACTOR_LOG LOG2(M1_VQD_SW_FILTER_BW_FACTOR)

#define PQD_CONVERSION_FACTOR (int32_t)((1000 * 3 * ADC_REFERENCE_VOLTAGE) /\
             (1.732 * AMPLIFICATION_GAIN))

#define USART_IRQHandler USART1_IRQHandler

/****** Prepares the UI configurations according the MCconfxx settings ********/
#define COM_ENABLE | OPT_COM

#define DAC_ENABLE | OPT_DAC
#define DAC_OP_ENABLE | UI_CFGOPT_DAC

/* Motor 1 settings */
#define FW_ENABLE | UI_CFGOPT_FW

#define DIFFTERM_ENABLE

/* Sensors setting */

#define AUX_SCFG UI_SCODE_STO_PLL

#define MAIN_SCFG UI_SCODE_HALL

#define PLLTUNING_ENABLE

#define UI_CFGOPT_PFC_ENABLE

/*******************************************************************************
  * UI configurations settings. It can be manually overwritten if special
  * configuartion is required.
*******************************************************************************/

/* Specific options of UI */
#define UI_CONFIG_M1 (UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE \
  | (MAIN_SCFG << MAIN_SCFG_POS) | (AUX_SCFG << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PLLTUNING)

#define UI_CONFIG_M2

#define DIN_ACTIVE_LOW Bit_RESET
#define DIN_ACTIVE_HIGH Bit_SET

#define DOUT_ACTIVE_HIGH   DOutputActiveHigh
#define DOUT_ACTIVE_LOW    DOutputActiveLow

/**********  AUXILIARY HALL TIMER MOTOR 1 *************/
#define M1_HALL_TIM_PERIOD 65535
#define M1_HALL_IC_FILTER  14
#define SPD_TIM_M1_IRQHandler TIM2_IRQHandler

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

#define SAMPLING_CYCLE_CORRECTION 0.5 /* Add half cycle required by STM32G431CBUx ADC */
#define LL_ADC_SAMPLINGTIME_1CYCLES_5 LL_ADC_SAMPLINGTIME_1CYCLE_5
#define LL_ADC_SAMPLING_CYCLE(CYCLE) LL_ADC_SAMPLINGTIME_ ## CYCLE ## CYCLES_5

//Parameters to convert that are based on motor-specific parameters
typedef struct 
{
    uint16_t hPeakCurrentAmps;
    int16_t hNominalPeakCurrent;
    
    int16_t hMaxBEMFVoltage;
    
    uint16_t hMaxApplicationSpeedUnit;
    
    uint16_t hNominalTorque;
    uint16_t hStartingTorque;
    float fGainTorqueIqRef;
    
    int16_t hC1;
    int16_t hC3;
    int16_t hC5;
    
    Foldback_Handle_t FoldbackInitSpeed;
    Foldback_Handle_t FoldbackInitTorque;
    Foldback_Handle_t FoldbackInitHeatsinkTemp;
    Foldback_Handle_t FoldbackInitMotorTemp;
    
    PIDHandle_t PIDInitSpeedLimit;
    PIDHandle_t PIDInitSpeed;
    PIDHandle_t PIDInitIq;
    PIDHandle_t PIDInitId;
    PIDHandle_t PIDInitMotorControl;
    PIDHandle_t PIDInitBemfObserverPl;
    
    NTCTempSensorHandle_t HeatsinkNTCInit;
    NTCTempSensorHandle_t MotorNTCInit;
    
} ParametersConversion_t;


#endif /*__PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
