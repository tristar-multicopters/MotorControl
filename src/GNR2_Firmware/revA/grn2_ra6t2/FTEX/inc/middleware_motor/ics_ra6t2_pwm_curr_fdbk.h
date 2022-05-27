/**
  ******************************************************************************
  * @file    ics_ra6t2_pwm_curr_fdbk.c
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement three phase PWM generation and current measurement
  *          functionality with 2 isolated sensors on A and B phase. It is specifically designed for RA6T2
  *          microcontrollers.
  *           + MCU peripheral and handle initialization function
  *           + two isolated current sensor
  *           + space vector modulation function
  *           + ADC sampling function
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef ICS_RA6T2_PWMNCURRFDBK_H
#define ICS_RA6T2_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"
#include "r_gpt_three_phase.h"
#include "r_gpt.h"
#include "r_poeg.h"
#include "r_adc_b.h"

/* Exported types ------------------------------------------------------- */

typedef const struct
{
  /* HW parameters -----------------------------*/
	const adc_instance_t * pADCHandle;										/*!< Pointer to ADC instance */
	const adc_channel_t ADCChannelIa;											/*!< ADC channel for Ia */
	const adc_channel_t ADCChannelIb;											/*!< ADC channel for Ib */
	const adc_group_mask_t ADCGroupMask; 									/*!< ADC group mask used for triggered conversion.
																														Need to use a group with 2 ADC channels for triggered
																														ADC conversion. .*/
  const three_phase_instance_t * pThreePhaseHandle;     /*!< Pointer to three phase instance used for PWM generation.*/
	const poeg_instance_t * pPOEGHandle;
  
  /* PWM Driving signals initialization ----------------------------------------*/
  uint8_t  RepetitionCounter;         /*!< It expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/
																	 																	  																					                                     
} ICS_Params_t, *pICS_Params_t;

/**
  * @brief  This structure is used to handle an instance of the
  *         PWMC_ICS_Handle_t component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /*!< Base handle  */
	
	uint16_t hIaRaw;					/*!< Latest conversion value for Ia  */
	uint16_t hIbRaw;					/*!< Latest conversion value for Ib  */
	
	volatile bool bOverrunFlag;
	
  uint32_t PhaseAOffset;   	/*!< Offset of Phase A current sensing network  */
  uint32_t PhaseBOffset;   	/*!< Offset of Phase B current sensing network  */
  uint16_t Half_PWMPeriod;  /*!< Half PWM Period in timer clock counts */
  volatile uint8_t PolarizationCounter;

  bool bOverCurrentFlag;     /*!< This flag is used to check if overcurrent occured */
	
  pICS_Params_t pParams_str; /*!< PWM component parameters*/
	
} PWMC_ICS_Handle_t;


/* Exported functions ------------------------------------------------------- */

/**
  * @brief  It initializes the module and its hardware components.
  * @param  pHandle: handle of the current instance of the PWMC_ICS_Handle_t component
  * @retval none
  */
bool ICS_Init( PWMC_ICS_Handle_t * pHandle );

/**
  * @brief  This function starts the current sensor polarization routine. It stores into the provided handle 
	*					the voltage present on Ia and Ib current feedback analog channels when no current is flowing into the
  *         motor.
  * @param  pHdl: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval none
  */
void ICS_CurrentReadingPolarization( PWMC_Handle_t * pHdl );

/**
  * @brief  This function computes and return latest converted motor phase currents motor
  * @param  pHdl: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval Ia and Ib current in ab_t format
  */ 
void ICS_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * Iab );

/**
  * @brief  Function to turn on low sides switches. This function is intended to be
  *         used for charging boot capacitors. It has to be
  *         called at each motor start-up.
  * @param  pHdl: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval none
  */
void ICS_TurnOnLowSides( PWMC_Handle_t * pHdl );

/**
  * @brief  It enables PWM generation
  * @param  pHdl: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval none
  */
void ICS_SwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * @brief  It stops PWM generation
  * @param  pHdl: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval none
  */
void ICS_SwitchOffPWM( PWMC_Handle_t * pHdl );

/**
  * @brief  Function to update duty cycle registers.
	* @param  pHdl: handle of the current instance of the PWMC_ICS_Handle_t component.
* @retval Motor control error code: MC_FOC_DURATION if overrun occured, MC_NO_FAULTS otherwise.
  */
uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHdl );


/**
  * @brief  It is the routine to run when PWM timer update event happens
  * @param  pHandle: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval Motor instance number
  */
void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHdl );

/**
  * @brief  It is the routine to run when overcurrent trigger interrupt occured
  * @param  pHandle: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval Motor instance number
  */
void * ICS_BRK2_IRQHandler( PWMC_ICS_Handle_t * pHdl );

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHdl: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t ICS_IsOverCurrentOccurred( PWMC_Handle_t * pHdl );


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*ICS_RA6T2_PWMNCURRFDBK_H*/


