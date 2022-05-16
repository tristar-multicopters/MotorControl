/**
  ******************************************************************************
  * @file    ics_ra6t2_pwm_curr_fdbk.c
  * @author  Sami Bouzid
  * @brief   This file provides firmware functions that implement current measurement
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
#include "r_adc_b.h"

/* Exported defines --------------------------------------------------------*/
#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/* Exported types ------------------------------------------------------- */

typedef const struct
{
  /* HW IP involved -----------------------------*/
	const adc_instance_t * pADCHandle;
	const adc_channel_t ADCChannelIa;
	const adc_channel_t ADCChannelIb;
  const three_phase_instance_t * pThreePhaseHandle;      /*!< three phase instance used for PWM generation.*/
  
  /* PWM Driving signals initialization ----------------------------------------*/
  uint8_t  RepetitionCounter;         /*!< It expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/
  /* Emergency input (BKIN2) signal initialization -----------------------------*/
  uint8_t BKIN2Mode;                 /*!< It defines the modality of emergency
                                           input 2. It must be any of the
                                           the following:
                                           NONE - feature disabled.
                                           INT_MODE - Internal comparator used
                                           as source of emergency event.
                                           EXT_MODE - External comparator used
                                           as source of emergency event.*/																			 																	  																					 
                                        

} ICS_Params_t, *pICS_Params_t;

/**
  * @brief  This structure is used to handle an instance of the
  *         r3_4_f30X_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /*!<   */
	
	uint16_t IaRaw;
	uint16_t IbRaw;
	
  uint32_t PhaseAOffset;   /*!< Offset of Phase A current sensing network  */
  uint32_t PhaseBOffset;   /*!< Offset of Phase B current sensing network  */
  uint16_t Half_PWMPeriod;  /*!< Half PWM Period in timer clock counts */
  volatile uint8_t PolarizationCounter;

  bool OverCurrentFlag;     /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;     /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;     /*!< This flag is set to avoid that brake action is
                                 interrupted.*/
  bool ADCRegularLocked; /* Cut 2.2 patch*/
  pICS_Params_t pParams_str;
} PWMC_ICS_Handle_t;


/* Exported functions ------------------------------------------------------- */

/**
  * It initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  * in three shunt topology using STM32F30X and shared ADC
  */
bool ICS_Init( PWMC_ICS_Handle_t * pHandle );

/**
  * It stores into the handle the voltage present on Ia and
  * Ib current feedback analog channels when no current is flowin into the
  * motor
  */
void ICS_CurrentReadingPolarization( PWMC_Handle_t * pHdl );

/**
  * It computes and return latest converted motor phase currents motor
  *
  */
void ICS_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * Iab );

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void ICS_TurnOnLowSides( PWMC_Handle_t * pHdl );

/**
  * It enables PWM generation on the proper Timer peripheral acting on MOE
  * bit
  */
void ICS_SwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit
  */
void ICS_SwitchOffPWM( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling 
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  * And call the WriteTIMRegisters method.
  */
uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHdl );


/**
  *  It contains the TIMx Update event interrupt
  */
void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHdl );

/**
  *  It contains the TIMx Break2 event interrupt
  */
void * ICS_BRK2_IRQHandler( PWMC_ICS_Handle_t * pHdl );

/**
  *  It contains the TIMx Break1 event interrupt
  */
void * ICS_BRK_IRQHandler( PWMC_ICS_Handle_t * pHdl );

/**
  * It is used to check if an overcurrent occurred since last call.
  */
uint16_t ICS_IsOverCurrentOccurred( PWMC_Handle_t * pHdl );


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*ICS_RA6T2_PWMNCURRFDBK_H*/


