/**
  * @file    ics_ra6t2_pwm_curr_fdbk.c
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement current measurement
  *          functionality with 2 isolated sensors on A and B phase. It is specifically designed for RA6T2
  *          microcontrollers.
  *           + MCU peripheral and handle initialization function
  *           + two isolated current sensor
  *           + space vector modulation function
  *           + ADC sampling function
  */

/* Includes ------------------------------------------------------------------*/
#include "ics_ra6t2_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"


/* Private function prototypes -----------------------------------------------*/
static void ICS_HFCurrentsPolarization( PWMCurrFdbkHandle_t * pHdl,ab_t * Iab );


bool PWMInsulCurrSensorFdbk_Init( PWMInsulCurrSensorFdbkHandle_t * pHandle )
{
	/* Nothing to initialize for now */
	
	bool bIsError = false;
	
	return bIsError;
}

void PWMInsulCurrSensorFdbk_CurrentReadingPolarization( PWMCurrFdbkHandle_t * pHdl )
{
	PWMInsulCurrSensorFdbkHandle_t * pHandle = ( PWMInsulCurrSensorFdbkHandle_t * )pHdl;

  /* Reset offset and counter */
  pHandle->wPhaseAOffset = 0u;
  pHandle->wPhaseBOffset = 0u;
  pHandle->bPolarizationCounter = 0u;
	
	/* Disable PWM output */
	R_GPT_OutputDisable(pHandle->pParamsStructure->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_U]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	R_GPT_OutputDisable(pHandle->pParamsStructure->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_V]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	R_GPT_OutputDisable(pHandle->pParamsStructure->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_W]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	
  /* Change function to be executed in ADC interrupt routine */
  pHandle->Super.pFctGetPhaseCurrents = &ICS_HFCurrentsPolarization;
	
	/* Start PWM, but output is disabled */
  PWMInsulCurrSensorFdbk_SwitchOnPWM( &pHandle->Super );
	
  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd(&pHandle->bPolarizationCounter);

	/* Stop PWM */
  PWMInsulCurrSensorFdbk_SwitchOffPWM( &pHandle->Super );
	
	/* Compute sensor offsets */
  pHandle->wPhaseAOffset /= NB_CONVERSIONS;
  pHandle->wPhaseBOffset /= NB_CONVERSIONS;

  /* Change back function to be executed in ADC interrupt routine */
  pHandle->Super.pFctGetPhaseCurrents = &PWMInsulCurrSensorFdbk_GetPhaseCurrents;
  pHandle->Super.pFctSetADCSampPointSectX = &PWMInsulCurrSensorFdbk_WriteTIMRegisters;

	/* Enable PWM output */
	R_GPT_OutputEnable(pHandle->pParamsStructure->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_U]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	R_GPT_OutputEnable(pHandle->pParamsStructure->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_V]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	R_GPT_OutputEnable(pHandle->pParamsStructure->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_W]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
}

void PWMInsulCurrSensorFdbk_GetPhaseCurrents( PWMCurrFdbkHandle_t * pHdl, ab_t * Iab )
{
	PWMInsulCurrSensorFdbkHandle_t * pHandle = ( PWMInsulCurrSensorFdbkHandle_t * )pHdl;
	
	pHandle->bOverrunFlag = false;
	
  int32_t aux;
  uint16_t reg;
	
	/* Read ADC converted value and store it into handle  */
	R_ADC_B_Read(pHandle->pParamsStructure->pADCHandle->p_ctrl, pHandle->pParamsStructure->ADCChannelIa, &pHandle->hIaRaw);
	R_ADC_B_Read(pHandle->pParamsStructure->pADCHandle->p_ctrl, pHandle->pParamsStructure->ADCChannelIb, &pHandle->hIbRaw);
  
  /* Ia = (PHASE_A_ADC_CHANNEL value) - (hPhaseAOffset)  */
  reg = ( uint16_t )( pHandle->hIaRaw );
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->wPhaseAOffset );

  /* Saturation of Ia */
  if ( aux < -INT16_MAX )
  {
	  Iab->a = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
	  Iab->a = INT16_MAX;
  }
  else
  {
	  Iab->a = ( int16_t )aux;
  }

  /* Ib = (PHASE_B_ADC_CHANNEL value) - (hPhaseBOffset) */
  reg = ( uint16_t )( pHandle->hIbRaw );
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->wPhaseBOffset );

  /* Saturation of Ib */
  if ( aux < -INT16_MAX )
  {
	  Iab->b = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
	  Iab->b = INT16_MAX;
  }
  else
  {
	  Iab->b = ( int16_t )aux;
  }
	
	/* Inversion of Ia and Ib. Temporary fix. */
	Iab->a = -Iab->a;
	Iab->b = -Iab->b;

	/* Compute Ic from Ia and Ib. Store them into base handle. */
  pHandle->Super.Ia = Iab->a;
  pHandle->Super.Ib = Iab->b;
  pHandle->Super.Ic = -Iab->a - Iab->b;
}

uint16_t PWMInsulCurrSensorFdbk_WriteTIMRegisters( PWMCurrFdbkHandle_t * pHdl )
{
	PWMInsulCurrSensorFdbkHandle_t * pHandle = ( PWMInsulCurrSensorFdbkHandle_t * )pHdl;
	uint16_t hAux = MC_NO_ERROR;
	three_phase_duty_cycle_t sDutyCycle;
	
	/* Set duty cycles according to values in base handle */
	sDutyCycle.duty[THREE_PHASE_CHANNEL_U] = pHandle->Super.hCntPhA;
	sDutyCycle.duty[THREE_PHASE_CHANNEL_V] = pHandle->Super.hCntPhB;
	sDutyCycle.duty[THREE_PHASE_CHANNEL_W] = pHandle->Super.hCntPhC;
	
	/* Update duty cycle registers */
	R_GPT_THREE_PHASE_DutyCycleSet(pHandle->pParamsStructure->pThreePhaseHandle->p_ctrl, &sDutyCycle);
	
	/* Check for overrun condition */
	if (pHandle->bOverrunFlag)
	{
		hAux = MC_FOC_DURATION;
	}

  return hAux;
}

/**
  * @brief  Implementation of PWMCurrFdbk_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseAOffset and
  *         wPhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is required to proper configure ADC inputs before to enable
  *         the offset computation.
	* @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval It always returns {0,0} in ab_t format
  */
static void ICS_HFCurrentsPolarization( PWMCurrFdbkHandle_t * pHdl, ab_t * Iab )
{
	PWMInsulCurrSensorFdbkHandle_t * pHandle = ( PWMInsulCurrSensorFdbkHandle_t * )pHdl;
	
	pHandle->bOverrunFlag = false;
	
	/* Read ADC converted value and store it into handle  */
	R_ADC_B_Read(pHandle->pParamsStructure->pADCHandle->p_ctrl, pHandle->pParamsStructure->ADCChannelIa, &pHandle->hIaRaw);
	R_ADC_B_Read(pHandle->pParamsStructure->pADCHandle->p_ctrl, pHandle->pParamsStructure->ADCChannelIb, &pHandle->hIbRaw);
	
	/* Accumulate ADC converted value into handle until NB_CONVERTIONS is reached. */
  if ( pHandle->bPolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->wPhaseAOffset += pHandle->hIaRaw;
    pHandle->wPhaseBOffset += pHandle->hIbRaw;
    pHandle->bPolarizationCounter++;
  }

  /* During offset calibration no current is flowing in the phases */
  Iab->a = 0;
  Iab->b = 0;
}

void PWMInsulCurrSensorFdbk_TurnOnLowSides( PWMCurrFdbkHandle_t * pHdl )
{
	PWMInsulCurrSensorFdbkHandle_t * pHandle = ( PWMInsulCurrSensorFdbkHandle_t * )pHdl;
	three_phase_duty_cycle_t sDutyCycle;
	
	pHandle->Super.hTurnOnLowSidesAction = true;
	
	/* Set duty cycles to zero, thus making low side switches always close and high sides always open */
	sDutyCycle.duty[THREE_PHASE_CHANNEL_U] = 0;
	sDutyCycle.duty[THREE_PHASE_CHANNEL_V] = 0;
	sDutyCycle.duty[THREE_PHASE_CHANNEL_W] = 0;
	
	/* Update duty cycle registers */
	R_GPT_THREE_PHASE_DutyCycleSet(pHandle->pParamsStructure->pThreePhaseHandle->p_ctrl, &sDutyCycle);
	
	/* Start timer */
	R_GPT_THREE_PHASE_Start(pHandle->pParamsStructure->pThreePhaseHandle->p_ctrl);
}

void PWMInsulCurrSensorFdbk_SwitchOnPWM( PWMCurrFdbkHandle_t * pHdl )
{
	PWMInsulCurrSensorFdbkHandle_t * pHandle = ( PWMInsulCurrSensorFdbkHandle_t * )pHdl;
	three_phase_duty_cycle_t sDutyCycle;
	
	pHandle->Super.hTurnOnLowSidesAction = false;
	
  /* Set all duty cycles to 50% */
  sDutyCycle.duty[THREE_PHASE_CHANNEL_U] = pHandle->hHalfPWMPeriod / 2;
  sDutyCycle.duty[THREE_PHASE_CHANNEL_V] = pHandle->hHalfPWMPeriod / 2;
  sDutyCycle.duty[THREE_PHASE_CHANNEL_W] = pHandle->hHalfPWMPeriod / 2;
	
	/* Update duty cycle registers */
	R_GPT_THREE_PHASE_DutyCycleSet(pHandle->pParamsStructure->pThreePhaseHandle->p_ctrl, &sDutyCycle);
	
	/* Start timer */
	R_GPT_THREE_PHASE_Start(pHandle->pParamsStructure->pThreePhaseHandle->p_ctrl);
}

void PWMInsulCurrSensorFdbk_SwitchOffPWM( PWMCurrFdbkHandle_t * pHdl )
{
	PWMInsulCurrSensorFdbkHandle_t * pHandle = ( PWMInsulCurrSensorFdbkHandle_t * )pHdl;
	
	pHandle->Super.hTurnOnLowSidesAction = false;
	
	/* Stop timer and reset PWM output */
	R_GPT_THREE_PHASE_Stop(pHandle->pParamsStructure->pThreePhaseHandle->p_ctrl);
}

void * PWMInsulCurrSensorFdbk_TIMx_UP_IRQHandler( PWMInsulCurrSensorFdbkHandle_t * pHdl )
{
	/* Make ADC ready for next conversion, will be triggered by timer */
	R_ADC_B_ScanGroupStart(pHdl->pParamsStructure->pADCHandle->p_ctrl, pHdl->pParamsStructure->ADCGroupMask);
	
	pHdl->bOverrunFlag = true;
	
	return &( pHdl->Super.Motor );
}

void * PWMInsulCurrSensorFdbk_BRK_IRQHandler( PWMInsulCurrSensorFdbkHandle_t * pHdl )
{
	/* Stop POEG module so it does not reenter the interrupt twice */
	R_POEG_Close(pHdl->pParamsStructure->pPOEGHandle->p_ctrl);
	
  pHdl->bOverCurrentFlag = true;

  return &( pHdl->Super.Motor );
}

uint16_t PWMInsulCurrSensorFdbk_IsOverCurrentOccurred( PWMCurrFdbkHandle_t * pHdl )
{
	PWMInsulCurrSensorFdbkHandle_t * pHandle = ( PWMInsulCurrSensorFdbkHandle_t * )pHdl;
	
  uint16_t retVal = MC_NO_FAULTS;

  if ( pHandle->bOverCurrentFlag == true )
  {
    retVal |= MC_BREAK_IN;
    pHandle->bOverCurrentFlag = false;
  }

  return retVal;
}

