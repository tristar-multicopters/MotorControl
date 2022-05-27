/**
  ******************************************************************************
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
static void ICS_HFCurrentsPolarization( PWMC_Handle_t * pHdl,ab_t * Iab );


bool ICS_Init( PWMC_ICS_Handle_t * pHandle )
{
	/* Nothing to initialize for now */
	
	bool bIsError = false;
	
	return bIsError;
}

void ICS_CurrentReadingPolarization( PWMC_Handle_t * pHdl )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;

  /* Reset offset and counter */
  pHandle->PhaseAOffset = 0u;
  pHandle->PhaseBOffset = 0u;
  pHandle->PolarizationCounter = 0u;
	
	/* Disable PWM output */
	R_GPT_OutputDisable(pHandle->pParams_str->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_U]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	R_GPT_OutputDisable(pHandle->pParams_str->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_V]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	R_GPT_OutputDisable(pHandle->pParams_str->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_W]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	
  /* Change function to be executed in ADC interrupt routine */
  pHandle->_Super.pFctGetPhaseCurrents = &ICS_HFCurrentsPolarization;
	
	/* Start PWM, but output is disabled */
  ICS_SwitchOnPWM( &pHandle->_Super );
	
  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd(&pHandle->PolarizationCounter);

	/* Stop PWM */
  ICS_SwitchOffPWM( &pHandle->_Super );
	
	/* Compute sensor offsets */
  pHandle->PhaseAOffset /= NB_CONVERSIONS;
  pHandle->PhaseBOffset /= NB_CONVERSIONS;

  /* Change back function to be executed in ADC interrupt routine */
  pHandle->_Super.pFctGetPhaseCurrents = &ICS_GetPhaseCurrents;
  pHandle->_Super.pFctSetADCSampPointSectX = &ICS_WriteTIMRegisters;

	/* Enable PWM output */
	R_GPT_OutputEnable(pHandle->pParams_str->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_U]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	R_GPT_OutputEnable(pHandle->pParams_str->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_V]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
	R_GPT_OutputEnable(pHandle->pParams_str->pThreePhaseHandle->p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_W]->p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
}

void ICS_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * Iab )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
	
	pHandle->bOverrunFlag = false;
	
  int32_t aux;
  uint16_t reg;
	
	/* Read ADC converted value and store it into handle  */
	R_ADC_B_Read(pHandle->pParams_str->pADCHandle->p_ctrl, pHandle->pParams_str->ADCChannelIa, &pHandle->hIaRaw);
	R_ADC_B_Read(pHandle->pParams_str->pADCHandle->p_ctrl, pHandle->pParams_str->ADCChannelIb, &pHandle->hIbRaw);
  
  /* Ia = (PHASE_A_ADC_CHANNEL value) - (hPhaseAOffset)  */
  reg = ( uint16_t )( pHandle->hIaRaw );
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->PhaseAOffset );

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
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->PhaseBOffset );

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
  pHandle->_Super.Ia = Iab->a;
  pHandle->_Super.Ib = Iab->b;
  pHandle->_Super.Ic = -Iab->a - Iab->b;
}

uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHdl )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
	uint16_t hAux = MC_NO_ERROR;
	three_phase_duty_cycle_t sDutyCycle;
	
	/* Set duty cycles according to values in base handle */
	sDutyCycle.duty[THREE_PHASE_CHANNEL_U] = pHandle->_Super.CntPhA;
	sDutyCycle.duty[THREE_PHASE_CHANNEL_V] = pHandle->_Super.CntPhB;
	sDutyCycle.duty[THREE_PHASE_CHANNEL_W] = pHandle->_Super.CntPhC;
	
	/* Update duty cycle registers */
	R_GPT_THREE_PHASE_DutyCycleSet(pHandle->pParams_str->pThreePhaseHandle->p_ctrl, &sDutyCycle);
	
	/* Check for overrun condition */
	if (pHandle->bOverrunFlag)
	{
		hAux = MC_FOC_DURATION;
	}

  return hAux;
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into PhaseAOffset and
  *         PhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is required to proper configure ADC inputs before to enable
  *         the offset computation.
	* @param  pHdl: handler of the current instance of the PWMC_ICS_Handle_t component
  * @retval It always returns {0,0} in ab_t format
  */
static void ICS_HFCurrentsPolarization( PWMC_Handle_t * pHdl, ab_t * Iab )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
	
	pHandle->bOverrunFlag = false;
	
	/* Read ADC converted value and store it into handle  */
	R_ADC_B_Read(pHandle->pParams_str->pADCHandle->p_ctrl, pHandle->pParams_str->ADCChannelIa, &pHandle->hIaRaw);
	R_ADC_B_Read(pHandle->pParams_str->pADCHandle->p_ctrl, pHandle->pParams_str->ADCChannelIb, &pHandle->hIbRaw);
	
	/* Accumulate ADC converted value into handle until NB_CONVERTIONS is reached. */
  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseAOffset += pHandle->hIaRaw;
    pHandle->PhaseBOffset += pHandle->hIbRaw;
    pHandle->PolarizationCounter++;
  }

  /* During offset calibration no current is flowing in the phases */
  Iab->a = 0;
  Iab->b = 0;
}

void ICS_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
	three_phase_duty_cycle_t sDutyCycle;
	
	pHandle->_Super.TurnOnLowSidesAction = true;
	
	/* Set duty cycles to zero, thus making low side switches always close and high sides always open */
	sDutyCycle.duty[THREE_PHASE_CHANNEL_U] = 0;
	sDutyCycle.duty[THREE_PHASE_CHANNEL_V] = 0;
	sDutyCycle.duty[THREE_PHASE_CHANNEL_W] = 0;
	
	/* Update duty cycle registers */
	R_GPT_THREE_PHASE_DutyCycleSet(pHandle->pParams_str->pThreePhaseHandle->p_ctrl, &sDutyCycle);
	
	/* Start timer */
	R_GPT_THREE_PHASE_Start(pHandle->pParams_str->pThreePhaseHandle->p_ctrl);
}

void ICS_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
	three_phase_duty_cycle_t sDutyCycle;
	
	pHandle->_Super.TurnOnLowSidesAction = false;
	
  /* Set all duty cycles to 50% */
  sDutyCycle.duty[THREE_PHASE_CHANNEL_U] = pHandle->Half_PWMPeriod / 2;
  sDutyCycle.duty[THREE_PHASE_CHANNEL_V] = pHandle->Half_PWMPeriod / 2;
  sDutyCycle.duty[THREE_PHASE_CHANNEL_W] = pHandle->Half_PWMPeriod / 2;
	
	/* Update duty cycle registers */
	R_GPT_THREE_PHASE_DutyCycleSet(pHandle->pParams_str->pThreePhaseHandle->p_ctrl, &sDutyCycle);
	
	/* Start timer */
	R_GPT_THREE_PHASE_Start(pHandle->pParams_str->pThreePhaseHandle->p_ctrl);
}

void ICS_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
	
	pHandle->_Super.TurnOnLowSidesAction = false;
	
	/* Stop timer and reset PWM output */
	R_GPT_THREE_PHASE_Stop(pHandle->pParams_str->pThreePhaseHandle->p_ctrl);
}

void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHdl )
{
	/* Make ADC ready for next conversion, will be triggered by timer */
	R_ADC_B_ScanGroupStart(pHdl->pParams_str->pADCHandle->p_ctrl, pHdl->pParams_str->ADCGroupMask);
	
	pHdl->bOverrunFlag = true;
	
	return &( pHdl->_Super.Motor );
}

void * ICS_BRK2_IRQHandler( PWMC_ICS_Handle_t * pHdl )
{
  pHdl->bOverCurrentFlag = true;

  return &( pHdl->_Super.Motor );
}

uint16_t ICS_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
	
  uint16_t retVal = MC_NO_FAULTS;

  if ( pHandle->bOverCurrentFlag == true )
  {
    retVal |= MC_BREAK_IN;
    pHandle->bOverCurrentFlag = false;
  }

  return retVal;
}

