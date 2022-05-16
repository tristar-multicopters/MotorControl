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

/* Includes ------------------------------------------------------------------*/
#include "ics_ra6t2_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/* Private defines -----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static bool ICS_TIMxInit( const three_phase_instance_t * TIMx, PWMC_Handle_t * pHdl );
static bool ICS_ADCxInit( const adc_instance_t * ADCx );
static void ICS_HFCurrentsPolarization( PWMC_Handle_t * pHdl,ab_t * Iab );

/**
  * @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  *         in ICS topology using STM32G4X and shared ADC
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
bool ICS_Init( PWMC_ICS_Handle_t * pHandle )
{
	bool err = false;
	
	ICS_ADCxInit(pHandle->pParams_str->pADCHandle);
	ICS_TIMxInit(pHandle->pParams_str->pThreePhaseHandle, &pHandle->_Super);
	
	return err;
}

static bool ICS_ADCxInit( const adc_instance_t * ADCx )
{
	bool err = false;
	
	return err;
}

/**
  * @brief  It initializes TIMx peripheral for PWM generation
  * @param TIMx: Timer to be initialized
  * @param pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static bool ICS_TIMxInit( const three_phase_instance_t * TIMx, PWMC_Handle_t * pHdl )
{
	bool err = false;

	return err;
}

/**
  * @brief  It stores into the component the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void ICS_CurrentReadingPolarization( PWMC_Handle_t * pHdl )
{

}


/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */ 
void ICS_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * Iab )
{
	PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
	
	R_ADC_B_Read(pHandle->pParams_str->pADCHandle->p_ctrl, pHandle->pParams_str->ADCChannelIa, &pHandle->IaRaw);
	R_ADC_B_Read(pHandle->pParams_str->pADCHandle->p_ctrl, pHandle->pParams_str->ADCChannelIb, &pHandle->IbRaw);
	
  int32_t aux;
  uint16_t reg;
  
  /* disable ADC trigger source */
  //LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
  
  /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL value)  */
  reg = ( uint16_t )( pHandle->IaRaw );
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

  /* Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
  reg = ( uint16_t )( pHandle->IbRaw );
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

  pHandle->_Super.Ia = Iab->a;
  pHandle->_Super.Ib = Iab->b;
  pHandle->_Super.Ic = -Iab->a - Iab->b;
}

/**
  * @brief  Stores into the component's handle the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor
  * @param  pHandle handler of the current instance of the PWM component
  * @retval none
  */
uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHdl )
{

}
/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into PhaseAOffset and
  *         PhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is required to proper configure ADC inputs before to enable
  *         the offset computation.
  * @param  pHdl Pointer on the target component instance
  * @retval It always returns {0,0} in Curr_Components format
  */
static void ICS_HFCurrentsPolarization( PWMC_Handle_t * pHdl, ab_t * Iab )
{

}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void ICS_TurnOnLowSides( PWMC_Handle_t * pHdl )
{

}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
  *         bit
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void ICS_SwitchOnPWM( PWMC_Handle_t * pHdl )
{

}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void ICS_SwitchOffPWM( PWMC_Handle_t * pHdl )
{

}

/**
  * @brief  It contains the TIMx Update event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHandle )
{

}

/**
  * @brief  It contains the TIMx Break2 event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void * ICS_BRK2_IRQHandler( PWMC_ICS_Handle_t * pHandle )
{

}

/**
  * @brief  It contains the TIMx Break1 event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void * ICS_BRK_IRQHandler( PWMC_ICS_Handle_t * pHandle )
{

}


/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHdl Pointer on the target component instance
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t ICS_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{

}

