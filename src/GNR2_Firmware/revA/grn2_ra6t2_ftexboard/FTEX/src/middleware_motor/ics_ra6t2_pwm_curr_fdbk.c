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
#include "board_hardware.h"
#include "hal_data.h"


/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Implementation of PWMCurrFdbk_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseAOffset and
  *         wPhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is required to proper configure ADC inputs before to enable
  *         the offset computation.
    * @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval It always returns {0,0} in ab_t format
  */
static void ICS_HFCurrentsPolarization(PWMCurrFdbkHandle_t * pHdl,ab_t * Iab);

/* Public function prototypes -----------------------------------------------*/

/**
  * @brief  It initializes the module and its hardware components.
  * @param  pHandle: handle of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval true if initialization is successful
  */
bool PWMInsulCurrSensorFdbk_Init(PWMInsulCurrSensorFdbkHandle_t * pHandle)
{
    bool bIsError = false;
    
    bIsError = PWMCurrFdbk_Init(&pHandle->Super);

    return bIsError;
}

/**
  * @brief  This function starts the current sensor polarization routine. It stores into the provided handle 
    *       the voltage present on Ia and Ib current feedback analog channels when no current is flowing into the
  *         motor.
  * @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval none
  */
void PWMInsulCurrSensorFdbk_CurrentReadingPolarization(PWMCurrFdbkHandle_t * pHdl)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;

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
    PWMInsulCurrSensorFdbk_SwitchOnPWM(&pHandle->Super);

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd(&pHandle->bPolarizationCounter);
    
    /* Stop PWM */
    PWMInsulCurrSensorFdbk_SwitchOffPWM(&pHandle->Super);
    
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

/**
  * @brief  This function computes and return latest converted motor phase currents motor
  * @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval Ia and Ib current in ab_t format
  */ 
void PWMInsulCurrSensorFdbk_GetPhaseCurrents(PWMCurrFdbkHandle_t * pHdl, ab_t * Iab)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;
    
    pHandle->bOverrunFlag = false;

    int32_t aux;
    uint16_t reg;
    
    /* Read ADC converted value and store it into handle  */
    R_ADC_B_Read(pHandle->pParamsStructure->pADCHandle->p_ctrl, pHandle->pParamsStructure->ADCChannelIa, &pHandle->hIaRaw);
    R_ADC_B_Read(pHandle->pParamsStructure->pADCHandle->p_ctrl, pHandle->pParamsStructure->ADCChannelIb, &pHandle->hIbRaw);
  
    /* Ia = (PHASE_A_ADC_CHANNEL value) - (hPhaseAOffset)  */
    reg = (uint16_t)(pHandle->hIaRaw);
    aux = (int32_t)(reg) - (int32_t)(pHandle->wPhaseAOffset);

    /* Saturation of Ia */
    if (aux < -INT16_MAX)
    {
        Iab->a = -INT16_MAX;
    }
    else  if (aux > INT16_MAX)
    {
        Iab->a = INT16_MAX;
    }
    else
    {
        Iab->a = (int16_t)aux;
    }

    /* Ib = (PHASE_B_ADC_CHANNEL value) - (hPhaseBOffset) */
    reg = (uint16_t)(pHandle->hIbRaw);
    aux = (int32_t)(reg) - (int32_t)(pHandle->wPhaseBOffset);

    /* Saturation of Ib */
    if (aux < -INT16_MAX)
    {
      Iab->b = -INT16_MAX;
    }
    else  if (aux > INT16_MAX)
    {
      Iab->b = INT16_MAX;
    }
    else
    {
      Iab->b = (int16_t)aux;
    }

    /* NOTE: Ia and Ib need to be inverted (to investigate later). Since it is already inverted in hardware gnr2 revA,
            no need to do by software. */

    /* Compute Ic from Ia and Ib. Store them into base handle. */
    pHandle->Super.Ia = Iab->a;
    pHandle->Super.Ib = Iab->b;
    pHandle->Super.Ic = -Iab->a - Iab->b;
}

/**
  * @brief  Function to update duty cycle registers.
  * @param  pHdl: handle of the current instance of the PWMInsulCurrSensorFdbkHandle_t component.
  * @retval Motor control error code: MC_FOC_DURATION if overrun occured, MC_NO_FAULT otherwise.
  */
uint32_t PWMInsulCurrSensorFdbk_WriteTIMRegisters(PWMCurrFdbkHandle_t * pHdl)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;
    uint32_t hAux = MC_NO_FAULT;
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
static void ICS_HFCurrentsPolarization(PWMCurrFdbkHandle_t * pHdl, ab_t * Iab)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;
    
    pHandle->bOverrunFlag = false;
    
    /* Read ADC converted value and store it into handle  */
    R_ADC_B_Read(pHandle->pParamsStructure->pADCHandle->p_ctrl, pHandle->pParamsStructure->ADCChannelIa, &pHandle->hIaRaw);
    R_ADC_B_Read(pHandle->pParamsStructure->pADCHandle->p_ctrl, pHandle->pParamsStructure->ADCChannelIb, &pHandle->hIbRaw);
    
    /* Accumulate ADC converted value into handle until NB_CONVERTIONS is reached. */
    if (pHandle->bPolarizationCounter < NB_CONVERSIONS)
    {
        pHandle->wPhaseAOffset += pHandle->hIaRaw;
        pHandle->wPhaseBOffset += pHandle->hIbRaw;
        pHandle->bPolarizationCounter++;
    }

    /* During offset calibration no current is flowing in the phases */
    Iab->a = 0;
    Iab->b = 0;
}

/**
  * @brief  Function to turn on low sides switches. This function is intended to be
  *         used for charging boot capacitors. It has to be
  *         called at each motor start-up.
  * @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval none
  */
void PWMInsulCurrSensorFdbk_TurnOnLowSides(PWMCurrFdbkHandle_t * pHdl)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;
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

/**
  * @brief  It enables PWM generation
  * @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval none
  */
void PWMInsulCurrSensorFdbk_SwitchOnPWM(PWMCurrFdbkHandle_t * pHdl)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;
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

/**
  * @brief  It stops PWM generation
  * @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval none
  */
void PWMInsulCurrSensorFdbk_SwitchOffPWM(PWMCurrFdbkHandle_t * pHdl)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;
    
    pHandle->Super.hTurnOnLowSidesAction = false;
    
    /* Stop timer and reset PWM output */
    R_GPT_THREE_PHASE_Stop(pHandle->pParamsStructure->pThreePhaseHandle->p_ctrl);
}

/**
  * @brief  It is the routine to run when PWM timer update event happens
  * @param  pHandle: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval Motor instance number
  */
void * PWMInsulCurrSensorFdbk_TIMx_UP_IRQHandler(PWMInsulCurrSensorFdbkHandle_t * pHdl)
{
    /* Make ADC ready for next conversion, will be triggered by timer */
    R_ADC_B_ScanGroupStart(pHdl->pParamsStructure->pADCHandle->p_ctrl, pHdl->pParamsStructure->ADCGroupMask);
    
    pHdl->bOverrunFlag = true;
    
    return &(pHdl->Super.Motor);
}

/**
  * @brief  It is the routine to run when OCD1 trigger interrupt occured
  * @param  pHandle: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval Motor instance number
  */
void * PWMInsulCurrSensorFdbk_OCD1_IRQHandler(PWMInsulCurrSensorFdbkHandle_t * pHdl)
{
    pHdl->bOCD1Flag = true;

    return &(pHdl->Super.Motor);
}

/**
  * @brief  It is the routine to run when OCD2 trigger interrupt occured
  * @param  pHandle: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval Motor instance number
  */
void * PWMInsulCurrSensorFdbk_OCD2_IRQHandler(PWMInsulCurrSensorFdbkHandle_t * pHdl)
{    
    pHdl->bOCD2Flag = true;

    return &(pHdl->Super.Motor);
}

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval uint16_t It returns MC_OCD1 or MC_OCD2 whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULT otherwise.
  */
uint32_t PWMInsulCurrSensorFdbk_IsOverCurrentOccurred(PWMCurrFdbkHandle_t * pHdl)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;
    
    uint32_t retVal = MC_NO_FAULT;
    
    //reset OCD1 flag
    if (pHandle->bOCD1Flag == true)
    {
        retVal |= MC_OCD1;
    }
    
    #if OCDX_POEG == OCD2_POEG
        //reset OCD2 flag and the POEG
        if (pHandle->bOCD2Flag == true)
        {
            retVal |= MC_OCD2;
            pHandle->bOCD2Flag = false;
            R_POEG_Reset((PWM_POEG0_HANDLE_ADDRESS)->p_ctrl);
        }
    #endif

  return retVal;
}

#if OCDX_POEG == OCD1_POEG
/**
  * @brief  It is used to check if OCD2 occurred since last call.
  * @param  pHdl: handler of the current instance of the PWMInsulCurrSensorFdbkHandle_t component
  * @retval uint16_t It returns MC_OCD2 if OCD2 has been
  *                  detected since last method call, MC_NO_FAULT otherwise.
  */
uint32_t PWMInsulCurrSensorFdbk_OCD2Occurred(PWMCurrFdbkHandle_t * pHdl)
{
    PWMInsulCurrSensorFdbkHandle_t * pHandle = (PWMInsulCurrSensorFdbkHandle_t *)pHdl;
    uint32_t retVal = MC_NO_FAULT;
    
    //reset OCD2 flag
    if (pHandle->bOCD2Flag == true)
    {
        retVal |= MC_OCD2;
        pHandle->bOCD2Flag = false;
    }

  return retVal;
}
#endif

