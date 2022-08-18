/**
  ******************************************************************************
  * @file    pulse_frequency_agt.c
  * @author  FTEX inc
  * @brief   Pulse frequency reading module using Low power Timer
  ******************************************************************************
*/

#include "pulse_frequency_agt.h"

// ======================= Privarte functions ============================ //

/**
  @brief  Function used to Initialize the Low Power Timer for Capture Mode 
  @param  PulseFrequency_Handle_AGT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Initialisation_AGT(PulseFrequency_Handle_AGT_t * pHandle)
{
    bool bIsError = false;
    bIsError |= R_GPT_Open(pHandle->PulseFreqParam_AGT.PF_AGT_Timer->p_ctrl, pHandle->PulseFreqParam_AGT.PF_AGT_Timer->p_cfg);
    return bIsError;
}

/**
  @brief  Function used to Deinitialize the Low Power Timer for Capture Mode 
  @param  PulseFrequency_Handle_AGT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Deinitialisation_AGT(PulseFrequency_Handle_AGT_t * pHandle)
{
    bool bIsError = false;
    bIsError |= R_GPT_Stop(pHandle->PulseFreqParam_AGT.PF_AGT_Timer->p_ctrl);
    return bIsError;
}

/**
  @brief  Function used to Enable the Low Power Timer for Capture Mode 
  @param  PulseFrequency_Handle_AGT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Enable_AGT(PulseFrequency_Handle_AGT_t * pHandle)
{
    bool bIsError = false;
    bIsError |= R_GPT_Enable(pHandle->PulseFreqParam_AGT.PF_AGT_Timer->p_ctrl);
    return bIsError;
}

/**
  @brief  Function used to Disable the Low Power Timer for Capture Mode 
  @param  PulseFrequency_Handle_AGT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Disable_AGT(PulseFrequency_Handle_AGT_t * pHandle)
{
    bool bIsError = false;
    bIsError |= R_GPT_Disable(pHandle->PulseFreqParam_AGT.PF_AGT_Timer->p_ctrl);
    return bIsError;
}

/**
  @brief  Function used to Reset the Low Power Timer for Capture Mode 
  @param  PulseFrequency_Handle_AGT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Reset_AGT(PulseFrequency_Handle_AGT_t * pHandle)
{
    bool bIsError = false;
    bIsError |= R_GPT_Reset(pHandle->PulseFreqParam_AGT.PF_AGT_Timer->p_ctrl);
    return bIsError;
}

// ======================= Public functions ============================ //

/**
	Function used to read the capture read input from the Low Power Timer
*/
void PulseFrequency_ReadInputCapture_AGT (PulseFrequency_Handle_AGT_t * pHandle)
{
    /* Check for the flag from ISR callback */
    if (pHandle->agt_start_measurement)
    {
        /* Reset the flag */
        pHandle->agt_start_measurement = false;
        /* Get the period count and clock frequency */	
        pHandle->agt_capture_count = pHandle->agt_capture_count;
    } 
    else 
    {
        pHandle->agt_capture_count = 0;
    }
}

/**
  Function used to return the capture read input from the Low Power Timer
*/
uint64_t PulseFrequency_InputCaptureValue_AGT (PulseFrequency_Handle_AGT_t * pHandle)
{
    return pHandle->agt_capture_count;
}	

/**
  Function used to update the capture variables from the Low Power Timer interrupt
*/
void PulseFrequency_IsrCallUpdate_AGT(PulseFrequency_Handle_AGT_t * pHandle , uint64_t  wCapture)
{
    /* Capture the count in a variable */
    pHandle->agt_capture_count = wCapture;
    /* Set start measurement */
    pHandle->agt_start_measurement = true;
}

/**
  Function used to update the overflow variable from the Low Power Timer interrupt
*/
void PulseFrequency_IsrOverFlowUpdate_AGT (PulseFrequency_Handle_AGT_t * pHandle)
{
    pHandle->agt_capture_overflow ++;
}

