/**
  ******************************************************************************
  * @file    pulse_frequency_gpt.c
  * @author  FTEX inc
  * @brief   Pulse frequency reading module using General perepherial Timer
  ******************************************************************************
*/

#include "pulse_frequency_gpt.h"

// ======================= Privarte functions ============================ //

/**
	@brief  Function used to Initialize the General Purpose Timer for Capture Mode 
  @param  PF_Handle_GPT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Initialisation_GPT(PF_Handle_GPT_t * pHandle)
{
	bool bIsError = false;
	bIsError |= R_GPT_Open(pHandle->PF_Param_GPT.PF_GPT_Timer->p_ctrl, pHandle->PF_Param_GPT.PF_GPT_Timer->p_cfg);
	return bIsError;
}

/**
	@brief  Function used to Deinitialize the General Purpose Timer for Capture Mode 
  @param  PF_Handle_GPT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Deinitialisation_GPT(PF_Handle_GPT_t * pHandle)
{
	bool bIsError = false;
	bIsError |= R_GPT_Stop(pHandle->PF_Param_GPT.PF_GPT_Timer->p_ctrl);
	return bIsError;
}

/**
	@brief  Function used to Enable the General Purpose Timer for Capture Mode 
  @param  PF_Handle_GPT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Enable_GPT(PF_Handle_GPT_t * pHandle)
{
	bool bIsError = false;
	bIsError |= R_GPT_Enable(pHandle->PF_Param_GPT.PF_GPT_Timer->p_ctrl);
	return bIsError;
}

/**
	@brief  Function used to Disable the General Purpose Timer for Capture Mode 
  @param  PF_Handle_GPT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Disable_GPT(PF_Handle_GPT_t * pHandle)
{
	bool bIsError = false;

		bIsError |= R_GPT_Disable(pHandle->PF_Param_GPT.PF_GPT_Timer->p_ctrl);
	return bIsError;
}

/**
	@brief  Function used to Reset the General Purpose Timer for Capture Mode 
  @param  PF_Handle_GPT_t
  @return bIsError in boolean
*/
bool PulseFrequency_Reset_GPT(PF_Handle_GPT_t * pHandle)
{
	bool bIsError = false;

		bIsError |= R_GPT_Reset(pHandle->PF_Param_GPT.PF_GPT_Timer->p_ctrl);
	return bIsError;
}
// ======================= Public functions ============================ //

/**
	Function used to read the capture read input from the Genral Purpose Timer
*/
void PulseFrequency_ReadInputCapture_GPT (PF_Handle_GPT_t * pHandle)
{
	/* Check for the flag from ISR callback */
	if (pHandle->gpt_start_measurement)
	{           
		/* Reset the flag */
		pHandle->gpt_start_measurement = false;
		/* Get the period count and clock frequency */	
		pHandle->gpt_capture_count = pHandle->gpt_capture_overflow + pHandle->gpt_capture_count;
	}
	else 
	{
		pHandle->gpt_capture_count = 0;
	}
}

/**
  Function used to return the capture read input from the Genral Purpose Timer
*/
uint64_t PulseFrequency_InputCaptureValue_GPT (PF_Handle_GPT_t * pHandle)
{
	return pHandle->gpt_capture_count;
}	

/**
  Function used to update the capture variables from the Genral Purpose Timer interrupt
*/
void PulseFrequency_IsrCallUpdate_GPT( PF_Handle_GPT_t * pHandle , uint64_t wCapture )
{
	/* Capture the count in a variable */
	pHandle->gpt_capture_count = wCapture;
	/* Set start measurement */
	pHandle->gpt_start_measurement = true;
}

/**
  Function used to update the overflow variable from the Genral Purpose Timer interrupt
*/
void PulseFrequency_IsrOverFlowUpdate_GPT( PF_Handle_GPT_t * pHandle )
{
	pHandle->gpt_capture_overflow ++;
}