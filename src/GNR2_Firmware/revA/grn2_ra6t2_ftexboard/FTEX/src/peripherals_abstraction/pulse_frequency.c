/**
  ******************************************************************************
  * @file    pulse_frequency.c
  * @author  FTEX inc
  * @brief   Pulse frequency reading module using General perepherial Timer
  ******************************************************************************
*/

#include "pulse_frequency.h"
#include "ASSERT_FTEX.h"

// =============================== Defines ================================== //
#define RESET_VALUE     (0x00)      // Reset value for the get info function
timer_info_t info = {.clock_frequency = RESET_VALUE, .count_direction = RESET_VALUE,
                     .period_counts =  RESET_VALUE};
// ======================= Privarte functions ============================ //

/**
  @brief  Function used to Initialize the timer for Capture Mode 
  @param  PulseFrequencyHandle_t handle
  @param  TimerType_t enumaration
  @return bIsError in boolean
*/
bool PulseFrequency_Init(PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    bool bIsError = false;
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:	
        // Initializes the GPT timer module and applies configurations
        bIsError |= R_GPT_Open(pHandle->PulseFreqParam.PF_Timer->p_ctrl, pHandle->PulseFreqParam.PF_Timer->p_cfg);
        break;
        case AGT_TIMER:
        // Initializes the AGT timer module and applies configurations
        bIsError |= R_AGT_Open(pHandle->PulseFreqParam.PF_Timer->p_ctrl, pHandle->PulseFreqParam.PF_Timer->p_cfg);
        break;

        default:
            ASSERT(false);
        break;
    }
    return bIsError;
}	

/**
  @brief  Function used to Deinitialize the timer for Capture Mode 
  @param  PulseFrequencyHandle_t handle
  @param  TimerType_t enumaration
  @return bIsError in boolean
*/
bool PulseFrequency_Deinitialisation(PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    bool bIsError = false;
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:	
        // Deinitializes the GPT timer module and applies configurations
        bIsError |= R_GPT_Stop(pHandle->PulseFreqParam.PF_Timer->p_ctrl);
        break;
        case AGT_TIMER:
        // Deinitializes the AGT timer module and applies configurations
        bIsError |= R_AGT_Stop(pHandle->PulseFreqParam.PF_Timer->p_ctrl);
        break;
        default:
            ASSERT(false);
        break;
    }
    return bIsError;
}

/**
  @brief  Function used to Enable the timer for Capture Mode 
  @param  PulseFrequencyHandle_t handle
  @param  TimerType_t enumaration
  @return bIsError in boolean
*/
bool PulseFrequency_Enable(PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    bool bIsError = false;
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:	
        // Enable the GPT timer module and applies configurations
        bIsError |= R_GPT_Enable(pHandle->PulseFreqParam.PF_Timer->p_ctrl);
        break;
        case AGT_TIMER:
        // Enable the AGT timer module and applies configurations
        bIsError |= R_AGT_Enable(pHandle->PulseFreqParam.PF_Timer->p_ctrl);
        break;
        default:
            ASSERT(false);
        break;
    }

    return bIsError;
}

/**
  @brief  Function used to Disable the timer for Capture Mode 
  @param  PulseFrequencyHandle_t handle
  @param  TimerType_t enumaration
  @return bIsError in boolean
*/
bool PulseFrequency_Disable(PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    bool bIsError = false;
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:
        // Disable the GPT timer module and applies configurations
        bIsError |= R_GPT_Disable(pHandle->PulseFreqParam.PF_Timer->p_ctrl);
        break;
        case AGT_TIMER:
        // Disable the AGT timer module and applies configurations
        bIsError |= R_AGT_Disable(pHandle->PulseFreqParam.PF_Timer->p_ctrl);
        break;
        default:
            ASSERT(false);
        break;
    }
    return bIsError;
}

/**
  @brief  Function used to get the info of the timer
  @param  PulseFrequencyHandle_t handle
  @return bIsError in boolean
*/
bool PulseFrequency_GetTimerInfo(PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    bool bIsError = false;
    /* Get the period count and clock frequency */
    bIsError =R_GPT_InfoGet(pHandle->PulseFreqParam.PF_Timer->p_ctrl, &info);
    return bIsError;
}

// ======================= Public functions ============================ //

/**
    Function used to read the capture read input from the Timer
*/
void PulseFrequency_ReadInputCapture (PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    float pulse_time;
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:
            /* Check for the flag from ISR callback */
            if (pHandle->start_measurement)
            {           
                /* Reset the flag */
                pHandle->start_measurement = false;
                /* Get the Timer Info for Perido check*/
                PulseFrequency_GetTimerInfo(pHandle);
                /* Get the period count and clock frequency */	
                pHandle->wCaptureCount = (info.period_counts * pHandle->wCaptureOverflow) + pHandle->wCaptureCount;
                /* Calculate the pulse time */
                pulse_time =(float)(((float)pHandle->wCaptureCount)/((float)info.clock_frequency));
                pHandle->wUsPeriod = ((uint32_t) (pulse_time * (float)MICRO_SEC)) * PERIOD_FACTOR;
                pHandle->wCaptureOverflow = 0U;
            }
            else 
            {
                /* Reset the capture varables */
                pHandle->wCaptureCount = 0U;
                pHandle->wCaptureOverflow = 0U;
                pHandle->wUsPeriod = 0U;
            }
            break;
            
        //Read AGT parameter, on this case the number of pulses
        //detected by the AGT.
        case AGT_TIMER:

        default:
            ASSERT(false);
            break;
    }
}

/**
    Function used to get the capture Period 	
*/
uint32_t PulseFrequency_GetPeriod(PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:
            /* return the Period */
            return pHandle->wCaptureCount;
            break;
        case AGT_TIMER:
            /* return the Period */
            return pHandle->wCaptureCount;	
            break;
        default:
            ASSERT(false);
            break;
	}
}

/**
    Function used to update the capture variables from Timer interrupt
*/
void PulseFrequency_IsrCallUpdate(PulseFrequencyHandle_t * pHandle ,uint32_t wCapture)
{
    ASSERT(pHandle != NULL);
    
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:
            /* Capture the count in a variable */
            pHandle->wCaptureCount = wCapture;
            /* Set start measurement */
            pHandle->start_measurement = true;
            break;
        case AGT_TIMER:
            
            //Increment the number of pulses detected by the AGT timer.
            pHandle->wNumberOfPulse++;
 
            break;
        default:
            ASSERT(false);
        break;
    }
}

/**
    Function used to update the overflow variable from Timer interrupt
*/
void PulseFrequency_ISROverflowUpdate(PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:
            /* Update the overflow variable for the GPT Timer */
            pHandle->wCaptureOverflow ++;
            break;
        case AGT_TIMER:
            /* Update the overflow variable for the AGT Timer */
            pHandle->wCaptureOverflow ++;	
            break;
        default:
            ASSERT(false);
        break;
    }
}