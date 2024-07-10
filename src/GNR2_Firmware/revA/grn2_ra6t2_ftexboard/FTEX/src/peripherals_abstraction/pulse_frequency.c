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

//Timer information for GPT wheel timer
timer_info_t wheel_timer_info = {.clock_frequency = RESET_VALUE, .count_direction = RESET_VALUE,
                                 .period_counts =  RESET_VALUE};

//Timer information for AGPT Pedal azsistance pulse width measurement timer
timer_info_t pedal_timer_info = {.clock_frequency = RESET_VALUE, .count_direction = RESET_VALUE,
                                 .period_counts =  RESET_VALUE};

                     
// ======================= Privarte functions ============================ //




// ======================= Public functions ============================ //
                     
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
    switch (pHandle->TimerType)
    {
        case GPT_TIMER:
            bIsError =R_GPT_InfoGet(pHandle->PulseFreqParam.PF_Timer->p_ctrl, &wheel_timer_info);
        break;
        case AGT_TIMER:
            bIsError =R_AGT_InfoGet(pHandle->PulseFreqParam.PF_Timer->p_ctrl, &pedal_timer_info);
        break;
    }
    return bIsError;
}

/**
    Function used to read the capture read input from the Timer
*/
void PulseFrequency_ReadInputCapture (PulseFrequencyHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    switch (pHandle->TimerType)
    {
        //request to read wheel speed period
        case GPT_TIMER:
            /* Calculate the pulse width on microseconds */
            //pulse width is almost the revolution time of the wheel.
           pHandle->wSecondPeriod =(float)(((float)pHandle->wCaptureCount)/((float)wheel_timer_info.clock_frequency));
            
        break;
        //request to read Pedal period        
        case AGT_TIMER:
            pHandle->wSecondPeriod =(float)(((float)pHandle->wCaptureCount)/((float)pedal_timer_info.clock_frequency));
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
            //A measured was done.
            pHandle->wCaptureCount = wCapture + (wheel_timer_info.period_counts*pHandle->wCaptureOverflow);
            pHandle->wCaptureOverflow = 0;
            //set this variable to indicate the time still running.
            //this is used to avoid the problem of the magnet alignement.
            pHandle->measuring = true;
           
            break;
        case AGT_TIMER:
            /* Capture the count in a variable */
            //A measured was done.
            pHandle->wCaptureCount = wCapture + (pedal_timer_info.period_counts*pHandle->wCaptureOverflow);
            pHandle->wCaptureOverflow = 0;
           
            //Increment the number of pulses detected by the AGT timer.
            pHandle->hNumberOfPulse++;
 
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
            //increment on this variable means the maximum wait
            //time to measure the width of the wheel speed was reached.
            //speed will be set as zero.
            pHandle->wCaptureOverflow ++;
        
            //set this variable to indicate the time still running.
            //this is used to avoid the problem of the magnet alignement.
            pHandle->measuring = true;
        
            //maximum number of allowed overflows.
            if (pHandle->wCaptureOverflow > MAXNUMBER_OVERFLOW_WHEELSPEED)
            {
                /* Capture the count in a variable */
                pHandle->wCaptureCount = 0;
                
                //set overflow to the max allowed
                pHandle->wCaptureOverflow = MAXNUMBER_OVERFLOW_WHEELSPEED + 1;
            }
            break;
        case AGT_TIMER:
            /* Update the overflow variable for the AGT Timer */
            //increment on this variable means the maximum wait
            //time to measure the width of the edal speed was reached.
            pHandle->wCaptureOverflow ++;
            break;
        default:
            ASSERT(false);
        break;
    }
}