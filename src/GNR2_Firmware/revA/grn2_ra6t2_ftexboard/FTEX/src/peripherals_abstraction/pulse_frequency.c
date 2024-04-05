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
    bIsError =R_GPT_InfoGet(pHandle->PulseFreqParam.PF_Timer->p_ctrl, &info);
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
            //pulse width is almost the rovolution time of the wheel.
           pHandle->wSecondPeriod =(float)(((float)pHandle->wCaptureCount)/((float)info.clock_frequency));
            
        //Read AGT parameter, on this case the number of pulses
        //detected by the AGT.
        
        break;
        
        case AGT_TIMER:
            
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
            pHandle->wCaptureCount = wCapture + (info.period_counts*pHandle->wCaptureOverflow);
            pHandle->wCaptureOverflow = 0;
            //set this variable to indicate the time still running.
            //this is used to avoid the problem of the magnet alignement.
            pHandle->measuring = true;
           
            break;
        case AGT_TIMER:
            
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

            break;
        default:
            ASSERT(false);
        break;
    }
}