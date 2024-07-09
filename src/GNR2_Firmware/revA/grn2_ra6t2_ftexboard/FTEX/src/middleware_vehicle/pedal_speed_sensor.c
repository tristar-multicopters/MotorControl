/**
  ******************************************************************************
  * @file    pedal_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal speed sensor
  ******************************************************************************
*/
#include "pedal_speed_sensor.h"
#include "ASSERT_FTEX.h"
#include "vc_parameters.h"
#include "hal_data.h"
#include "board_hardware.h"

//===================== Defines =========================================== //
#define MAX_ALLOWED_PERIOD_SEC     120  //Period of measurement for RMP in sec, two minutes

PulseFrequencyHandle_t pssPulseFrequency = 
{
    .TimerType = AGT_TIMER,    
    .measuring = false, 
    .hNumberOfPulse = 0,        
    .PulseFreqParam =
    {
        .PF_Timer = PEDAL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS,
    }
};

PedalSpeedSensorHandle_t pss = 
{
    .pPulseFrequency = &pssPulseFrequency,
    .minPulsesStartup = STARTUP_PULSE_NUMBER,
    .windowStartup = STARTUP_TIME_WINDOW,
    .minPulsesRunning = RUNTIME_PULSE_NUMBER,
    .windowRunning = RUNTIME_TIME_WINDOW,
    .resetWindowsFlag = false,
    .previousNumberOfPulses = 0,
};

// ==================== Public function prototypes ======================== //
/**
    Pedal Speed Sensor module Initialization
*/
void PSS_Init()
{
    PulseFrequency_GetTimerInfo(pss.pPulseFrequency);
}

/**
    Pedal Speed Sensor capture the number of pulses detected from the 
    cadence signal/sensor.
*/
void PSS_ReadNumberOfPulses()
{    
    //PulseFrequency_ReadInputCapture (pHandle->pPulseFrequency); 
    pss.numberOfPulses = (uint16_t) pss.pPulseFrequency->hNumberOfPulse;  
    //Initialize the number of pulses detected by the AGT Timer.
    pss.pPulseFrequency->hNumberOfPulse = 0;
}

/**
    Pedal Speed Sensor Periode Get value
*/
uint16_t PSS_GetNumberOfPulses()
{   
    return pss.numberOfPulses; 
}

/**
    Reset all variables used to hold the number of pulses measured
    by the AGT timer.
*/
void PSS_ResetValue()
{
    pss.numberOfPulses = 0;
    //Initialize the number of pulses detected by the AGT Timer.
    pss.pPulseFrequency->hNumberOfPulse = 0;
}

/**
    Set windows reset flag 
*/
void PSS_SetWindowsFlag()
{    
    pss.resetWindowsFlag = true;
}

/**
    Clear windows reset flag
*/
void PSS_ClearWindowsFlag()
{    
    pss.resetWindowsFlag = false;
}

/**
    Get windows reset flag
*/
bool PSS_GetWindowsFlag()
{    
    return pss.resetWindowsFlag;
}

/**
    Pedal Speed Sensor RPM Get value
*/
uint16_t PSS_GetSpeedRPM()
{    
    return pss.RPM;
}

/**
    Check if we have a new number of pulses detected from the previous number of pulses detected 
*/
bool PSS_NewPedalPulsesDetected()
{
    // Validate if we have new pulses detected, compared to the previous number of pulses
    if(pss.previousNumberOfPulses != pss.pPulseFrequency->hNumberOfPulse)
    {
        // Update the previous number of pulse with the current one   
        pss.previousNumberOfPulses = pss.pPulseFrequency->hNumberOfPulse;
        return true;
    }

    return false;   
}

/**
    Pedal Speed Sensor calculate periode value
*/
void PSS_CalculateRPM()
{
    //update basic wheel speed information.
    //Calculate the basic parameters of the Input PAS Sensor
    PulseFrequency_ReadInputCapture(pss.pPulseFrequency); 
    // Get the total period time. Since the timer is configured to measure the pulse width
    float period = pss.pPulseFrequency->wSecondPeriod;
    // Prevent division by zero.    
    static const float EPSILON = 1e-6f;
    static const uint16_t ONE_DECIMAL_PLACE = 10; 
    if (period > EPSILON && period < MAX_ALLOWED_PERIOD_SEC && pss.magnetsCount > 0)
    {
        pss.RPM = (uint16_t)(((float)(RPMCOEFF * ONE_DECIMAL_PLACE))/(period * pss.magnetsCount));
    }
    else
    {
        pss.RPM = 0; 
    }
}

/**
    Getter for the pss startup window
 */
uint16_t PSS_GetStartupWindow(void)
{
    return pss.windowStartup;
}

/**
    Getter for the number of pulses
    required for startup activation
 */
uint32_t PSS_GetStartupPulsesCount(void)
{
    return pss.minPulsesStartup;
}

/**
    Getter for the pss running window
 */
uint16_t PSS_GetRunningWindow(void)
{
    return pss.windowRunning;
}

/**
    Getter for the number of pulses
    required for running activation
 */
uint32_t PSS_GetRunningPulsesCount(void)
{
    return pss.minPulsesRunning;
}

/**
    Setter for the pss startup window
 */
void PSS_SetStartupWindow(uint16_t value)
{
    pss.windowStartup = value;
}

/**
    Setter for the number of pulses
    required for startup activation
 */
void PSS_SetStartupPulsesCount(uint32_t value)
{
    pss.minPulsesStartup = value; 
}

/**
    Setter for the pss running window
 */
void PSS_SetRunningWindow(uint16_t value)
{
    pss.windowRunning = value;
}

/**
    Setter for the number of pulses
    required for running activation
 */
void PSS_SetRunningPulsesCount(uint32_t value)
{
    pss.minPulsesRunning = value;
}

/**
    Setter to update the number of magnet
 */
void PSS_SetNumberOfMagnets(uint8_t value)
{
    pss.magnetsCount = value;
}

/**
    Update the pulse capture value coming from the ISR
*/
void PSS_UpdatePulseFromISR(uint32_t capture)
{
    PulseFrequency_IsrCallUpdate(pss.pPulseFrequency, capture);
}

/**
    Update the overflow coming from ISR
*/
void PSS_OverflowPulseFromISR()
{
    PulseFrequency_ISROverflowUpdate(pss.pPulseFrequency); 
}