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


//===================== Defines =========================================== //

#define MAX_ALLOWED_PERIOD_SEC     120  //Period of measurement for RMP in sec, two minutes

// ==================== Public function prototypes ======================== //
/**
    Pedal Speed Sensor module Initialization
*/
void PedalSpdSensor_Init(PedalSpeedSensorHandle_t* pHandle)
{
    ASSERT(pHandle != NULL);
    PulseFrequency_GetTimerInfo(pHandle->pPulseFrequency);
}

/**
    Pedal Speed Sensor capture the number of pulses detected from the 
    cadence signal/sensor.
*/
void PedalSpdSensor_ReadNumberOfPulses(PedalSpeedSensorHandle_t* pHandle)
{    
    //PulseFrequency_ReadInputCapture (pHandle->pPulseFrequency); 
    pHandle->hPedalSpeedSens_NumberOfPulses = (uint16_t) pHandle->pPulseFrequency->hNumberOfPulse; 
    
    //Initialize the number of pulses detected by the AGT Timer.
    pHandle->pPulseFrequency->hNumberOfPulse = 0;
}

/**
    Pedal Speed Sensor Periode Get value
*/
uint16_t PedalSpdSensor_GetNumberOfPulses(PedalSpeedSensorHandle_t* pHandle)
{    
    return pHandle->hPedalSpeedSens_NumberOfPulses;
}

/**
    Reset all variables used to hold the number of pulses measured
    by the AGT timer.
*/
void PedalSpdSensor_ResetValue(PedalSpeedSensorHandle_t* pHandle)
{    
    pHandle->hPedalSpeedSens_NumberOfPulses = 0;
    //Initialize the number of pulses detected by the AGT Timer.
    pHandle->pPulseFrequency->hNumberOfPulse = 0;
}

/**
    Set windows reset flag 
*/
void PedalSpdSensor_SetWindowsFlag(PedalSpeedSensorHandle_t* pHandle)
{    
    pHandle->bPedalSpeedSens_ResetWindowsFlag = true;
}

/**
    Clear windows reset flag
*/
void PedalSpdSensor_ClearWindowsFlag(PedalSpeedSensorHandle_t* pHandle)
{    
    pHandle->bPedalSpeedSens_ResetWindowsFlag = false;
}

/**
    Get windows reset flag
*/
bool PedalSpdSensor_GetWindowsFlag(PedalSpeedSensorHandle_t* pHandle)
{    
    return pHandle->bPedalSpeedSens_ResetWindowsFlag;
}

/**
    Pedal Speed Sensor RPM Get value
*/
uint16_t PedalSpdSensor_GetSpeedRPM(PedalSpeedSensorHandle_t* pHandle)
{    
    return pHandle->wPedalSpeedSens_RPM;
}

/**
    Check if we have a new number of pulses detected from the previous number of pulses detected 
*/
bool PedalSpdSensor_NewPedalPulsesDetected(PedalSpeedSensorHandle_t* pHandle)
{
    ASSERT(pHandle != NULL);

    // Validate if we have new pulses detected, compared to the previous number of pulses
    if(pHandle->hPreviousNumberOfPulse != pHandle->pPulseFrequency->hNumberOfPulse)
    {
        // Update the previous number of pulse with the current one   
        pHandle->hPreviousNumberOfPulse = pHandle->pPulseFrequency->hNumberOfPulse;
        return true;
    }
    return false;   
}


/**
    Pedal Speed Sensor calculate periode value
*/
void PedalSpdSensor_CalculateRPM(PedalSpeedSensorHandle_t* pHandle)
{
    //check input conditions
    ASSERT(pHandle != NULL);
    //update basic wheel speed information.
    //Calculate the basic parameters of the Input PAS Sensor
    PulseFrequency_ReadInputCapture (pHandle->pPulseFrequency); 
    // Get the total period time. Since the timer is configured to measure the pulse width
    float period = pHandle->pPulseFrequency->wSecondPeriod;
    // Prevent division by zero.    
    static const float EPSILON = 1e-6f;
    static const uint16_t ONE_DECIMAL_PLACE = 10; 
    if (period > EPSILON && period < MAX_ALLOWED_PERIOD_SEC && pHandle->bNB_magnets > 0)
    {
        pHandle->wPedalSpeedSens_RPM = (uint16_t)(((float)(RPMCOEFF*ONE_DECIMAL_PLACE))/(period*pHandle->bNB_magnets));
    }
    else
    {
        pHandle->wPedalSpeedSens_RPM = 0; 
    }
}