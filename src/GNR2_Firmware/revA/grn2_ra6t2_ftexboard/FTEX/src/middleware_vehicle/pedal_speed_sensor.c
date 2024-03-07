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

// ==================== Public function prototypes ======================== //
/**
    Pedal Speed Sensor module Initialization
*/
void PedalSpdSensor_Init(PedalSpeedSensorHandle_t* pHandle)
{
    ASSERT(pHandle != NULL);    
}

/**
    Pedal Speed Sensor capture the number of pulses detected from the 
    cadence signal/sesnor.
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
uint32_t PedalSpdSensor_GetSpeedRPM(PedalSpeedSensorHandle_t* pHandle)
{    
    return pHandle->wPedalSpeedSens_RPM;
}

