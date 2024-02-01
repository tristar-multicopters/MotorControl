/**
  ******************************************************************************
  * @file    wheel_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for wheel speed sensor
  ******************************************************************************
*/

#include "wheel_speed_sensor.h"
#include "ASSERT_FTEX.h"
// ==================== Public function prototypes ======================== //

/**
    Wheel Speed Sensor Initialization
*/
void WheelSpdSensor_Init(WheelSpeedSensorHandle_t* pHandle)
{
    ASSERT(pHandle != NULL);
    
    //get GPT timer information
    PulseFrequency_GetTimerInfo(pHandle->pPulseFrequency);
}

/**
    Wheel Speed Sensor calculate periode value
*/
void WheelSpdSensor_CalculatePeriodValue(WheelSpeedSensorHandle_t* pHandle)
{
    //verify if the timer is measuring.
    //if yes, initialise the flag to wait for the timer
    //to set the flag again and indicating it still working.
    if (pHandle->pPulseFrequency->measuring == true)
    {
        pHandle->pPulseFrequency->measuring = false;
        pHandle->wWheelSpeedTimeOut = 0;
    }
    else
    {
        //avoid variable overflow
        if (pHandle->wWheelSpeedTimeOut < WHEELSPEED_TIMEOUT_MS)
        {   
            //increment the timeout variable
            pHandle->wWheelSpeedTimeOut = pHandle->wWheelSpeedTimeOut + WHEELSPEED_TIME_INCREMENT_MS;
        }
    }
    
	// Detect if the timer did more than 2 overflows or timer is not running anymore, speed must to be set to zero.
	if ((pHandle->pPulseFrequency->wCaptureOverflow > MAXNUMBER_OVERFLOW_WHEELSPEED) || ((pHandle->pPulseFrequency->measuring == false) && (pHandle->wWheelSpeedTimeOut >= WHEELSPEED_TIMEOUT_MS)))
	{
        //Wheel speed is zero becacuse the timer overflowed
        //the time period determine the maximum time to the
        //wheel complete a full revolution.
        //this means that: 
        pHandle->wWheelSpeed_Read = 0;  
	}
    else
    {
        //update basic wheel speed information.
        PulseFrequency_ReadInputCapture (pHandle->pPulseFrequency); 
        //get the time used by the wheel to complet a revolution.
        pHandle->wWheelSpeed_Read = pHandle->pPulseFrequency->wUsPeriod;   
    }
}

/**
    Wheel Speed Sensor Get periode value in usec
*/
float WheelSpdSensor_GetPeriodValue(WheelSpeedSensorHandle_t* pHandle)
{
	return pHandle->wWheelSpeed_Read;
}

/**
    Wheel Speed Sensor retrun RPM in tr/min
*/
uint16_t WheelSpdSensor_GetSpeedRPM(WheelSpeedSensorHandle_t* pHandle)
{
    float wSpeed = WheelSpdSensor_GetPeriodValue(pHandle);
    
    pHandle->wWheelSpeedRpm = (int32_t)(((float)(RPMCOEFF))/(wSpeed*pHandle->bPulsePerRotation));

	return (uint16_t)round(pHandle->wWheelSpeedRpm);
}

