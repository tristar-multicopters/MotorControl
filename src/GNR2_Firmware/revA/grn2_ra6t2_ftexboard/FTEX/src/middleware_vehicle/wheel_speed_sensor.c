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
}

/**
    Wheel Speed Sensor calculate periode value
*/
void WheelSpdSensor_CalculatePeriodValue(WheelSpeedSensorHandle_t* pHandle)
{
	/* Detect if the WSS is in slow mode read (more than 200ms)*/
	if (pHandle->bSpeedslowDetect)
	{
		pHandle->bSlowDetectCount ++;
		if (pHandle->bSlowDetectCount > pHandle->bSlowDetectCountValue)
		{
			PulseFrequency_ReadInputCapture (pHandle->pPulseFrequency); 
			pHandle->bSlowDetectCount = 0;
		}
	}
	/* WSS calculate speed in normal mode*/
	else
    {
        PulseFrequency_ReadInputCapture (pHandle->pPulseFrequency); 
        /* Affectation to the wheel speed sensor read value from the gpt counter read*/
        pHandle->wWheelSpeed_Read = (uint32_t) pHandle->pPulseFrequency->wUsPeriod; 
    }
}

/**
    Wheel Speed Sensor Get periode value in usec
*/
uint32_t WheelSpdSensor_GetPeriodValue(WheelSpeedSensorHandle_t* pHandle)
{
	return pHandle->wWheelSpeed_Read;
}

/**
    Wheel Speed Sensor retrun Frequency in mHz
*/
uint32_t WheelSpdSensor_GetSpeedFreq(WheelSpeedSensorHandle_t* pHandle)
{
	pHandle->wWheelSpeedFreq = COEFFREQ / WheelSpdSensor_GetPeriodValue(pHandle);
	return pHandle->wWheelSpeedFreq;
}

/**
    Wheel Speed Sensor retrun RPM in tr/min
*/
int32_t WheelSpdSensor_GetSpeedRPM(WheelSpeedSensorHandle_t* pHandle)
{
	pHandle->wWheelSpeedRpm = (((WheelSpdSensor_GetSpeedFreq(pHandle) / pHandle->bPulsePerRotation)* RPMCOEFF)/PRECISIONCOEFF);
	return pHandle->wWheelSpeedRpm;
}

/**
    Wheel Speed Sensor Detection
*/
void WheelSpdSensor_UpdateWSSDetection (WheelSpeedSensorHandle_t * pHandle) 
{
	int32_t	wWheelSpeed;
	
	wWheelSpeed = WheelSpdSensor_GetSpeedRPM (pHandle);

	if (wWheelSpeed > 0)
	{
		pHandle->bSpeedDetected = true;
	}
	else 
	{
		pHandle->bSpeedDetected = false;
	}
}    

/**
    Wheel Speed Sensor Get Detection flag
*/
bool WheelSpdSensor_IsSpeedDetected(WheelSpeedSensorHandle_t * pHandle) 
{
	return pHandle->bSpeedDetected;
}
