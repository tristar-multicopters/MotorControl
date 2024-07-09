/**
  ******************************************************************************
  * @file    wheel_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for wheel speed sensor
  ******************************************************************************
*/

#include "wheel_speed_sensor.h"

// ==================== Private function prototypes ======================== //

/**
    Wheel Speed Sensor calculate periode value
*/
void WSS_CalculatePeriodValue(WheelSpeedSens_Handle_t* pHandle)
{
	/* Detect if the WSS is in slow mode read (more than 200ms)*/
	if (pHandle->bWSSslowDetect)
	{
		pHandle->bSlowDetectCount ++;
		if (pHandle->bSlowDetectCount > pHandle->bSlowDetectCountValue)
		{
			PulseFrequency_ReadInputCapture_GPT(pHandle->wSpulse);
			pHandle->bSlowDetectCount = 0;
		}
	}
	/* WSS calculate speed in normal mode*/
	else
    {
        PulseFrequency_ReadInputCapture_GPT(pHandle->wSpulse);
        /* Affectation to the wheel speed sensor read value from the gpt counter read*/
        pHandle->wWheel_Sensor_Read = (uint32_t) pHandle->wSpulse->gpt_capture_count; 
    }
}

/**
    Wheel Speed Sensor Get periode value in usec
*/
uint32_t WSS_GetPeriodValue(WheelSpeedSens_Handle_t* pHandle)
{
	return pHandle->wWheel_Sensor_Read;
}

/**
    Wheel Speed Sensor Calculate frequency in mHz
*/
void WSS_CalculateSpeedFreq(WheelSpeedSens_Handle_t* pHandle)
{	
	pHandle->frequency = WCOEFFREQ / WSS_GetPeriodValue(pHandle);
}

/**
    Wheel Speed Sensor retrun Frequency in mHz
*/
uint32_t WSS_GetSpeedFreq(WheelSpeedSens_Handle_t* pHandle)
{
	return pHandle->frequency;
}

/**
    Wheel Speed Sensor Calculate RPM in tr/min
*/
void WSS_CalculateSpeedRPM(WheelSpeedSens_Handle_t* pHandle)
{		
	pHandle->speedRPM = (((WSS_GetSpeedFreq(pHandle) / pHandle->bWSPulseNumb_pr)* WRPMCOEFF)/WPRECISIONCOEFF);
}

/**
    Wheel Speed Sensor retrun RPM in tr/min
*/
int32_t WSS_GetSpeedRPM(WheelSpeedSens_Handle_t* pHandle)
{
	return pHandle->speedRPM;
}

/**
    Wheel Speed Sensor Detection
*/
void WSS_UpdateWSSDetection (WheelSpeedSens_Handle_t * pHandle) 
{
	int32_t	wWheelSpeed;
	
	wWheelSpeed = WSS_GetSpeedRPM (pHandle);

	if (wWheelSpeed > 0)
    {
        pHandle->bWSSDetected = true;
	}
    else 
	{
        pHandle->bWSSDetected = false;
    }
}    

/**
    Wheel Speed Sensor Get Detection flag
*/
bool WSS_IsWSSDetected(WheelSpeedSens_Handle_t * pHandle) 
{
	return pHandle->bWSSDetected;
}
