/**
  ******************************************************************************
  * @file    pedal_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal speed sensor
  ******************************************************************************
*/

#include "pedal_speed_sensor.h"

// ==================== Private function prototypes ======================== //

/**
    Pedal Speed Sensor module Initialization
*/
void PSS_Init(PedalSpeedSens_Handle_t* pHandle)
{
    PedalTorqSensor_Init(pHandle->pTorque);
}

/**
	Pedal Speed Sensor capture Periode calculation
*/
void PSS_CalculateSpeed(PedalSpeedSens_Handle_t* pHandle)
{	
    PulseFrequency_ReadInputCapture_AGT (pHandle->pSpulse); 
    pHandle->wPedal_Sensor_Read = (uint32_t) pHandle->pSpulse->agt_capture_count; 
}

/**
	Pedal Speed Sensor Periode Get value
*/
uint32_t PSS_GetPeriodValue(PedalSpeedSens_Handle_t* pHandle)
{	
    return pHandle->wPedal_Sensor_Read;
}

/**
	Pedal Speed Sensor Frequency calculate value
*/
void PSS_CalculateSpeedFreq(PedalSpeedSens_Handle_t* pHandle)
{	
	pHandle->wPedalSpeedSens_Freq = COEFFREQ / PSS_GetPeriodValue(pHandle);
}

/**
	Pedal Speed Sensor Frequency Get value
*/
uint32_t PSS_GetSpeedFreq(PedalSpeedSens_Handle_t* pHandle)
{	
	return pHandle->wPedalSpeedSens_Freq;
}

/**
	Pedal Speed Sensor RPM Calculate value
*/
void PSS_CalculateSpeedRPM(PedalSpeedSens_Handle_t* pHandle)
{	
	//int8_t 	bdirec = Get_Drive_Direction (pHandle->pSpulse);
	pHandle->wPedalSpeedSens_RPM = (int32_t)((PSS_GetSpeedFreq(pHandle) / pHandle->bPulseNumb_pr)* RPMCOEFF);
}

/**
	Pedal Speed Sensor RPM Get value
*/
int32_t PSS_GetSpeedRPM(PedalSpeedSens_Handle_t* pHandle)
{	
	return pHandle->wPedalSpeedSens_RPM;
}

/**
    Pedal Speed Torque Sensor average value calculation
*/
void PSS_CalcTSAvValue(PedalSpeedSens_Handle_t* pHandle)
{
    PedalTorqSensor_CalcAvValue(pHandle->pTorque);
}


/**
    Pedal Speed Sensor Presence Flag 
*/
void PSS_UpdatePASDetection (PedalSpeedSens_Handle_t * pHandle) 
{
	uint32_t	wSpeedt;
	uint16_t	hTorqueSens;
	
	wSpeedt = PSS_GetPeriodValue(pHandle);
	hTorqueSens = PedalTorqSensor_GetAvValue(pHandle->pTorque);

	if ((pHandle->bTorqueSensorUsed) && (hTorqueSens > pHandle->pTorque->hParameters.hOffsetMT) )
		pHandle->bPSSDetected = true;
	else if (wSpeedt > 0)
		pHandle->bPSSDetected = true;
	else 
		pHandle->bPSSDetected = false;
} 

/**
    Pedal Speed Sensor Flag Detection
*/
bool PSS_IsPASDetected(PedalSpeedSens_Handle_t * pHandle) 
{
	return pHandle->bPSSDetected;
}
