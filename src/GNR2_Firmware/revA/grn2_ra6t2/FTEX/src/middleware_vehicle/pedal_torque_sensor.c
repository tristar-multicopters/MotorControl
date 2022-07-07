/**
  ******************************************************************************
  * @file    pedal_torque_sensor.c
  * @author  FTEX Inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal assist
  ******************************************************************************
*/

#include "pedal_torque_sensor.h"

// ==================== Public function prototypes ======================== //


/**
	Pedal torque Sensor conversion Initialization
*/
void PedalTorqSensor_Init(PedalTorqSensorHandle_t * pHandle)
{	
    pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->PTS_RegConv);
    PedalTorqSensor_Clear(pHandle);
}


/**
	Pedal torque Sensor ADC hardware values clear
*/
void PedalTorqSensor_Clear( PedalTorqSensorHandle_t * pHandle )
{
    pHandle->hAvTorqueValue = 0u;
    pHandle->hAvADCValue = 0u;
}

/**
	Pedal torque Sensor ADC value calculation and filtering
*/
void PedalTorqSensor_CalcAvValue( PedalTorqSensorHandle_t * pHandle )
{
    uint32_t tTorqaux;
    uint16_t hTorqux;
    uint16_t htorque;
	
    uint16_t hBandwidth;
    /* Use the Read conversion Manager for ADC read*/
    htorque = RegConvMng_ReadConv(pHandle->bConvHandle);
    pHandle->hInstTorque = htorque;
	/* Verify the filter coefficient to apply*/
	if (pHandle->hInstTorque > pHandle->hAvADCValue)
		hBandwidth = pHandle->hParameters.hLowPassFilterBW1;
	else
		hBandwidth = pHandle->hParameters.hLowPassFilterBW2;
    /* Filter pedal torque sensor ADC data */
	if (htorque != 0xFFFFu)
	{
		tTorqaux = (uint32_t)(hBandwidth - 1u);
		tTorqaux *= (uint32_t)(pHandle->hAvADCValue);
		tTorqaux += htorque;
		tTorqaux /= (uint32_t)(hBandwidth);

		pHandle->hAvADCValue = (uint16_t)tTorqaux;
	}

	/* Compute torque sesnor value (between 0 and 65535) */
	hTorqux = (pHandle->hAvADCValue > pHandle->hParameters.hOffsetPTS) ? 
					(pHandle->hAvADCValue - pHandle->hParameters.hOffsetPTS) : 0; //Substraction without overflow
	
	tTorqaux = (uint32_t)(pHandle->hParameters.bSlopePTS * hTorqux);
	tTorqaux /= pHandle->hParameters.bDivisorPTS;
	if (tTorqaux > pHandle->hParameters.hTorqueSensMax) 
    {	
        tTorqaux = pHandle->hParameters.hTorqueSensMax;
    }
	hTorqux = (uint16_t)tTorqaux;
	
	pHandle->hAvTorqueValue = hTorqux;	
}


/**
	Pedal torque Sensor return ADC value
*/
uint16_t PedalTorqSensor_GetAvValue( PedalTorqSensorHandle_t * pHandle )
{
  return pHandle->hAvTorqueValue;
}

/**
	Pedal torque Sensor  Convert Torque sensor data to motor torque
*/
int16_t PedalTorqSensor_ToMotorTorque(PedalTorqSensorHandle_t * pHandle)
{
	int32_t tAux;
	
	/* Compute torque value (between -32768 and 32767) */
	tAux = (int32_t)(pHandle->hAvTorqueValue - pHandle->hParameters.hOffsetMT);
	if (tAux < 0)
    {
		tAux = 0;
	}
        /* Use slope factor to translate the torque speed sensor to a motor torque */
	tAux = ((pHandle->hParameters.bSlopeMT)*tAux);
	tAux /= pHandle->hParameters.bDivisorMT;
	
	/* Data limitation secure if there is any exceed */
	if (tAux > INT16_MAX)
    {
		tAux = INT16_MAX;
    }
    else if (tAux < INT16_MIN)
	{
        tAux = INT16_MIN;
	}
	return (int16_t)tAux;
}
