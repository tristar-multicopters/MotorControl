/**
  ******************************************************************************
  * @file    pedal_toque_sensor.h
  * @author  FTEX Inc
  * @brief   This file defines the handles, constants and function prototypes 
  *          used in higher level modules for pedal torque sensor
  *
	******************************************************************************
*/


#ifndef __PEDAL_TORQUE_SENSOR_H
#define __PEDAL_TORQUE_SENSOR_H


#include "regular_conversion_manager.h"

// ================= Structure used to configure a pin ===================== //

/* Torque_Param_t structure used to store torque user parameters */
typedef struct
{          
    uint16_t    hLowPassFilterBW1;   /* used to configure the first order software filter bandwidth */
    uint16_t    hLowPassFilterBW2;	

    uint16_t    hTorqueSensMax;   /* torque signal when at maximum position */
    
    uint16_t    hOffsetPTS;  /* Offset of the torque sensor signal when at lowest position */
    uint8_t     bSlopePTS;   /* Gain factor of ADC value vs torque sensor */
    uint8_t     bDivisorPTS; /* Scaling factor of ADC value vs torque sensor */
    
    uint16_t    hOffsetMT;  /* Offset of torque sensor vs torque */
    int8_t      bSlopeMT;   /* Gain factor of torque sensor vs torque */
    uint8_t     bDivisorMT; /* Scaling factor of torque sensor vs torque */
	
} PTS_Param_t;


typedef struct
{
    RegConv_t   PTS_RegConv;
    uint8_t     bConvHandle;            /* handle to the regular conversion */

    uint16_t    hInstTorque;            /* It contains latest available insteateonous torque
										   This parameter is expressed in u16 */
    uint16_t    hAvADCValue;            /* It contains latest available average ADC value */
    uint16_t    hAvTorqueValue;         /* It contains latest available average torque */

    PTS_Param_t hParam;
    
} PedalTorqSensorHandle_t;
// ==================== Public function prototypes ========================= //

/**
  @brief  Pedal torque Sensor conversion Initialization
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void PedalTorqSensor_Init(PedalTorqSensorHandle_t * pHandle);
/**
  @brief  Pedal torque Sensor ADC hardware values clear
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void PedalTorqSensor_Clear( PedalTorqSensorHandle_t * pHandle );

/**
  @brief  Pedal torque Sensor ADC value calculation and filtering
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void PedalTorqSensor_CalcAvValue( PedalTorqSensorHandle_t * pHandle );

/**
  @brief  Pedal torque Sensor return ADC value
  @param  PedalTorqSensorHandle_t handle
  @return hAvTorqueValue in uin16_t format
*/
uint16_t PedalTorqSensor_GetAvValue( PedalTorqSensorHandle_t * pHandle );

/**
  @brief  Pedal torque Sensor  Convert Torque sensor data to motor torque
  @param  PedalTorqSensorHandle_t handle
  @return torque reference value in int16 format
*/
int16_t PedalTorqSensor_ToMotorTorque(PedalTorqSensorHandle_t * pHandle);
#endif
