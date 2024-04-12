/**
  ******************************************************************************
  * @file    pedal_torque_sensor.h
  * @author  FTEX Inc
  * @brief   This file defines the handles, constants and function prototypes 
  *          used in higher level modules for pedal torque sensor
  *
    ******************************************************************************
*/


#ifndef __PEDAL_TORQUE_SENSOR_H
#define __PEDAL_TORQUE_SENSOR_H

// ============================ Includes ================================= //
#include "regular_conversion_manager.h"
#include "signal_filtering.h"
#include "delay.h"

// ============================= Defines ================================= //

#define SAFE_TORQUE_COUNT_100MS           (uint16_t)20    /* Called every 5ms, 20*5ms = 100s
                                                          stabilize time for the torque sensor */

#define PTS_SLOPE_FACTOR   100   // Factor used to take a floatign point and make a fraction
                                 // If factor == 100 then 1.25f would make a 125/100 fraction 
                                 
#define PTS_PERCENTAGE    (uint8_t)100    /* Percentage for PTS use */
    
//hLowPassFilterBW1 and 2 array size.
#define BW_ARRAY_SIZE        3
//hFilterSpeed array size.
#define FILTERSPEED_ARRAY_SIZE       (BW_ARRAY_SIZE - 1)

// ======================= Public strutures ============================= //

typedef struct
{          
    float fFilterAlpha;     /* Alpha coefficient for low pass first order butterworth filter */
    float fFilterBeta;      /* Beta coefficient for low pass first order butterworth filter */
    
    uint16_t    hOffsetPTS;     /* Offset of the torque sensor signal when at lowest position */
    uint16_t    hMax;               /* torque signal when at maximum position */
    uint16_t    bSlopePTS;      /* Gain factor of ADC value vs torque sensor */
    uint16_t    bDivisorPTS;    /* Scaling factor of ADC value vs torque sensor */
    
    uint16_t    hOffsetMTStartup;          /* Offset of torque sensor vs torque on Startup*/
    uint16_t    hStartupOffsetMTSpeedRPM;  /* Speed under which the startup offset is used in rpm */
    uint16_t    hStartupOffsetMTSpeedKMH;  /* Speed under which the startup offset is used in Kmh */
    uint16_t    hOffsetMT;                 /* Offset of torque sensor vs torque */
    uint16_t    hOffsetMTSafety;           /* Offset of torque sensor vs torque used for safety check */ 
    
    int16_t     bSlopeMT;       /* Gain factor of torque sensor vs torque */
    int16_t     bDivisorMT;     /* Scaling factor of torque sensor vs torque */
    

    uint16_t    PasMaxOutputTorque; /* max motor torque that PAS is allowed to use */
    
    uint16_t    hLowPassFilterBW1[BW_ARRAY_SIZE];      /* used to configure the first coefficient software filter bandwidth */
    uint16_t    hLowPassFilterBW2[BW_ARRAY_SIZE];      /* used to configure the second coefficient software filter bandwidth */ 
    
    uint8_t     hFilterSpeed[FILTERSPEED_ARRAY_SIZE];   /* speed value used to decide what filter band will be used.*/
        
} PTS_Param_t;


typedef struct
{
    RegConv_t   PTSRegConv;
    
    bool        bSafeStart;         /* Stuck Pedal Torque Sensor check on start */
    
    uint8_t     bConvHandle;        /* handle to the regular conversion */

    uint16_t    hInstTorque;        /* It contains latest available instantaneous torque
                                        This parameter is expressed in u16 */
    uint16_t    hAvADCValue;        /* It contains latest available average ADC value */
    uint16_t    hAvTorqueValue;     /* It contains latest available average torque */
    
    SignalFilteringHandle_t TorqSensorFilter; /* Filter structure used to filter out noise */
    
    Delay_Handle_t * pPTSstuckDelay;    /* Pedal torque sensor stuck delay used during init */

    PTS_Param_t hParameters;
    
} PedalTorqSensorHandle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief  Pedal torque Sensor conversion Initialization
  @param  PedalTorqSensorHandle_t handle & Delay_Handle_t pPTSstuckDelay
  @return None
*/
void PedalTorqSensor_Init(PedalTorqSensorHandle_t * pHandle, Delay_Handle_t * pPTSstuckDelay, uint16_t maxTorque);
/**
  @brief  Pedal torque Sensor ADC hardware values clear
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void PedalTorqSensor_Clear(PedalTorqSensorHandle_t * pHandle);

/**
  @brief  Pedal torque Sensor ADC value calculation and filtering
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void PedalTorqSensor_CalcAvValue(PedalTorqSensorHandle_t * pHandle,uint8_t speed);

/**
  @brief  Pedal torque Sensor return ADC value
  @param  PedalTorqSensorHandle_t handle
  @return hAvTorqueValue in uin16_t format
*/
uint16_t PedalTorqSensor_GetAvValue(PedalTorqSensorHandle_t * pHandle);

/**
  @brief  Pedal torque Sensor Reset ADC value
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void   PedalTorqSensor_ResetAvValue(PedalTorqSensorHandle_t * pHandle);

/**
  @brief  Pedal torque Sensor  Convert Torque sensor data to motor torque
  @param  PedalTorqSensorHandle_t handle
  @return torque reference value in int16 format
*/
int16_t PedalTorqSensor_ToMotorTorque(PedalTorqSensorHandle_t * pHandle);

/**
     Return true if the Pedal Torque sensor is pressed (threshold is passed) 
  */
bool PedalTorqSensor_IsDetected (PedalTorqSensorHandle_t * pHandle);

/**
  @brief  Compute slope for torque sensor module
  @param  PedalTorqSensorHandle_t handle
  @return void
*/
void PedalTorqSensor_ComputeSlopes(PedalTorqSensorHandle_t * pHandle);

/**
  @brief  Select the index of the bw buffer based on the bike speed.
  @param  PedalTorqSensorHandle_t handle
  @param  uint8_t speed to be compared.
  @return void
*/
uint8_t PedalTorqSensor_GetBwUsingSpeed(PedalTorqSensorHandle_t * pHandle, uint8_t speed);

#endif
