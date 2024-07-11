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

#define PETAL_TORQUE_SENSOR_ERROR_OFFSET (float) 40 // Value added to the idleSensorOffset (in %) before we trigger a torque sensor issue 

// ======================= Public strutures ============================= //

typedef struct
{          
    float filterAlpha;                              /* Alpha coefficient for low pass first order butterworth filter */
    float filterBeta;                               /* Beta coefficient for low pass first order butterworth filter */
    uint16_t idleSensorOffset;                      /* Offset of the torque sensor signal when at lowest position */
    uint16_t maxTorqueValue;                        /* torque signal when at maximum position */
    uint16_t sensorSlope;                           /* Gain factor of ADC value vs torque sensor */
    uint16_t sensorADCScale;                        /* Scaling factor of ADC value vs torque sensor */
    uint16_t offsetMTStartup;                       /* Offset of torque sensor vs torque on Startup*/
    uint16_t startupOffsetSpeedRPM;                 /* Speed under which the startup offset is used in rpm */
    uint16_t startupOffsetSpeedKMH;                 /* Speed under which the startup offset is used in Kmh */
    uint16_t offsetMT;                              /* Offset of torque sensor vs torque */
    uint16_t offsetMTSafety;                        /* Offset of torque sensor vs torque used for safety check */ 
    int16_t  slopeMT;                               /* Gain factor of torque sensor vs torque */
    int16_t  divisorMT;                             /* Scaling factor of torque sensor vs torque */
    uint16_t PasMaxOutputTorque;                    /* max motor torque that PAS is allowed to use */
    uint16_t lowPassFilterBW1[BW_ARRAY_SIZE];       /* used to configure the first coefficient software filter bandwidth */
    uint16_t lowPassFilterBW2[BW_ARRAY_SIZE];       /* used to configure the second coefficient software filter bandwidth */     
    uint8_t  filterSpeed[FILTERSPEED_ARRAY_SIZE];   /* speed value used to decide what filter band will be used.*/
        
} PedalTorqueSensorParameters_t;


typedef struct
{
    RegConv_t   PTSRegConv; 
    bool        safeStart;                      /* Stuck Pedal Torque Sensor check on start */
    uint8_t     convHandle;                     /* handle to the regular conversion */
    uint16_t    instTorque;                     /* It contains latest available instantaneous torque*/
    uint16_t    avgADCValue;                    /* It contains latest available average ADC value */
    uint16_t    avgTorqueValue;                 /* It contains latest available average torque */
    SignalFilteringHandle_t TorqSensorFilter;   /* Filter structure used to filter out noise */
    Delay_Handle_t * pPTSstuckDelay;            /* Pedal torque sensor stuck delay used during init */
    PedalTorqueSensorParameters_t parameters;
} PedalTorqueSensorHandle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief  Pedal torque Sensor conversion Initialization
  @param  PedalTorqSensorHandle_t handle & Delay_Handle_t pPTSstuckDelay
  @return None
*/
void PedalTorqueSensor_Init(Delay_Handle_t * pPTSstuckDelay, uint16_t maxTorque);
/**
  @brief  Pedal torque Sensor ADC hardware values clear
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void PedalTorqueSensor_Clear(void);

/**
  @brief  Pedal torque Sensor ADC value calculation and filtering
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void PedalTorqueSensor_CalcAvValue(uint8_t speed);

/**
  @brief  Pedal torque Sensor return ADC value
  @param  PedalTorqSensorHandle_t handle
  @return hAvTorqueValue in uin16_t format
*/
uint16_t PedalTorqueSensor_GetAvValue(void);

/**
  @brief  Pedal torque Sensor Value in percentage
  @param  PedalTorqSensorHandle_t handle
  @return Torque value in percentage in uin8_t format
*/
uint8_t PedalTorqueSensor_GetPercentTorqueValue(void);

/**
  @brief  Pedal torque Sensor Reset ADC value
  @param  PedalTorqSensorHandle_t handle
  @return None
*/
void   PedalTorqueSensor_ResetAvValue(void);

/**
  @brief  Pedal torque Sensor  Convert Torque sensor data to motor torque
  @param  PedalTorqSensorHandle_t handle
  @return torque reference value in int16 format
*/
int16_t PedalTorqueSensor_ToMotorTorque(void);

/**
     Return true if the Pedal Torque sensor is pressed (threshold is passed) 
  */
bool PedalTorqueSensor_IsDetected(void);

/**
  @brief  Compute slope for torque sensor module
  @param  PedalTorqSensorHandle_t handle
  @return void
*/
void PedalTorqueSensor_ComputeSlopes(void);

/**
  @brief  Select the index of the bw buffer based on the bike speed.
  @param  PedalTorqSensorHandle_t handle
  @param  uint8_t speed to be compared.
  @return void
*/
uint8_t PedalTorqueSensor_GetBwUsingSpeed(uint8_t speed);

/**
  @brief  Getter for Startup offset Speed RPM
  @return Value Startup offset Speed RPM
*/
uint16_t PedalTorqueSensor_GetStartupOffsetMTSpeedRPM(void);

/**
  @brief  Getter for Startup Offset
  @return Value Startup Offset
*/
uint16_t PedalTorqueSensor_GetOffsetMTStartup(void);

/**
  @brief Getter for Running Offset
  @return Value Running Offset
*/
uint16_t PedalTorqueSensor_GetOffsetMT(void);

/**
  @brief Getter for Idle Sensor Offset
  @return Value Idle Sensor Offset
*/
uint16_t PedalTorqueSensor_GetSensorOffset(void);

/**
  @brief  Getter for Max Torque Value
  @return Value Startup offset Speed RPM
*/
uint16_t PedalTorqueSensor_GetMaxTorqueValue(void);

/**
  @brief  Setter for Startup Offset Speed KM/H
  @param value : New value to set
  @return None
*/
void PedalTorqueSensor_SetStartupOffsetMTSpeedKMH(uint16_t value);

/**
  @brief  Setter for Startup MT Offset 
  @param value : New value to set
  @return None
*/
void PedalTorqueSensor_SetOffsetMTStartup(uint16_t value);

/**
  @brief  Setter for Running MT Offset 
  @param value : New value to set
  @return None
*/
void PedalTorqueSensor_SetOffsetMT(uint16_t value);

/**
  @brief  Setter for Idle Sensor Offset
  @param value : New value to set
  @return None
*/
void PedalTorqueSensor_SetSensorOffset(uint16_t value);

/**
  @brief  Setter for Max Torque Value 
  @param value : New value to set
  @return None
*/
void PedalTorqueSensor_SetMaxTorqueValue(uint16_t value);

/**
  @brief  Setter for Speed Filter 
  @param value : New value to set
  @param index : Index in the speed filter buffer
                 where to apply the new value
  @return None
*/
void PedalTorqueSensor_SetFilterSpeed(uint8_t value, uint8_t index);

/**
  @brief  Setter for First Butterworth Filter  
  @param value : New value to set
  @param index : Index in the butterworth filter buffer
                 where to apply the new value
  @return None
*/
void PedalTorqueSensor_SetBWFilter1(uint16_t value, uint8_t index);

/**
  @brief  Setter for Second Butterworth Filter  
  @param value : New value to set
  @param index : Index in the butterworth filter buffer
                 where to apply the new value
  @return None
*/
void PedalTorqueSensor_SetBWFilter2(uint16_t value, uint8_t index);

#endif