/**
  * @file    motor_parameters.h
  * @brief   This file contains the structures containing the motor parameters.
*/

// Define to prevent recursive inclusion
#ifndef __MOTOR_PARAMETERS_H
#define __MOTOR_PARAMETERS_H

#include "stdint.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "current_pid_vs_speed_table.h"


// Motor Config Parameters
typedef struct 
{
    float fMotorGearRatio;
    MotorType_t bMotorType;
    uint8_t bPolePairNum;
    float fRS;
    float fLS;
    float fMotorMagnetFlux;
    float fMotorVotlageConstant;
    float fSTTorqueCoef;
    
    uint16_t hPeakCurrentMotorAmps;
    
    
} ConfigParameters_t;

//Motor parameters that limit power
typedef struct
{   
    uint16_t hEstimatedEfficiency;
    
} PowerParameters_t;

//Motor speed parameters
typedef struct
{
    uint16_t hMaxAppliationSpeedRPM;
    
    bool bEnableSpeedLimitControl;
    int16_t hPIDSpeedKpDefault;
    int16_t hPIDSpeedKiDefault;
    uint16_t hSpKiDiv;
    uint16_t hSpKiDivLog;
    
    uint16_t hFoldbackSpeedInterval;
    
} SpeedParameters_t;

//Motor torque parameters
typedef struct
{
    int16_t hPIDTorqueKpDefault;
} TorqueParameters_t;

typedef struct
{
    uint8_t bFluxWeakeningEnable;
    
    int16_t hPIDFluxKPDefault;
    int16_t hPIDFluxKIDefault;
} FluxParameters_t;

//Motor parameters for the ramp
typedef struct
{
    uint32_t wDefaultTorqueSlopeUp;
    uint32_t wDefaultTorqueSlopeDown;
    uint32_t wDefaultSpeedSlopeUp;
    uint32_t wDefaultSpeedSlopeDown;
    
    float fMecSpeedFilterButterworthAlpha;
    float fMecSpeedFilterButterworthBeta;
    
} RampManagerParameters_t;

//Motor temperature parameters
typedef struct
{
    SensorType_t bMotorTempSensorType;
    bool bMotorTempMixed;
    
    int16_t hOverTempMotorThresholdC;
    int16_t hOverTempMotorHysteresisC;
    uint16_t hFoldbackMotorTempInterval;
    
} TempParameters_t;

//Motor hall sensor parameters
typedef struct
{
    uint8_t bHallSensorsPlacement;
    uint8_t bHallAveragingFifoDepth;
    int16_t hHallPhaseShift;
    
    uint8_t bEnVibrationError;
    
} HallSensorParameters_t;

//Wheel speed sensor parameters
typedef struct
{
    uint8_t bWheelSpeedSensorNbrPerRotation;
    float   fMotorWSSTimeOnOneMagnetPercent;
      
} WheelSpeedSensorParameters_t;

typedef struct
{
    int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE];
    int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE];
    int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE];
    int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE];

} CurrentSpeedPID_t;

//Motor Specific Parameters
typedef struct 
{
    ConfigParameters_t ConfigParameters;
    PowerParameters_t PowerParameters;
    SpeedParameters_t SpeedParameters;
    TorqueParameters_t TorqueParameters;
    FluxParameters_t FluxParameters;
    RampManagerParameters_t RampManagerParameters;
    TempParameters_t TempParameters;
    HallSensorParameters_t HallSensorParameters;
    WheelSpeedSensorParameters_t WheelSpeedSensorParameters;
    CurrentSpeedPID_t CurrentSpeedPID;
    ParametersConversion_t ParametersConversion;
    
    bool bAutotuneEnable;
    
} MotorParameters_t;


/*
 * Init parameters that are dependent on motor parameters
 */
void MotorParameters_Init(MotorParameters_t * MotorParameters);

#endif
