/**
  ******************************************************************************
  * @file    pedal_speed_sensor.h
  * @author  FTEX inc
  * @brief   This file defines the handles, constantas and function prototypes
  *           used in higher level modules for pedal speed sensor
  *
    ******************************************************************************
*/

#ifndef __PEDAL_SPEED_SENSOR_H
#define __PEDAL_SPEED_SENSOR_H

// =============================== Includes ================================== //
#include "pulse_frequency.h"
//#include "pedal_torque_sensor.h"

// ================= Structure used to configure a pin ===================== //
typedef struct {
    
    PulseFrequencyHandle_t * pPulseFrequency;   /* Pointer to pedal handle */
    uint16_t hPedalSpeedSens_MinPulseStartup;   //It has the minimum number of pulses, on startup, 
                                                //to detect PAS from cadence sensor
    uint32_t wPedalSpeedSens_WindowsStartup;    //Maximum time, on ms, to verify the Detected Number of pulses
                                                //from the cadence sensor when starts to pedal.
    uint16_t hPedalSpeedSens_MinPulseRunning;   //It has the minimum number of pulses, on running, 
                                                //to detect PAS from cadence sensor
    uint32_t wPedalSpeedSens_WindowsRunning;    //Maximum time, on ms, to verify the Detected Number of pulses
                                                //from the cadence sensor when on run mode.
    bool bPedalSpeedSens_ResetWindowsFlag;      //Used to 
    uint16_t hPedalSpeedSens_NumberOfPulses;    /*Detected Number of pulses from teh cadence signal*/
    uint32_t  wPedalSpeedSens_RPM;   /* Pedal Speed sensor RPM calculated value */
    
} PedalSpeedSensorHandle_t;

// ==================== Public function prototypes ========================= //
/**
  @brief  Function to initialize pedal speed sensor handle
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PedalSpdSensor_Init(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to capture pedal speed sensor Periode
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PedalSpdSensor_ReadNumberOfPulses(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to Get the Pedal Speed Sensor Periode
  @param  PedalSpeedSensorHandle_t handle
  @return wPedal_Sensor_Read in unit32_t
*/
uint16_t PedalSpdSensor_GetNumberOfPulses(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to reset all variables used to hold the number of pulses measured
          by the AGT timer.
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PedalSpdSensor_ResetValue(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Set windows reset flag
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PedalSpdSensor_SetWindowsFlag(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Clear windows reset flag
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PedalSpdSensor_ClearWindowsFlag(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Get windows reset flag
  @param  PedalSpeedSensorHandle_t handle
  @return bool bPedalSpeedSens_ResetWindowsFlag
*/
bool PedalSpdSensor_GetWindowsFlag(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to return speed in rpm
  @param  PedalSpeedSensorHandle_t handle
  @retval Speed in rpm
*/
uint32_t PedalSpdSensor_GetSpeedRPM(PedalSpeedSensorHandle_t* pHandle);

#endif
