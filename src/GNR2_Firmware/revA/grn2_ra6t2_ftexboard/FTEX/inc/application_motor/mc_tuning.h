/**
  * @file    mc_tuning.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          motor control tuning component of the Motor Control application.
*/



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_TUNING_H
#define __MC_TUNING_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "virtual_speed_sensor.h"
#include "speed_torq_ctrl.h"
#include "motor_power_measurement.h"
#include "pqd_motor_power_measurement.h"
#include "ntc_temperature_sensor.h"
#include "bus_voltage_sensor.h"
#include "feed_forward_ctrl.h"
#include "flux_weakening_ctrl.h"


/**
  * @brief  MC tuning internal objects initialization structure type;
  */
typedef struct
{
  PIDHandle_t * pPIDSpeed;
  PIDHandle_t * pPIDIq;
  PIDHandle_t * pPIDId;
  PIDHandle_t * pPIDMotorControl;
  PWMCurrFdbkHandle_t * pPWMnCurrFdbk;
  SpdPosFdbkHandle_t * pSpeedSensorMain;
  SpdPosFdbkHandle_t * pSpeedSensorAux;
  VirtualSpeedSensor_Handle_t * pSpeedSensorVirtual;
  SpdTorqCtrlHandle_t * pSpeednTorqueCtrl;
  NTCTempSensorHandle_t * pTemperatureSensorController;
  NTCTempSensorHandle_t * pTemperatureSensorMotor;
  BusVoltageSensorHandle_t * pBusVoltageSensor;
  MotorPowerMeasHandle_t * pMotorPower;
  MCConfigHandle_t  * pFieldWeakening;
  FeedforwardHandle_t  * pFeedforward;
} MotorControlTuningHandle_t;

#endif
