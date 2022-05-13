/**
  ******************************************************************************
  * @file    mc_tuning.h
  * @author  FTEX inc
  * @brief   This file contains all definitions and functions prototypes for the
  *          motor control tuning component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MCTuning
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
#include "state_machine.h"


/**
  * @brief  MC tuning internal objects initialization structure type;
  */
typedef struct
{
  PID_Handle_t * pPIDSpeed;
  PID_Handle_t * pPIDIq;
  PID_Handle_t * pPIDId;
  PID_Handle_t * pPIDFluxWeakening;
  PWMC_Handle_t * pPWMnCurrFdbk;
  SpeednPosFdbk_Handle_t * pSpeedSensorMain;
  SpeednPosFdbk_Handle_t * pSpeedSensorAux;
  VirtualSpeedSensor_Handle_t * pSpeedSensorVirtual;
  SpeednTorqCtrl_Handle_t * pSpeednTorqueCtrl;
  STM_Handle_t * pStateMachine;
  NTC_Handle_t * pTemperatureSensor;
  BusVoltageSensor_Handle_t * pBusVoltageSensor;
  MotorPowMeas_Handle_t * pMPM;
  FW_Handle_t  * pFW;
  FF_Handle_t  * pFF;
} MCT_Handle_t;

#endif
