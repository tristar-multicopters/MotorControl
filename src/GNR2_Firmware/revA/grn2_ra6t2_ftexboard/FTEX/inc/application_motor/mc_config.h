/**
  * @file    mc_config.h
  * @brief   Motor Control Subsystem components configuration and handler
  *          structures declarations.
*/

#ifndef __MC_CONFIG_H
#define __MC_CONFIG_H

#include "pid_regulator.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "ntc_temperature_sensor.h"
#include "pwm_curr_fdbk.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "feed_forward_ctrl.h"
#include "flux_weakening_ctrl.h"
#include "pqd_motor_power_measurement.h"

#include "ics_ra6t2_pwm_curr_fdbk.h"

#include "hall_speed_pos_fdbk.h"
#include "circle_limitation.h"

#include "sto_speed_pos_fdbk.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "rotor_pos_obs.h"

#define NBR_OF_MOTORS 1

extern PIDHandle_t PIDSpeedHandleM1;
extern PIDHandle_t PIDIqHandleM1;
extern PIDHandle_t PIDIdHandleM1;
extern NTCTempSensorHandle_t TempSensorParamsM1;
extern PIDHandle_t PIDFluxWeakeningHandleM1;
extern FluxWeakeningHandle_t FluxWeakeningM1;
extern PWMInsulCurrSensorFdbkHandle_t PWMInsulCurrSensorFdbkHandleM1;
extern SpdTorqCtrlHandle_t SpeednTorqCtrlM1;
extern MotorPowerQDHandle_t PQDMotorPowMeasM1;
extern MotorPowerQDHandle_t *pPQD_MotorPowMeasM1;
extern BemfObserverPllHandle_t BemfObserverPllM1;
extern HallPosSensorHandle_t HallPosSensorM1;
extern ResDivVbusSensorHandle_t RealBusVoltageSensorParamsM1;
extern CircleLimitationHandle_t CircleLimitationM1;
extern FeedforwardHandle_t FeedforwardM1;

extern RotorPositionObserverHandle_t RotorPosObsM1;


#endif /* __MC_CONFIG_H */
