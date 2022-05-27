/**
  ******************************************************************************
  * @file    mc_config.h
  * @author  FTEX inc
  * @brief   Motor Control Subsystem components configuration and handler
  *          structures declarations.
  ******************************************************************************
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
#include "ramp_ext_mngr.h"
#include "circle_limitation.h"

#include "sto_speed_pos_fdbk.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "ao_spd_pos_fdbk.h"


extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
extern NTC_Handle_t TempSensorParamsM1;
extern PID_Handle_t PIDFluxWeakeningHandle_M1;
extern FW_Handle_t FW_M1;
extern PWMC_ICS_Handle_t PWM_Handle_M1;
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1;
extern STO_PLL_Handle_t STO_PLL_M1;
extern HALL_Handle_t HALL_M1;
extern RDivider_Handle_t RealBusVoltageSensorParamsM1;
extern CircleLimitation_Handle_t CircleLimitationM1;
extern FF_Handle_t FF_M1;

extern AO_Handle_t AngleObserverM1;

#define NBR_OF_MOTORS 1
#endif /* __MC_CONFIG_H */

