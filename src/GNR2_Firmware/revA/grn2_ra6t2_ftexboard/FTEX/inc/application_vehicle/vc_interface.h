/**
  * @file    vc_interface.h
  * @brief   This module offers an interface to interact with vehicle properties
    */

#ifndef __VC_INTERFACE_H
#define __VC_INTERFACE_H

#include "stdlib.h"
#include "powertrain_management.h"
#include "vc_state_machine.h"

typedef struct {
    PWRT_Handle_t * pPowertrain;
    VCSTM_Handle_t * pStateMachine;
    CO_OBJ_DOM * pFirmwareUpdateDomainObj;
} VCI_Handle_t;

// Register tables
typedef enum
{
    /* Vehicle parameters (0-255)     */
    REG_PWRTRAIN_TYPE = 0,              /*< Powertrain type (hub, middrive, etc)                              */
    REG_DUALMOTOR,                      /*< Register for determine if the controller is a dual or a single    */
    REG_MAINMOTOR,                      /*< Parameter that saves the motor selected in a dualmotor controller */
    REG_MOTORSELECT_ENABLE,             /*< Boolean for knowing if there's a motor selector in the vehicle    */
    REG_POWERLOCK_ENABLE,               /*< Boolean for knowing if a powerlock is being used in the vehicle   */
    REG_FLDB_START_TEMP,                /*< TO VERIFY */
    REG_FLDB_END_TEMP,                  /*< TO VERIFY */
    REG_REGEN_SPEED_THR,
    REG_SPEED_LIMIT_RAMP,               /*< ?? */
    REG_CTRL_MODE,                      /*< Register for setting the mode control, if speed or torque         */
    REG_TORQUESENSOR_FAULTTIME,
    REG_TORQUESENSOR_VOLTAGEFAULT_THR,  /*< ?? */
    REG_TORQUESENSOR_FILTER,
    REG_MAXSPEED_THROTTLE,              /*< ?? */
    REG_MAXSPEED_PAS,
    REG_FLDB_END_VOLTAGE,
    REG_FLDB_START_VOLTAGE,
    REG_FLDB_END_SOC,
    REG_FLDB_START_SOC,

    /* Peripheral parameters (256-511)*/
    // Throttle parameters
    REG_THROTTLE_SLOPE = 256,  /*< Gain factor of ADC value vs throttle           */
    REG_THROTTLE_DIVISOR,      /*< Scaling factor of ADC value vs throttle        */
    REG_THROTTLE_OFFSET,       /*< Offset of ADC value vs throttle                */
    REG_START_THROTTLE,        /*< Minimum throttle value for starting the motors */
    REG_STOP_THROTTLE,         /*< Minimum throttle value for stoping the motors  */
    REG_THROTTLE_BW1,          /*< Used to configure the first order software filter bandwidth     */
    REG_THROTTLE_BW2,          /*< Used to configure the second order software filter bandwidth    */
    REG_THROTTLE_DEADBAND_THR,
    REG_THROTTLE_FAULTRANGE,
    // Torque parameters
    REG_TORQUE_SLOPE,          /*< Gain factor of throttle vs torque                               */
    REG_TORQUE_DIVISOR,        /*< Scaling factor of throttle vs torque                            */
    REG_TORQUE_OFFSET,         /*< Offset of throttle vs torque                                    */
    REG_TORQE_RAMP_UP,         /*< Torque ramp time in millisecond when controller is ramping UP   */
    REG_TORQE_RAMP_DOWN,       /*< Torque ramp time in millisecond when controller is ramping DOWN */
    REG_TORQUESENSOR_HIGH_THR, /*< ?? */
    REG_TORQUESENSOR_LOW_THR,  /*< ?? */
    // Speed parameters
    REG_STOP_SPEED,            /*< Minimum speed value for stoping the motor */
    REG_SPEED_SLOPE,           /*< Gain factor of throttle vs speed          */
    REG_SPEED_DIVISOR,         /*< Scaling factor of throttle vs speed       */
    REG_SPEED_OFFSET,          /*< Offset of throttle vs speed               */
    REG_SPEED_RAMP_UP,         /*< Speed ramp time in millisecond when controller is ramping UP    */
    REG_SPEED_RAMP_DOWN,       /*< Speed ramp time in millisecond when controller is ramping DOWN  */
    // PAS parameters
    REG_PAS_ENABLE,
    REG_PAS_TYPE,
    REG_PAS_MAXTORQUE,
    REG_PAS_LEVEL,
    REG_PAS_SENSE_DELAY,
    REG_PAS_POWERGAIN,
    REG_PAS_TORQUESENSOR_OFFSET,
    REG_PAS_MAXLEVEL,

    /* Battery parameters (512-767)*/
    REG_FAST_OVERVOLT_THR = 512,
    REG_FAST_UNDERVOLT_THR,
    REG_BAT_CURRENT_MAX,
    REG_SLOW_OVERVOLT_THR,
    REG_SLOW_UNDERVOLT_THR,
        
    /* Motor parameters (768-1023)*/
    // Motor no. 1
    REG_M1_ENABLE = 768,                        /*< To set once, true if motor 1 is used */
    REG_M1_TORQUE_KP,
    REG_M1_TORQUE_KI,
    REG_M1_FLUX_KP,
    REG_M1_FLUX_KI,
    REG_M1_DTHRESHOLD,
    REG_M1_DSLOPE,
    REG_M1_ANGLE_OFFSET,
    REG_M1_FF_C1,
    REG_M1_FF_C2,
    REG_M1_FF_C3,
    REG_M1_FW_KP,
    REG_M1_FW_KI,
    REG_M1_MAXPHASECURR,
    REG_M1_MAXMOTORTEMP,
    REG_M1_WHEELDIA,
    REG_M1_GEARRATIO,
    REG_M1_POLE_PAIR,
    REG_M1_SENSORLESS,
    REG_M1_OBS_G1,
    REG_M1_OBS_G2,
    REG_M1_RS,
    REG_M1_LS,
    REG_M1_KE,
    REG_M1_RATEDCURR,
    REG_M1_RATEDPOWER_RACE_PAS,
    REG_M1_RATEDPOWER_THROTTLE_RACE_THROTTLE,
    REG_M1_RATEDPOWER_STREET_PAS,
    REG_M1_RATEDPOWER_STREET_THROTTLE,
    REG_M1_RATEDSPEED,
    REG_M1_SPEEDLIMITER_OUTPUT,
    REG_M1_MOTOR_TEMP_RAW,
    REG_M1_AUTOTUNE_HALL_OFFSET,
    REG_M1_AUTOTUNE_LS,
    REG_M1_AUTOTUNE_RATEDSPEED,
    REG_M1_AUTOTUNE_RS,
    REG_M1_AUTOTUNE_KE,
    REG_M1_HALL_INTERPOL_START,
    REG_M1_HALL_INTERPOL_STOP,
    REG_M1_FWCURR_MAX,
    REG_M1_FLDB_END_MOTTEMP,
    REG_M1_FLDB_START_MOTTEMP,
    REG_M1_OPENLOOP_ANGLE,
    REG_M1_OPENLOOP_CURR,
    REG_M1_OPENLOOP_FREQ,
    REG_M1_OVRLOAD_CONTINUOUS_CURR,
    REG_M1_OVRLOAD_COOLING_CURR,
    REG_M1_OVRLOAD_COOLING_TIME,
    REG_M1_OVRLOAD_FLDB_END,
    REG_M1_OVRLOAD_FLDB_START,
    REG_M1_OVRLOAD_HEATING_CURR,
    REG_M1_OVRLOAD_HEATING_TIME,
    REG_M1_SENSORLESS_PLL_KI,
    REG_M1_SENSORLESS_PLL_KP,
    REG_M1_RATEDFREQ,
    REG_M1_REGENCURR_MAX,
    REG_M1_SENSORLESS_OPENLOOP_CURR_RAMPTIME,
    REG_M1_SENSORLESS_TRANSITION_FREQ,
    REG_M1_SENSORLESS_OPENLOOP_CURR_HOLDTIME,
    REG_M1_SENSORLESS_OPENLOOP_FREQ_RAMPTIME,
    REG_M1_SENSORLESS_OPENLOOP_CURR,
    REG_M1_SPEEDCTRL_KI,
    REG_M1_SPEEDCTRL_KP,
    REG_M1_HEATSINK_TEMP_RAW,
    REG_M1_HEATSINK_OVERTEMP_THR,
    REG_M1_REGENRATIO_MAX,
    REG_M1_FWRATIO_MAX,
    // Motor no. 2
    REG_M2_ENABLE,                              /*< To set once, true if motor 2 is used */
    REG_M2_TORQUE_KP,
    REG_M2_TORQUE_KI,
    REG_M2_FLUX_KP,
    REG_M2_FLUX_KI,
    REG_M2_DTHRESHOLD,
    REG_M2_DSLOPE,
    REG_M2_ANGLE_OFFSET,
    REG_M2_FF_C1,
    REG_M2_FF_C2,
    REG_M2_FF_C3,
    REG_M2_FW_KP,
    REG_M2_FW_KI,
    REG_M2_MAXPHASECURR,
    REG_M2_MAXMOTORTEMP,
    REG_M2_WHEELDIA,
    REG_M2_GEARRATIO,
    REG_M2_POLE_PAIR,
    REG_M2_SENSORLESS,
    REG_M2_NOMINALCURR,
    REG_M2_OBS_G1,
    REG_M2_OBS_G2,
    REG_M2_RS,
    REG_M2_LS,
    REG_M2_KE,
    REG_M2_RATEDCURR,
    REG_M2_RATEDPOWER_RACE_PAS,
    REG_M2_RATEDPOWER_THROTTLE_RACE_THROTTLE,
    REG_M2_RATEDPOWER_STREET_PAS,
    REG_M2_RATEDPOWER_STREET_THROTTLE,
    REG_M2_RATEDSPEED,
    REG_M2_SPEEDLIMITER_OUTPUT,
    REG_M2_MOTOR_TEMP_RAW,
    REG_M2_AUTOTUNE_HALL_OFFSET,
    REG_M2_AUTOTUNE_LS,
    REG_M2_AUTOTUNE_RATEDSPEED,
    REG_M2_AUTOTUNE_RS,
    REG_M2_AUTOTUNE_KE,
    REG_M2_HALL_INTERPOL_START,
    REG_M2_HALL_INTERPOL_STOP,
    REG_M2_FWCURR_MAX,
    REG_M2_FLDB_END_MOTTEMP,
    REG_M2_FLDB_START_MOTTEMP,
    REG_M2_OPENLOOP_ANGLE,
    REG_M2_OPENLOOP_CURR,
    REG_M2_OPENLOOP_FREQ,
    REG_M2_OVRLOAD_CONTINUOUS_CURR,
    REG_M2_OVRLOAD_COOLING_CURR,
    REG_M2_OVRLOAD_COOLING_TIME,
    REG_M2_OVRLOAD_FLDB_END,
    REG_M2_OVRLOAD_FLDB_START,
    REG_M2_OVRLOAD_HEATING_CURR,
    REG_M2_OVRLOAD_HEATING_TIME,
    REG_M2_SENSORLESS_PLL_KI,
    REG_M2_SENSORLESS_PLL_KP,
    REG_M2_RATEDFREQ,
    REG_M2_REGENCURR_MAX,
    REG_M2_SENSORLESS_OPENLOOP_CURR_RAMPTIME,
    REG_M2_SENSORLESS_TRANSITION_FREQ,
    REG_M2_SENSORLESS_OPENLOOP_CURR_HOLDTIME,
    REG_M2_SENSORLESS_OPENLOOP_FREQ_RAMPTIME,
    REG_M2_SENSORLESS_OPENLOOP_CURR,
    REG_M2_SPEEDCTRL_KI,
    REG_M2_SPEEDCTRL_KP,
    REG_M2_HEATSINK_TEMP_RAW,
    REG_M2_HEATSINK_OVERTEMP_THR,
    REG_M2_REGENRATIO_MAX,
    REG_M2_FWRATIO_MAX,
    /* Communication parameters (1024-1279)*/
    REG_EUART_PROTOCOL = 1024,
        REG_UART_BAUDRATE,       /*< Register for choose the UART protocol for communicate whether with EVionics tool or with a screen */
    REG_CAN_BAUDRATE,
    REG_CAN_HEARTBEAT_PERIOD,
    REG_CAN_ID,
    REG_SLAVE_ID,
    /* Manufacturing parameters (1280-1535)*/
    REG_TEST_MODE = 1280,
    REG_FIRMVER,
    REG_BOOTCOUNTER,
        REG_DEVICE_ID_LOW,
        REG_DEVICE_ID_HIGH
} VCI_RegID_t;

/* Function for reading the value of a register for the drive train */
int32_t VCI_ReadRegister(VCI_Handle_t* pHandle, uint16_t RegID);

/* Function for setting the value of a register for the drive train */
void VCI_SetRegister(VCI_Handle_t* pHandle, uint16_t RegID, int32_t value);

#endif
