/**
 * @file    drive_parameters.h
 * @brief   This file contains the parameters needed in order to configure the motor to drive.
 *
 */

/* Define to prevent recursive inclusion ---
----------------------------------*/
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

#include "gnr_parameters.h"

#define CONTROLLER_EP1200       0
#define CONTROLLER_EP600        1
#define CONTROLLER_EP350        2
#define CONTROLLER_EP700        3


#if VEHICLE_SELECTION == VEHICLE_A2_350W

#include "drive_parameters_a2_350w.h"

#elif VEHICLE_SELECTION == VEHICLE_A2_500W 

#include "drive_parameters_a2_500w.h"


#elif VEHICLE_SELECTION == VEHICLE_E_CELLS

#include "drive_parameters_e_cells.h"

#elif VEHICLE_SELECTION == VEHICLE_MAHLE

#include "drive_parameters_mahle.h"

#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#include "drive_parameters_nidec.h"

#elif VEHICLE_SELECTION == VEHICLE_PEGATRON

#include "drive_parameters_pegatron.h"

#elif VEHICLE_SELECTION == VEHICLE_QUIETKAT

#include "drive_parameters_quietkat.h"

#elif VEHICLE_SELECTION == VEHICLE_R48_750W

#include "drive_parameters_r48_750w.h"

#elif VEHICLE_SELECTION == VEHICLE_TSUGAWA

#include "drive_parameters_tsugawa.h"

#elif VEHICLE_SELECTION == VEHICLE_UTK_350W 

#include "drive_parameters_utk_350w.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC_CITI_500W

#include "drive_parameters_velec_citi_500w.h"

#endif



#if CONTROLLER_SELECTION == CONTROLLER_EP1200

#include "controller_parameters_EP1200.h"

#elif CONTROLLER_SELECTION == CONTROLLER_EP600

#include "controller_parameters_EP600.h"

#elif CONTROLLER_SELECTION == CONTROLLER_EP350

#include "controller_parameters_EP350.h"

#elif CONTROLLER_SELECTION == CONTROLLER_EP700

#include "controller_parameters_EP700.h"

#endif


/****** Constants ******/
#define MAX_CURRENT_LIMIT               1
#define MAX_POWER_LIMIT                 2
#define OCD2_DISABLE                    0
#define OCD2_ENABLED                    1

/****** Motor Parameters ******/
#define MIN_APPLICATION_SPEED_RPM       0           // Min speed for the current application in mechanical rpm
#define MEAS_ERRORS_BEFORE_FAULTS       6           // Number of speed measurement errors before triggering faults
#define HALL_MEAS_ERRORS_BEFORE_FAULTS  6           // Number of failed Hall sensor measurements before triggering faults
#define HALL_MTPA                       true        // Must be set true. TODO: Remove that parameter.
#define VARIANCE_THRESHOLD              0.062       // Maximum accepted variance for speed regulation

#define F1                              16384
#define F2                              8192
#define F1_LOG                          LOG2(16384)
#define F2_LOG                          LOG2(8192)

#define GAIN1                           -23040
#define GAIN2                           28800
#define PLL_KP_GAIN                     918
#define PLL_KI_GAIN                     32
#define PLL_KPDIV                       16384
#define PLL_KPDIV_LOG                   LOG2(PLL_KPDIV)
#define PLL_KIDIV                       65535
#define PLL_KIDIV_LOG                   LOG2(PLL_KIDIV)
#define OBS_MEAS_ERRORS_BEFORE_FAULTS   6           // Number of consecutive errors in observer before triggering faults
#define STO_FIFO_DEPTH_DPP              64          // Depth of the FIFO used for observer
#define STO_FIFO_DEPTH_DPP_LOG          LOG2(64)
#define STO_FIFO_DEPTH_UNIT             64          // Depth of the FIFO used for observer
#define BEMF_CONSISTENCY_TOL            32          // Parameter for B-emf consistency check
#define BEMF_CONSISTENCY_GAIN           64          // Parameter for B-emf consistency check

/****** Current Filtering Parameters ******/
#define CURRENT_FILTER_ALPHA            2.273F      // Alpha constant used in butterworth filter for current filtering
#define CURRENT_FILTER_BETA             -0.273F     // Beta constant used in butterworth filter for current filtering

/****** PWM Parameters ******/
#define PWM_FREQUENCY                   20000       // PWM switching frequency
#define PWM_FREQ_SCALING                1           // Not used, set to one.

/****** FOC Parameters ******/
#define No_Load_PID_KIq_Gain            500
#define REGULATION_EXECUTION_RATE       1

#define PID_TORQUE_KI_DEFAULT           1000
#define PID_TORQUE_KD_DEFAULT           100
#define PID_FLUX_KD_DEFAULT             100

#define TF_KPDIV                        4096
#define TF_KIDIV                        16384
#define TF_KDDIV                        8192
#define TF_KPDIV_LOG                    LOG2(4096)
#define TF_KIDIV_LOG                    LOG2(16384)
#define TF_KDDIV_LOG                    LOG2(8192)

#define SPEED_LOOP_FREQUENCY_HZ         1000

#define PID_SPEED_KD_DEFAULT            0
#define PID_SPEEDLIMIT_KP_DEFAULT       PID_SPEED_KP_DEFAULT
#define PID_SPEEDLIMIT_KI_DEFAULT       PID_SPEED_KI_DEFAULT

#define SP_KPDIV                        256
#define SP_KDDIV                        16
#define SP_KPDIV_LOG                    LOG2(256)
#define SP_KDDIV_LOG                    LOG2(16)
#define SPD_CTRL_MAX_TORQUE             1000

#define DYNAMICTORQUE_THRESHOLD_SPEED    120

#define FOLDBACK_SPEED_END_VALUE        MAX_APPLICATION_SPEED_RPM
#define FOLDBACK_HS_TEMP_END_VALUE      OV_TEMP_CONTROLLER_THRESHOLD_C
#define FOLDBACK_HS_TEMP_INTERVAL       OV_TEMP_CONTROLLER_HYSTERESIS_C
#define FOLDBACK_MOTOR_TEMP_END_VALUE   OV_TEMP_MOTOR_THRESHOLD_C

#define DEFAULT_CONTROL_MODE            STC_TORQUE_MODE
#define MAX_POWER_RECOVER_TIMEOUT       80
#define OBS_MINIMUM_SPEED_RPM           580
#define NB_CONSECUTIVE_TESTS            2
#define SPEED_BAND_UPPER_LIMIT          17
#define SPEED_BAND_LOWER_LIMIT          15
#define TRANSITION_DURATION             25

/****** Flux Weakening Parameters  ******/
#define FW_VOLTAGE_REF                  900         // Flux weakening Vs reference, tenth
#define FW_KP_GAIN                      0           // Flux weakening default Kp gain
#define FW_KI_GAIN                      0           // Flux weakening default Ki gain
#define FW_KPDIV                        32768       // Flux weakening gain divider, to allow decimal value
#define FW_KIDIV                        32768       // Flux weakening gain divider, to allow decimal value
#define FW_KPDIV_LOG                    LOG2(32768) // Flux weakening gain divider log2, to allow decimal value
#define FW_KIDIV_LOG                    LOG2(32768) // Flux weakening gain divider log2, to allow decimal value
#define ID_DEMAG_amps                   -5          // Demagnetization current

/****** Feedforward Parameters ******/
#define CONSTANT1_Q                     0           // Feedforward Iq related gain
#define CONSTANT1_D                     0           // Feedforward Id related gain
#define CONSTANT2_QD                    0           // Feedforward speed related gain

/****** Rotor Position Observer Parameters ******/
#define ROTOR_POS_OBS_KP                1250        // Rotor position observer default gain
#define ROTOR_POS_OBS_KI                0           // Rotor position observer default gain
#define ROTOR_POS_OBS_KD                1000        // Rotor position observer default gain
#define ROTOR_POS_OBS_KPDIV             256         // Rotor position observer gain divider, to allow decimal value
#define ROTOR_POS_OBS_KIDIV             256         // Rotor position observer gain divider, to allow decimal value
#define ROTOR_POS_OBS_KDDIV             1           // Rotor position observer gain divider, to allow decimal value
#define ROTOR_POS_OBS_KPDIV_LOG         LOG2(256)   // Rotor position observer gain divider log2, to allow decimal value
#define ROTOR_POS_OBS_KIDIV_LOG         LOG2(256)   // Rotor position observer gain divider log2, to allow decimal value
#define ROTOR_POS_OBS_KDDIV_LOG         LOG2(1)     // Rotor position observer gain divider log2, to allow decimal value

/****** Phase Wire Disconnection Detection *******/
#define CURRENT_AVG_WIN_SIZE            32          // the moving average window size
#define PHASE_WIRE_DISCONNECT_WAIT_MCCYCLE 100      // The time to wait and check before raising warning

/******** Stuck Protection Settings Section ********/
#define STUCK_TIMER_MAX_TICKS           2000        // protection timeout in MC Layer ticks
#define STUCK_TIMER_MAX_COUNTS          STUCK_TIMER_MAX_TICKS *SPEED_LOOP_FREQUENCY_HZ / 1000u - 1u     // protection timeout
#define STUCK_MIN_TORQUE                200         // minimum torque that can cause the protection to get activated
#define STUCK_LOW_VOLTAGE_THRESHOLD     42          // this parameter is used to reduce protection timeout when battery SoC is low
#define STUCK_TIMER_MAX_COUNTS_LOWBATTERY (STUCK_TIMER_MAX_TICKS / 10) * SPEED_LOOP_FREQUENCY_HZ / 1000u - 1u // the protection timeout battery SoC is detected as low

/******** Hall Sensor factors ********/
#define VIBRATION_PATTERN               0xAAAA      // = 0b1010101010101010 which is 8 time of direction change
                                        
/******** Speed Control Settings Section ********/
#define SPDCTRL_UPPER_INTEGRAL_LIMIT    2097152     // =2^21 The maximum allowed value for Integral Term of Speed Control PID

#endif  /*__DRIVE_PARAMETERS_H*/
