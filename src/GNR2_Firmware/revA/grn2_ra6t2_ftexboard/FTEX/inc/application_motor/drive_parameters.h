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
#include "vc_parameters.h"
#include "hardware_parameters.h"


#if MOTOR_SELECTION == MOTOR_DEFAULT

#include "motor_parameters_default.h"

#elif MOTOR_SELECTION == MOTOR_AKM_128SX_350W

#include "motor_parameters_akm_128sx_350w.h"

#elif MOTOR_SELECTION == MOTOR_AKM_128SX_500W 

#include "motor_parameters_akm_128sx_500w.h"

#elif MOTOR_SELECTION == MOTOR_AKM_128SX_750W

#include "motor_parameters_akm_128sx_750w.h"

#elif MOTOR_SELECTION == MOTOR_NIDEC_B900_V3

#include "motor_parameters_nidec_b900_v3.h"

#elif MOTOR_SELECTION == MOTOR_BAFANG_G020_500W

#include "motor_parameters_bafang_g020_500w.h"

#elif MOTOR_SELECTION == MOTOR_BAFANG_G040_500W

#include "motor_parameters_bafang_g040_500w.h"

#elif MOTOR_SELECTION == MOTOR_BAFANG_G60_750W

#include "motor_parameters_bafang_g60_750w.h"

#elif MOTOR_SELECTION == MOTOR_BAFANG_G062_750W

#include "motor_parameters_bafang_g062_750w.h"

#elif MOTOR_SELECTION == MOTOR_BAFANG_G0900_750W

#include "motor_parameters_bafang_g0900_750w.h"

#elif MOTOR_SELECTION == MOTOR_TSUGAWA_L13S5_350W

#include "motor_parameters_tsugawa_li3s5_350w.h"

#elif MOTOR_SELECTION == MOTOR_RS2_1200W

#include "motor_parameters_rs2_1200w.h"

#elif MOTOR_SELECTION == MOTOR_UTK_G250R_CA11_350W 

#include "motor_parameters_utk_g250r_ca11_350w.h"

#elif MOTOR_SELECTION == MOTOR_GHR_0194_DD

#include "motor_parameters_ghr_0194_dd.h"

#elif MOTOR_SELECTION == MOTOR_SUPER73_500W

#include "motor_parameters_super73_500w.h"

#elif MOTOR_SELECTION == MOTOR_SUPER73_1200W

#include "motor_parameters_super73_1200w.h"

#endif



#if HARDWARE_SELECTION == HARDWARE_EP1200

#include "hardware_parameters_EP1200.h"

#elif HARDWARE_SELECTION == HARDWARE_EP600

#include "hardware_parameters_EP600.h"

#elif HARDWARE_SELECTION == HARDWARE_EP350

#include "hardware_parameters_EP350.h"

#elif HARDWARE_SELECTION == HARDWARE_EP700

#include "hardware_parameters_EP700.h"

#endif


/****** Constants ******/
#define MAX_CURRENT_LIMIT               1
#define MAX_POWER_LIMIT                 2

#define OCD2_DISABLE                    0
#define OCD2_ENABLED                    1

#define OCD1_POEG                       0
#define OCD2_POEG                       1

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

/****** Power Parameters ******/
#define DEFAULT_MAX_APPLICATION_CURRENT             PEAK_CURRENT_CONTROLLER_amps * 0.8          //Make default max DC current about 80% of the peak phase current
#define DEFAULT_MAX_APPLICATION_POSITIVE_POWER      DEFAULT_MAX_APPLICATION_CURRENT * OV_VOLTAGE_THRESHOLD_V
#define DEFAULT_MAX_APPLICATION_NEGATIVE_POWER      DEFAULT_MAX_APPLICATION_CURRENT * OV_VOLTAGE_THRESHOLD_V

/****** PWM Parameters ******/
#define PWM_FREQUENCY                   20000       // PWM switching frequency
#define PWM_FREQ_SCALING                1           // Not used, set to one.

#define MAX_DUTY                        INT16_MAX       // INT16_MAX is 100% duty cycle

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

#define SP_KPDIV                        256
#define SP_KDDIV                        16
#define SP_KPDIV_LOG                    LOG2(256)
#define SP_KDDIV_LOG                    LOG2(16)
#define SPD_CTRL_MAX_TORQUE             1000

#define DYNAMICTORQUE_THRESHOLD_SPEED    120

#define FOLDBACK_HS_TEMP_END_VALUE      OV_TEMP_CONTROLLER_THRESHOLD_C
#define FOLDBACK_HS_TEMP_INTERVAL       OV_TEMP_CONTROLLER_HYSTERESIS_C

#define DEFAULT_CONTROL_MODE            STC_TORQUE_MODE
#define MAX_POWER_RECOVER_TIMEOUT       80
#define OBS_MINIMUM_SPEED_RPM           580
#define NB_CONSECUTIVE_TESTS            2
#define SPEED_BAND_UPPER_LIMIT          17
#define SPEED_BAND_LOWER_LIMIT          15
#define TRANSITION_DURATION             25

/****** Flux Weakening Parameters  ******/
#define FW_VOLTAGE_REF                  900         // Flux weakening Vs reference, tenth
#define FW_KP_GAIN                      2000        // Flux weakening default Kp gain
#define FW_KI_GAIN                      100           // Flux weakening default Ki gain
#define FW_KPDIV                        32768       // Flux weakening gain divider, to allow decimal value
#define FW_KIDIV                        32768       // Flux weakening gain divider, to allow decimal value
#define FW_KPDIV_LOG                    LOG2(32768) // Flux weakening gain divider log2, to allow decimal value
#define FW_KIDIV_LOG                    LOG2(32768) // Flux weakening gain divider log2, to allow decimal value
#define ID_DEMAG_amps                   -30          // Demagnetization current

/****** Feedforward Parameters ******/
#define CONSTANT1_Q                     0           // Feedforward Iq related gain
#define CONSTANT1_D                     0           // Feedforward Id related gain
#define CONSTANT2_QD                    0           // Feedforward speed related gain

/***********Regen Parameters *********/
#define IQ_REGEN_AMPS                    -25         // regeneration current
#define MIN_REGEN_SPEED                 50         // minimum speed for activating the regen
#define RESET_SPEED                     25        //  speed for reseting the PIDs
#define MAX_NEG_DC_CURRENT              -5        // maximum negative DC currnet to battery in amps
#define REGEN_TORQUE_RAMP               -2        // the rate of increasing the regenerative current in Nm/msec

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
#define PHASE_WIRE_DISCONNECT_WAIT_MCCYCLE 3000      // The time to wait and check before raising the error
#define PHASE_DISC_OFFSET               300         // Offset to check for phase wire disconnection error, in digital current
#define MIN_PHASE_DISC_VALUE            120         // If current on one phase is under this value for a certain period of time,
                                                    // raise the error. In digital current
#define PHASE_DISC_DIVISOR              5           // If current on one phase is under a fifth of other phase currents, raise the error

/******** Stuck Protection Settings Section ********/
#define STUCK_TIMER_MAX_TICKS           1500        // protection timeout in MC Layer ticks
#define STUCK_TIMER_MAX_COUNTS          STUCK_TIMER_MAX_TICKS *SPEED_LOOP_FREQUENCY_HZ / 1000u - 1u     // protection timeout
#define STUCK_MIN_TORQUE                200         // minimum torque that can cause the protection to get activated
#define STUCK_LOW_VOLTAGE_THRESHOLD     42          // this parameter is used to reduce protection timeout when battery SoC is low
#define STUCK_TIMER_MAX_COUNTS_LOWBATTERY (STUCK_TIMER_MAX_TICKS / 10) * SPEED_LOOP_FREQUENCY_HZ / 1000u - 1u // the protection timeout battery SoC is detected as low

/******** Hall Sensor factors ********/
#define VIBRATION_PATTERN               0xAAAA      // = 0b1010101010101010 which is 8 time of direction change
                                        
/******** NTC Sensor Parameters ********/
#ifndef MOTOR_NTC_BETA_COEFFICIENT
    #define MOTOR_NTC_BETA_COEFFICIENT                     0    //Beta coefficient value as specified in the datasheet
#endif
#ifndef MOTOR_NTC_RESISTANCE_COEF
    #define MOTOR_NTC_RESISTANCE_COEF                     0    //This value is calculated based on this formula: exp(NTCBetaCoef / TEMP_25_CELSIUS_IN_KELVIN) / NTC Rated Resistance.
                                                               //NTC Rated Resistance = NTC resistance at 25 degree celsius in ohm
#endif

/******** Speed Control Settings Section ********/
#define SPDCTRL_UPPER_INTEGRAL_LIMIT    2097152     // =2^21 The maximum allowed value for Integral Term of Speed Control PID

#endif  /*__DRIVE_PARAMETERS_H*/
