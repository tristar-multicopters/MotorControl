/**
  * @file    drive_parameters.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM       1500 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM       0 /*!< rpm, mechanical,
                                                           absolute value */
#define MEAS_ERRORS_BEFORE_FAULTS       6 /*!< Number of speed
                                                             measurement errors before
                                                             main sensor goes in fault */
/****** Hall sensors ************/
#define HALL_MEAS_ERRORS_BEFORE_FAULTS  6 /*!< Number of failed
                                                           derived class specific speed
                                                           measurements before main sensor
                                                           goes in fault */

#define HALL_AVERAGING_FIFO_DEPTH        8 /*!< depth of the FIFO used to
                                                           average mechanical speed in
                                                           0.1Hz resolution */
#define HALL_MTPA  true
/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD              0.062 /*!<Maximum accepted
                                                            variance on speed
                                                            estimates (percentage) */
/* State observer scaling factors F1 */
#define F1                               16384
#define F2                               8192
#define F1_LOG                           LOG2(16384)
#define F2_LOG                           LOG2(8192)

/* State observer constants */
#define GAIN1                            -23040
#define GAIN2                            28800
/*Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                      918
#define PLL_KI_GAIN                      32
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2(PLL_KPDIV)
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2(PLL_KIDIV)

#define OBS_MEAS_ERRORS_BEFORE_FAULTS    6  /*!< Number of consecutive errors
                                                           on variance test before a speed
                                                           feedback error is reported */
#define STO_FIFO_DEPTH_DPP               64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG           LOG2(64)

#define STO_FIFO_DEPTH_UNIT              64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in the unit defined by #SPEED_UNIT */
#define BEMF_CONSISTENCY_TOL             32   /* Parameter for B-emf
                                                            amplitude-speed consistency */
#define BEMF_CONSISTENCY_GAIN            64   /* Parameter for B-emf
                                                           amplitude-speed consistency */

/* USER CODE BEGIN angle reconstruction M1 */
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */

#define PWM_FREQUENCY   20000
#define PWM_FREQ_SCALING 1

#define SW_DEADTIME_NS                   250 /*!< Dead-time to be inserted
                                                           by FW, only if low side
                                                           signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                           number of PWM cycles */
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT         300
#define PID_TORQUE_KI_DEFAULT         4000
#define PID_TORQUE_KD_DEFAULT         100
#define PID_FLUX_KP_DEFAULT           100
#define PID_FLUX_KI_DEFAULT           6000
#define PID_FLUX_KD_DEFAULT           100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      4096
#define TF_KIDIV                      16384
#define TF_KDDIV                      8192
#define TF_KPDIV_LOG                  LOG2(4096)
#define TF_KIDIV_LOG                  LOG2(16384)
#define TF_KDDIV_LOG                  LOG2(8192)
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ       1000 /*!<Execution rate of speed
                                                      regulation loop (Hz) */

#define PID_SPEED_KP_DEFAULT          100
#define PID_SPEED_KI_DEFAULT          10
#define PID_SPEED_KD_DEFAULT          0
/* Speed PID parameter dividers */
#define SP_KPDIV                      256
#define SP_KIDIV                      16384
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2(256)
#define SP_KIDIV_LOG                  LOG2(16384)
#define SP_KDDIV_LOG                  LOG2(16)

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX                          32000

/* Default settings */
#define DEFAULT_CONTROL_MODE           STC_TORQUE_MODE /*!< STC_TORQUE_MODE or
                                                        STC_SPEED_MODE */
#define DEFAULT_TARGET_SPEED_RPM      1500
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/_RPM)
#define DEFAULT_TORQUE_COMPONENT       0
#define DEFAULT_FLUX_COMPONENT         0

#define DEFAULT_TORQUE_SLOPE_UP        1000
#define DEFAULT_TORQUE_SLOPE_DOWN      1000
#define DEFAULT_SPEED_SLOPE_UP         1000
#define DEFAULT_SPEED_SLOPE_DOWN       1000

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_PROT_ENABLING        ENABLE
#define UV_VOLTAGE_PROT_ENABLING        ENABLE
#define OV_VOLTAGE_THRESHOLD_V          75 /*!< Over-voltage
                                               //          threshold */
#define UD_VOLTAGE_THRESHOLD_V          24 /*!< Under-voltage
                                               //           threshold */
#if 0
#define ON_OVER_VOLTAGE                 TURN_OFF_PWM /*!< TURN_OFF_PWM,
                                                         TURN_ON_R_BRAKE or
                                                         TURN_ON_LOW_SIDES */
#endif /* 0 */
#define R_BRAKE_SWITCH_OFF_THRES_V      60

#define OV_TEMPERATURE_THRESHOLD_C      65 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     10 /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /*!< In case ON_OVER_VOLTAGE
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by
                                                          power stage) */
/******************************   START-UP PARAMETERS   **********************/

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM          580

#define NB_CONSECUTIVE_TESTS           2 /* corresponding to
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT         17 /*!< It expresses how much
                                                            estimated speed can exceed
                                                            forced stator electrical
                                                            without being considered wrong.
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT         15  /*!< It expresses how much
                                                             estimated speed can be below
                                                             forced stator electrical
                                                             without being considered wrong.
                                                             In 1/16 of forced speed */

#define TRANSITION_DURATION            25  /* Switch over duration, ms */
/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(24)
/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(24)
/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES (6 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

#define FW_VOLTAGE_REF                90 /*!<Vs reference, tenth
                                                        of a percent */
#define FW_KP_GAIN                    0 /*!< Default Kp gain */
#define FW_KI_GAIN                    0 /*!< Default Ki gain */
#define FW_KPDIV                      32768
                                                /*!< Kp gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through
                                                algebrical right shifts to speed up PIs execution.
                                                Only in this case this parameter specifies the
                                                number of right shifts to be executed */
#define FW_KIDIV                      32768
                                                /*!< Ki gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through
                                                algebrical right shifts to speed up PIs execution.
                                                Only in this case this parameter specifies the
                                                number of right shifts to be executed */
#define FW_KPDIV_LOG                  LOG2(32768)
#define FW_KIDIV_LOG                  LOG2(32768)
/*  Feed-forward parameters */
#define FEED_FORWARD_CURRENT_REG_ENABLING ENABLE
#define CONSTANT1_Q                    0
#define CONSTANT1_D                    0
#define CONSTANT2_QD                   0

/*** On the fly start-up ***/

/******************************   Rotor position observer Motor 1   **********************/

#define ROTOR_POS_OBS_KP         									1250
#define ROTOR_POS_OBS_KI         									0
#define ROTOR_POS_OBS_KD         									1000

#define ROTOR_POS_OBS_KPDIV         							256
#define ROTOR_POS_OBS_KIDIV         							256
#define ROTOR_POS_OBS_KDDIV         							1
#define ROTOR_POS_OBS_KPDIV_LOG                  	LOG2(256)
#define ROTOR_POS_OBS_KIDIV_LOG                  	LOG2(256)
#define ROTOR_POS_OBS_KDDIV_LOG                  	LOG2(1)


/******************************   Software overcurrent protection Motor 1   **********************/

#define OCSP_SAFETY_MARGIN 	            4000	/* Measured current amplitude can be until SOCP_SAFETY_MARGIN higher
                                                than reference current before overcurrent software protection triggers */
#define OCSP_MAX_CURRENT                22000 /* Max current that can be reached before triggering software overcurrent */



#endif /*__DRIVE_PARAMETERS_H*/

