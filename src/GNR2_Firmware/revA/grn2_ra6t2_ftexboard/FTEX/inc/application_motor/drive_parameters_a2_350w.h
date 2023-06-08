/**
  * @file    drive_parameters_velec_a2.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. This file is specific to velec A2 motor.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_VELEC_A2_H
#define __DRIVE_PARAMETERS_VELEC_A2_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/		// motorRPM=speed(km/h)*GR*100000/(R*pi*2.54*60)   where R=25 inch and GR=7.86
#define MAX_APPLICATION_SPEED_RPM       4000 //2600   /*!< Max speed for the current application in mechanical rpm */
                                               /* Old Example 2750 for 38Km/h */
#define MIN_APPLICATION_SPEED_RPM       0     /*!< Min speed for the current application in mechanical rpm */
#define MEAS_ERRORS_BEFORE_FAULTS       6     /*!< Number of speed
                                                             measurement errors before
                                                             main sensor goes in fault */ 

#define No_Load_PID_KIq_Gain    500  /* this is the Kiq  when the torque set is very low
                                                             main sensor goes in fault */                                                           
/****** Hall sensors ************/
#define HALL_MEAS_ERRORS_BEFORE_FAULTS  6 /*!< Number of failed
                                                           derived class specific speed
                                                           measurements before main sensor
                                                           goes in fault */

#define HALL_AVERAGING_FIFO_DEPTH        8 /*!< depth of the FIFO used to
                                                           average mechanical speed */

#define HALL_MTPA  true                                 /* Must be set true. TODO: Remove that parameter. */
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

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/

#define OV_VOLTAGE_THRESHOLD_V          75 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V          32 /*!< Under-voltage threshold */


#define OV_TEMPERATURE_THRESHOLD_C      70 /*!< Heatsink overtemperature threshold before thermal shutdown. Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     15 /*!< Heatsink overtemperature hysteresis after a thermal shutdown occured. Celsius degrees */

#define OCSP_SAFETY_MARGIN_amps 	            85	/* Measured current amplitude can be until SOCP_SAFETY_MARGIN higher
                                                than reference current before overcurrent software protection triggers */
#define OCSP_MAX_CURRENT_amps                85   /* Max current that can be reached before triggering software overcurrent */
#define CURRENT_FILTER_ALPHA            2.273F       /* Alpha constant used in butterworth filter for current filtering */
#define CURRENT_FILTER_BETA             -0.273F      /* Beta constant used in butterworth filter for current filtering */


/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */

#define PWM_FREQUENCY   20000           /* PWM switching frequency */
#define PWM_FREQ_SCALING 1              /* Not used, set to one. */

#define SW_DEADTIME_NS                   250 /*!< Dead-time to be inserted by FW */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                           number of PWM cycles */
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT         300       /* Default gain if adaptative gain feature is not used */
#define PID_TORQUE_KI_DEFAULT         1000      /* Default gain if adaptative gain feature is not used */
#define PID_TORQUE_KD_DEFAULT         100       /* Default gain if adaptative gain feature is not used */
#define PID_FLUX_KP_DEFAULT           100       /* Default gain if adaptative gain feature is not used */
#define PID_FLUX_KI_DEFAULT           1000      /* Default gain if adaptative gain feature is not used */
#define PID_FLUX_KD_DEFAULT           100       /* Default gain if adaptative gain feature is not used */

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      4096          /* Gain divider, to allow decimal value */
#define TF_KIDIV                      16384         /* Gain divider, to allow decimal value */
#define TF_KDDIV                      8192          /* Gain divider, to allow decimal value */o
#define TF_KPDIV_LOG                  LOG2(4096)    /* Gain divider, to allow decimal value */
#define TF_KIDIV_LOG                  LOG2(16384)   /* Gain divider, to allow decimal value */
#define TF_KDDIV_LOG                  LOG2(8192)    /* Gain divider, to allow decimal value */

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ       1000 /*!<Execution rate of speed
                                                      regulation loop (Hz) */

#define PID_SPEED_KP_DEFAULT          100           /* Default gain speed control loop */
#define PID_SPEED_KI_DEFAULT          10            /* Default gain speed control loop */
#define PID_SPEED_KD_DEFAULT          0             /* Default gain speed control loop */
/* Speed PID parameter dividers */
#define SP_KPDIV                      256           /* Speed control gain divider, to allow decimal value */
#define SP_KIDIV                      16384         /* Speed control gain divider, to allow decimal value */
#define SP_KDDIV                      16            /* Speed control gain divider, to allow decimal value */
#define SP_KPDIV_LOG                  LOG2(256)     /* Speed control gain divider log2, to allow decimal value */
#define SP_KIDIV_LOG                  LOG2(16384)   /* Speed control gain divider log2, to allow decimal value */
#define SP_KDDIV_LOG                  LOG2(16)      /* Speed control gain divider log2, to allow decimal value */

#define SPD_CTRL_MAX_TORQUE             1000        /* Maximum torque that speed control loop can apply */

#define LOW_BATTERY_TORQUE              150         /* Maximum torque allowed in low SoC of battery */
#define LOW_BATTERY_VOLTAGE_THRESHOLD   35          /* the threshold of battery voltage before limit torque */

#define MAX_CURRENT_LIMIT               1           /* to calculate maximum poweer based on maximum current and Battery S0C */
#define MAX_POWER_LIMIT                 2           /* to use  MAX_APPLICATION_POSITIVE_POWER as the reference for Maximum power*/
#define POWER_LIMIT_REF                 MAX_POWER_LIMIT     /* defines if the code should use MAX_APPLICATION_POSITIVE_POWER or MAX_APPLICATION_CURRENT
                                                            as the reference for maximum power limitation*/
#define MAX_APPLICATION_POSITIVE_POWER  1100    /* Refers to maximum power in watts that drive can push to the motor */
#define MAX_APPLICATION_NEGATIVE_POWER  1100    /* Refers to maximum power in watts that drive can accept from the motor */
#define MAX_APPLICATION_CURRENT         22      /* Refers to maximum battery current in amps that drive can accept from the motor */

#define DYNAMICTORQUE_THRESHOLD_SPEED  120       /* Refers to motor speed which starts the transition between STARTING_TORQUE and NOMINAL_TORQUE */
  
/* Foldbacks */
#define FOLDBACK_SPEED_END_VALUE        MAX_APPLICATION_SPEED_RPM   /* Max speed value (#SPEED_UNIT) of the decreasing torque ramp to limit speed */
#define FOLDBACK_SPEED_INTERVAL         0//750 Removed to let VC control top speed /* Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed */

#define FOLDBACK_HS_TEMP_END_VALUE      OV_TEMPERATURE_THRESHOLD_C   /* Max temperature value (degree C) of the decreasing torque ramp to limit heatsink temperature */
#define FOLDBACK_HS_TEMP_INTERVAL       OV_TEMPERATURE_HYSTERESIS_C  /* Temperature interval (degree C) of the decreasing torque ramp to limit heatsink temperature */

#define FOLDBACK_MOTOR_TEMP_END_VALUE   70                          /* Max temperature value (degree C) of the decreasing torque ramp to limit motor temperature */
#define FOLDBACK_MOTOR_TEMP_INTERVAL    20                         /* Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature */

/* Control mode */
#define DEFAULT_CONTROL_MODE           STC_TORQUE_MODE /*!< Torque control or speed control. Can be STC_TORQUE_MODE or STC_SPEED_MODE */

/*Torque ramp settings */
#define DEFAULT_TORQUE_SLOPE_UP        5000        /* Slope in cNm per second */
#define DEFAULT_TORQUE_SLOPE_DOWN      10000        /* Slope in cNm per second */
#define DEFAULT_SPEED_SLOPE_UP         500        /* Slope in #SPEED_UNIT per second */
#define DEFAULT_SPEED_SLOPE_DOWN       500        /* Slope in #SPEED_UNIT per second */

/* Dynamic maximum power foldback settings */
#define ENABLE_MAX_POWER_LIMIT          true        /* to enable or disable the foldback */
#define MAX_BMS_POSITIVE_POWER          500         /* Maximum Power at the end point of foldback */
#define MAX_TIME_BMS_TOLERANT           20000       /* milliseconds - time of foldback end point */
#define MAX_POWER_LIMIT_TIMEOUT         10000       /* milliseconds - timeout before start derating */
#define MAX_POWER_RECOVER_TIMEOUT       80          /* milliseconds - timeout before reset maximum power to default value */
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

/******************************   ADDITIONAL FEATURES   **********************/

/*  Flux weakening parameters */
#define FW_VOLTAGE_REF                900 /*!<Flux weakening Vs reference, tenth
                                                        of a percent */
#define FW_KP_GAIN                    0 /*!< Flux weakening default Kp gain */
#define FW_KI_GAIN                    0 /*!< Flux weakening default Ki gain */

#define FW_KPDIV                      32768     /*!< Flux weakening gain divider, to allow decimal value */
#define FW_KIDIV                      32768     /*!< Flux weakening gain divider, to allow decimal value */                                         
#define FW_KPDIV_LOG                  LOG2(32768)       /*!< Flux weakening gain divider log2, to allow decimal value */
#define FW_KIDIV_LOG                  LOG2(32768)       /*!< Flux weakening gain divider log2, to allow decimal value */

/*  Feed-forward parameters */
#define CONSTANT1_Q                    0            /* Feedforward Iq related gain */
#define CONSTANT1_D                    0            /* Feedforward Id related gain */
#define CONSTANT2_QD                   0            /* Feedforward speed related gain */

/*** On the fly start-up ***/

/******************************   Rotor position observer Motor 1   **********************/

#define ROTOR_POS_OBS_KP         		1250        /* Rotor position observer default gain */
#define ROTOR_POS_OBS_KI         		0           /* Rotor position observer default gain */
#define ROTOR_POS_OBS_KD         		1000        /* Rotor position observer default gain */

#define ROTOR_POS_OBS_KPDIV             256         /* Rotor position observer gain divider, to allow decimal value */
#define ROTOR_POS_OBS_KIDIV         	256         /* Rotor position observer gain divider, to allow decimal value */
#define ROTOR_POS_OBS_KDDIV         	1           /* Rotor position observer gain divider, to allow decimal value */
#define ROTOR_POS_OBS_KPDIV_LOG         LOG2(256)   /* Rotor position observer gain divider log2, to allow decimal value */
#define ROTOR_POS_OBS_KIDIV_LOG         LOG2(256)   /* Rotor position observer gain divider log2, to allow decimal value */
#define ROTOR_POS_OBS_KDDIV_LOG         LOG2(1)     /* Rotor position observer gain divider log2, to allow decimal value */

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA     16.91F /*!< Alpha constant to configure butterworth filter for mecanical speed filtering */
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA     -14.91F /*!< Beta constant to configure butterworth filter for mecanical speed filtering */


#endif /*__DRIVE_PARAMETERS_VELEC_H*/
