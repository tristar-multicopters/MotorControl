/**
  * @file    vc_parameters_grizzly.h
  * @brief   This file contains the parameters needed in order to configure the motor of grizzly bike.
  *
*/

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_GRIZZLY_H
#define __VC_PARAMETERS_GRIZZLY_H

#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"


/***************** THROTTLE PARAMETERS  ******************************/

#define THROTTLE_FILTER_ALPHA                 2.27F    /* Butterworth alpha coefficient for throttle filtering */
#define THROTTLE_FILTER_BETA                  -0.27F   /* Butterworth beta coefficient for throttle filtering */

#define THROTTLE_OFFSET_ADC2THROTTLE          12500    /* Offset for ADC to throttle linear transformation  */
#define THROTTLE_SLOPE_ADC2THROTTLE           5        /* Slope for ADC to throttle linear transformation  */
#define THROTTLE_DIVISOR_ADC2THROTTLE         7        /* Divisor for ADC to throttle linear transformation  */

#define THROTTLE_OFFSET_THROTTLE2TORQUE       4000    /* Offset for throttle to torque linear transformation  */
#define THROTTLE_SLOPE_THROTTLE2TORQUE        9      /* Slope for throttle to torque linear transformation  */
#define THROTTLE_DIVISOR_THROTTLE2TORQUE      140     /* Divisor for throttle to torque linear transformation  */

#define THROTTLE_DETECTION_THRESHOLD          1000    /* Throttle is considered pressed once it passed this threshold  */


/***************** PEDAL TORQUE SENSOR PARAMETERS  ******************************/

#define PTS_FILTER_ALPHA                  2.27F				/* Butterworth alpha coefficient pedal torque sensor filtering */
#define PTS_FILTER_BETA                   -0.27F			/* Butterworth beta coefficient pedal torque sensor filtering  */
#define PTS_MAX_PTSVALUE                  UINT16_MAX	    /* Maximum analog value to reach */

#define PTS_OFFSET_ADC2PTS                8500    /* Offset for ADC to pedal torque sensor linear transformation  */
#define PTS_SLOPE_ADC2PTS                 7        /* Slope for ADC to pedal torque sensor linear transformation  */
#define PTS_DIVISOR_ADC2PTS               6        /* Divisor for ADC to pedal torque sensor linear transformation  */

#define PTS_OFFSET_PTS2TORQUE             20    		/* Offset for pedal torque sensor to torque linear transformation  */
#define PTS_SLOPE_PTS2TORQUE              7        /* Slope for pedal torque sensor to torque linear transformation  */
#define PTS_DIVISOR_PTS2TORQUE            255        /* Divisor for pedal torque sensor to torque linear transformation  */

#define PTS_FILTER_BW1                    10        /* BW coefficient for pedal torque sensor avereging */
#define PTS_FILTER_BW2                    25        /* BW coefficient for pedal torque sensor avereging */

/************** WHEEL SPEED SENSOR PARAMETERS  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION 4       /* Wheel speed sensor cycle number for one wheel rotation */
#define WHEEL_SPEED_SLOW_LOOP_DETECT        false   /* Wheel speed sensor slow detect counter flag activation */
#define WHEEL_SPEED_SLOW_LOOP_COUNT         1       /* Wheel speed sensor slow detect counter to get 750ms per function call */
#define WHEEL_SPEED_SENSOR_CORRECTION_FACTOR 1      /* Wheel speed sensor slow detect correction for a signal after two wheel spin detection */    

/***************** MOTOR SELECTOR PARAMETERS  ******************************/

#define MOTOR_SELECTOR_ENABLE              false      /* True if active motor can be changed using 3 way switch  */
  

/***************** POWER ENABLE PARAMETERS  ******************************/

#define POWER_ENABLE_ENABLE                false      /* (Sorry for bad name) True if power enable input is used to prevent powertrain start  */


/***************** POWERTRAIN MANAGEMENT PARAMETERS  ******************************/

#define POWERTRAIN_USE_MOTOR1                             true              /* True if motor1 is used  */
#define POWERTRAIN_USE_MOTOR2                             false             /* True if motor2 is used  */
#define POWERTRAIN_DEFAULT_MAIN_MOTOR                     M1                /* Default main motor, can be M1 or M2  */
#define POWERTRAIN_DEFAULT_MODE                           SINGLE_MOTOR      /* Default powertrain mode, can be SINGLE_MOTOR or DUAL_MOTOR  */
#define POWERTRAIN_DEFAULT_CONTROL_TYPE                   TORQUE_CTRL       /* Default control type, can be TORQUE_CTRL or SPEED_CTRL  */
#define POWERTRAIN_M2_TORQUE_INVERSION                    false			    /* If true, M2 torque is inverted compared to M1  */
#define POWERTRAIN_START_THROTTLE_THRESHOLD               1000	            /* Throttle value to start powertrain  */
#define POWERTRAIN_STOP_THROTTLE_THRESHOLD                100               /* Throttle value to stop powertrain  */
#define POWERTRAIN_STOP_SPEED_THRESHOLD                   0	                /* Speed value to stop powertrain  */
#define POWERTRAIN_PAS_MAX_TORQUE                         1700              /* Maximum motor torque to apply using pedal assist  */
#define POWERTRAIN_PAS_MAX_SPEED                          1350              /* Maximum motor speed reachable using pedal assist  */
#define POWERTRAIN_PAS_MAX_KM_SPEED                       33                /* Maximum Bike Speed in Km/h using RPM */

#define POWERTRAIN_PAS_MAX_LEVEL                          5                 /* Maximum PAS Level given by the screen */
#define POWERTRAIN_PAS_LEVEL_COEFF                        1                 /* PAS ramp multiplication coefficient for a better user feeling   */
#define POWERTRAIN_PAS_MAX_TORQUE_RATIO                   80                /* Maximum PAS Torque feed ration in 100% */
#define POWERTRAIN_PAS_MAX_SPEED_RATIO                    80                /* Maximum PAS Speed feed ration in 100% */
#define POWERTRAIN_PAS_TORQUE_USE                         true              /* PAS based torque sensor use Flag */

#define POWERTRAIN_MOTOR_GEARRATIO                        0x000B0005        /* Motor gear ratio, i.e. wheel speed divided by motor speed.
                                                                                Upper half of 32 bits is numerator, second half is denominator */
#define POWERTRAIN_WHEEL_SPEED_SENSOR_PPR                 2                  /* Number of electrical pulses per wheel rotation */
#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT               200                /* Number of task ticks to wait after a fault occurs to attempt
                                                                              a powertrain restart (OC, SF and SU faults)   */
#define POWERTRAIN_MAX_MOTOR_TORQUE                       STARTING_TORQUE                   /* Maximum motor torque to apply with powertrain management */
#define POWERTRAIN_DUAL_MOTOR_STARTUP_ENABLE              false                             /* Enable dual motor startup when single motor mode is selected */
#define POWERTRAIN_DUAL_MOTOR_STARTUP_SPEED_END           MAX_APPLICATION_SPEED_RPM/2       /* Speed value that dual motor startup strategy stops outputting
                                                                                               torque */
#define POWERTRAIN_DUAL_MOTOR_STARTUP_SPEED_INTERVAL      MAX_APPLICATION_SPEED_RPM/4       /* Speed interval value between maximum torque and zero torque,
                                                                                               when using dual motor startup strategy. */
#define POWERTRAIN_DUAL_MOTOR_SPEED_INTERVAL              MAX_APPLICATION_SPEED_RPM/10      /* Speed interval value between maximum torque and start torque,
                                                                                               when using single motor startup strategy. */
#define FOLDBACK_SLOW_START_BANDWIDTH                     250               /* Fold Back slow start filter bandwidth coefficient for slow ramp to the PAS Control */
#define FOLDBACK_SLOW_STOP_BANDWIDTH                      50                /* Fold Back slow stop filter bandwidth coefficient for slow ramp to the PAS Control */
#define FOLDBACK_TIMEOUT                                  400               /* Fold Back Timeout for the slow start ramp */

/*********************************Battery Monitoring*******************************/

#define BATTERY_FULL_VOLT    52
#define BATTERY_EMPTY_VOLT   46

#endif                                            

