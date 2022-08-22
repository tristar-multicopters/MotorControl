/**
  * @file    vc_parameters_e_cells.h
  * @brief   This file contains the parameters needed in order to configure the motor of grizzly bike.
  *
*/

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_E_CELLS_H
#define __VC_PARAMETERS_E_CELLS_H

#include "powertrain_management.h"


/***************** THROTTLE PARAMETERS  ******************************/

#define THROTTLE_FILTER_ALPHA                 2.27F    /* Butterworth alpha coefficient */
#define THROTTLE_FILTER_BETA                  -0.27F   /* Butterworth beta coefficient */

#define THROTTLE_OFFSET_ADC2THROTTLE          12500    /* Offset for ADC to throttle linear transformation  */
#define THROTTLE_SLOPE_ADC2THROTTLE           5        /* Slope for ADC to throttle linear transformation  */
#define THROTTLE_DIVISOR_ADC2THROTTLE         3        /* Divisor for ADC to throttle linear transformation  */

#define THROTTLE_OFFSET_THROTTLE2TORQUE       4000    /* Offset for throttle to torque linear transformation  */
#define THROTTLE_SLOPE_THROTTLE2TORQUE        14        /* Slope for throttle to torque linear transformation  */
#define THROTTLE_DIVISOR_THROTTLE2TORQUE      600        /* Divisor for throttle to torque linear transformation  */

#define THROTTLE_DETECTION_THRESHOLD          1000    /* Throttle is considered pressed once it passed this threshold  */


/***************** PEDAL TORQUE SENSOR PARAMETERS  ******************************/

#define PTS_FILTER_ALPHA                  2.27F    /* Butterworth alpha coefficient */
#define PTS_FILTER_BETA                   -0.27F    /* Butterworth beta coefficient */

#define PTS_OFFSET_ADC2PTS                12500    /* Offset for ADC to pedal torque sensor linear transformation  */
#define PTS_SLOPE_ADC2PTS                 5        /* Slope for ADC to pedal torque sensor linear transformation  */
#define PTS_DIVISOR_ADC2PTS               4        /* Divisor for ADC to pedal torque sensor linear transformation  */

#define PTS_OFFSET_PTS2TORQUE             20000    /* Offset for pedal torque sensor to torque linear transformation  */
#define PTS_SLOPE_PTS2TORQUE              -7        /* Slope for pedal torque sensor to torque linear transformation  */
#define PTS_DIVISOR_PTS2TORQUE            25        /* Divisor for pedal torque sensor to torque linear transformation  */


/***************** MOTOR SELECTOR PARAMETERS  ******************************/

#define MOTOR_SELECTOR_ENABLE              false      /* True if active motor can be changed using 3 way switch  */
  

/***************** POWER ENABLE PARAMETERS  ******************************/

#define POWER_ENABLE_ENABLE                false      /* (Sorry for bad name) True if power enable input is used to prevent powertrain start  */


/***************** POWERTRAIN MANAGEMENT PARAMETERS  ******************************/

#define POWERTRAIN_USE_MOTOR1                             true              /* True if motor1 is used  */
#define POWERTRAIN_USE_MOTOR2                             false              /* True if motor2 is used  */
#define POWERTRAIN_DEFAULT_MAIN_MOTOR                     M1                /* Default main motor, can be M1 or M2  */
#define POWERTRAIN_DEFAULT_MODE                           SINGLE_MOTOR      /* Default powertrain mode, can be SINGLE_MOTOR or DUAL_MOTOR  */
#define POWERTRAIN_DEFAULT_CONTROL_TYPE                   TORQUE_CTRL        /* Default control type, can be TORQUE_CTRL or SPEED_CTRL  */
#define POWERTRAIN_M2_TORQUE_INVERSION                    false              /* If true, M2 torque is inverted compared to M1  */
#define POWERTRAIN_START_THROTTLE_THRESHOLD               1000              /* Throttle value to start powertrain  */
#define POWERTRAIN_STOP_THROTTLE_THRESHOLD                100                /* Throttle value to stop powertrain  */
#define POWERTRAIN_STOP_SPEED_THRESHOLD                   0                  /* Speed value to stop powertrain  */
#define POWERTRAIN_PAS_MAX_TORQUE                         800               /* Maximum motor torque to apply using pedal assist  */
#define POWERTRAIN_PAS_MAX_SPEED                          500                /* Maximum motor speed reachable using pedal assist  */
#define POWERTRAIN_MOTOR_GEARRATIO                        0x000B0005        /* Motor gear ratio, i.e. wheel speed divided by motor speed.
                                                                                Upper half of 32 bits is numerator, second half is denominator */
#define POWERTRAIN_WHEEL_SPEED_SENSOR_PPR                 2                  /* Number of electrical pulses per wheel rotation */
#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT               200                /* Number of task ticks to wait after a fault occurs to attempt
                                                                              a powertrain restart (OC, SF and SU faults)   */


#endif                                            

