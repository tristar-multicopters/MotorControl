/**
  * @file    vc_parameters.h
    * @author     Sami Bouzid, FTEX inc     
  * @brief   This file contains the parameters needed in order to configure vehicle control layer.
  *
*/

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_H
#define __VC_PARAMETERS_H

#include "gnr_parameters.h"
#include "ramps.h"

#define MOTOR_AKM_128SX_750W          1
#define MOTOR_AKM_128SX_500W          2
#define MOTOR_AKM_128SX_350W          3
#define MOTOR_BAFANG_G020_500W        4
#define MOTOR_BAFANG_G040_500W        5
#define MOTOR_BAFANG_G062_750W        6
#define MOTOR_BAFANG_G60_750W         7
#define MOTOR_BAFANG_G0900_750W       8
#define MOTOR_NIDEC_B900_V3           9
#define MOTOR_TSUGAWA_L13S5_350W     10
#define MOTOR_UTK_G250R_CA11_350W    11
#define MOTOR_RS2_1200W              12
#define MOTOR_GHR_0194_DD            13
#define MOTOR_SUPER73_500W           14
#define MOTOR_SUPER73_1200W          15

#if VEHICLE_SELECTION == VEHICLE_DEFAULT 

#include "vc_parameters_default.h"

#elif VEHICLE_SELECTION == VEHICLE_A2_350W 

#include "vc_parameters_a2_350w.h"

#elif VEHICLE_SELECTION ==  VEHICLE_A2_500W 

#include "vc_parameters_a2_500w.h"

#elif VEHICLE_SELECTION ==  VEHICLE_E_CELLS

#include "vc_parameters_e_cells.h"
 
#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#include "vc_parameters_nidec.h"

#elif VEHICLE_SELECTION == VEHICLE_PEGATRON

#include "vc_parameters_pegatron.h"

#elif VEHICLE_SELECTION == VEHICLE_QUIETKAT

#include "vc_parameters_quietkat.h"

#elif VEHICLE_SELECTION == VEHICLE_R48_750W

#include "vc_parameters_r48_750w.h"

#elif VEHICLE_SELECTION ==  VEHICLE_SUPER73_Z

#include "vc_parameters_super73_z.h"

#elif VEHICLE_SELECTION == VEHICLE_SUPER73_R

#include "vc_parameters_super73_r.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC_CITI_500W

#include "vc_parameters_velec_citi_500w.h"

#elif VEHICLE_SELECTION == VEHICLE_MILEBOX

#include "vc_parameters_milebox.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC21_500W

#include "vc_parameters_vel21_500w.h"

#endif

#ifndef  VEHICLE_TOP_SPEED_KMH
    #define VEHICLE_TOP_SPEED_KMH   32 
#endif

//Percentage of time that the wheel speed sensor spends on each magnet
#ifndef EXTERNAL_WSS_TIME_ON_ONE_MAGNET_PERCENT
    #define EXTERNAL_WSS_TIME_ON_ONE_MAGNET_PERCENT (float)5
#endif

// PAS sensor Torque power in % default values

#ifndef PAS_0_TORQUE_GAIN
    #define PAS_0_TORQUE_GAIN      0    // PAS T (Toque) 0 has a ratio of   0%
#endif

#ifndef PAS_1_TORQUE_GAIN
    #define PAS_1_TORQUE_GAIN     100    // PAS T (Toque) has a ratio of  100%
#endif

#ifndef PAS_2_TORQUE_GAIN
    #define PAS_2_TORQUE_GAIN     100    // PAS T (Toque)has a ratio of  100%
#endif

#ifndef PAS_3_TORQUE_GAIN
    #define PAS_3_TORQUE_GAIN     100    // PAS T (Toque) 3 has a ratio of  100%
#endif

#ifndef PAS_4_TORQUE_GAIN
    #define PAS_4_TORQUE_GAIN     100    // PAS T (Toque) 4 has a ratio of  100%
#endif

#ifndef PAS_5_TORQUE_GAIN
    #define PAS_5_TORQUE_GAIN     100    // PAS T (Toque) 5 has a ratio of 100%
#endif

#ifndef PAS_6_TORQUE_GAIN   
    #define PAS_6_TORQUE_GAIN     100    // PAS T (Toque) 6 has a ratio of 100%
#endif

#ifndef PAS_7_TORQUE_GAIN   
    #define PAS_7_TORQUE_GAIN     100    // PAS T (Toque) 7 has a ratio of 100%
#endif

#ifndef PAS_8_TORQUE_GAIN   
    #define PAS_8_TORQUE_GAIN     100    // PAS T (Toque) 8 has a ratio of 100%
#endif

#ifndef PAS_9_TORQUE_GAIN   
    #define PAS_9_TORQUE_GAIN     100    // PAS T (Toque) 9 has a ratio of 100%
#endif


//Max speed for PAS level on Torque or Cadence.

#ifndef PAS_LEVEL_SPEED_0
    #define PAS_LEVEL_SPEED_0       (uint8_t)0      /* Maximum Speed for PAS Level 0 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_1
    #define PAS_LEVEL_SPEED_1       (uint8_t)10     /* Maximum Speed for PAS Level 1 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_2
    #define PAS_LEVEL_SPEED_2       (uint8_t)15     /* Maximum Speed for PAS Level 2 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_3
    #define PAS_LEVEL_SPEED_3       (uint8_t)20     /* Maximum Speed for PAS Level 3 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_4
    #define PAS_LEVEL_SPEED_4       (uint8_t)25     /* Maximum Speed for PAS Level 4 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_5
    #define PAS_LEVEL_SPEED_5       (uint8_t)32     /* Maximum Speed for PAS Level 5 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_6
    #define PAS_LEVEL_SPEED_6       (uint8_t)32     /* Maximum Speed for PAS Level 6 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_7
    #define PAS_LEVEL_SPEED_7       (uint8_t)32     /* Maximum Speed for PAS Level 7 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_8
    #define PAS_LEVEL_SPEED_8       (uint8_t)32     /* Maximum Speed for PAS Level 8 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_9
    #define PAS_LEVEL_SPEED_9       (uint8_t)32     /* Maximum Speed for PAS Level 9 in Km/h */
#endif


#ifndef PAS_LEVEL_SPEED_WALK
    #define PAS_LEVEL_SPEED_WALK    (uint8_t)6     /* Maximum Speed for PAS Level walk in Km/h */
#endif


// PAS Max Torque power in % default values

#ifndef PAS_0_MAX_TORQUE_PERCENT
    #define PAS_0_MAX_TORQUE_PERCENT      0
#endif

#ifndef PAS_1_MAX_TORQUE_PERCENT
    #define PAS_1_MAX_TORQUE_PERCENT     20
#endif

#ifndef PAS_2_MAX_TORQUE_PERCENT
    #define PAS_2_MAX_TORQUE_PERCENT     40
#endif

#ifndef PAS_3_MAX_TORQUE_PERCENT
    #define PAS_3_MAX_TORQUE_PERCENT     60
#endif

#ifndef PAS_4_MAX_TORQUE_PERCENT
    #define PAS_4_MAX_TORQUE_PERCENT     80
#endif

#ifndef PAS_5_MAX_TORQUE_PERCENT
    #define PAS_5_MAX_TORQUE_PERCENT    100 
#endif

#ifndef PAS_6_MAX_TORQUE_PERCENT
    #define PAS_6_MAX_TORQUE_PERCENT      0
#endif

#ifndef PAS_7_MAX_TORQUE_PERCENT
    #define PAS_7_MAX_TORQUE_PERCENT      0
#endif

#ifndef PAS_8_MAX_TORQUE_PERCENT
    #define PAS_8_MAX_TORQUE_PERCENT      0 
#endif

#ifndef PAS_9_MAX_TORQUE_PERCENT
    #define PAS_9_MAX_TORQUE_PERCENT      0 
#endif

// PAS Min Torque power in % default values

#ifndef PAS_0_MIN_TORQUE_PERCENT
    #define PAS_0_MIN_TORQUE_PERCENT      0
#endif

#ifndef PAS_1_MIN_TORQUE_PERCENT
    #define PAS_1_MIN_TORQUE_PERCENT      0
#endif

#ifndef PAS_2_MIN_TORQUE_PERCENT
    #define PAS_2_MIN_TORQUE_PERCENT      0
#endif

#ifndef PAS_3_MIN_TORQUE_PERCENT
    #define PAS_3_MIN_TORQUE_PERCENT      0 
#endif

#ifndef PAS_4_MIN_TORQUE_PERCENT
    #define PAS_4_MIN_TORQUE_PERCENT      0
#endif

#ifndef PAS_5_MIN_TORQUE_PERCENT
    #define PAS_5_MIN_TORQUE_PERCENT      0 
#endif

#ifndef PAS_6_MIN_TORQUE_PERCENT
    #define PAS_6_MIN_TORQUE_PERCENT      0
#endif

#ifndef PAS_7_MIN_TORQUE_PERCENT
    #define PAS_7_MIN_TORQUE_PERCENT      0
#endif

#ifndef PAS_8_MIN_TORQUE_PERCENT
    #define PAS_8_MIN_TORQUE_PERCENT      0 
#endif

#ifndef PAS_9_MIN_TORQUE_PERCENT
    #define PAS_9_MIN_TORQUE_PERCENT      0 
#endif


// PAS Torque sensor parameters as default values

#ifndef PTS_FILTER_ALPHA
    #define PTS_FILTER_ALPHA              2.27F    // Butterworth alpha coefficient pedal torque sensor filtering
#endif

#ifndef PTS_FILTER_BETA
    #define PTS_FILTER_BETA              -0.27F    // Butterworth beta coefficient pedal torque sensor filtering
#endif

#ifndef PTS_MAX_PTSVALUE
    #define PTS_MAX_PTSVALUE         UINT16_MAX    // Maximum analog value to reach
#endif

#ifndef PTS_OFFSET_ADC2PTS
    #define PTS_OFFSET_ADC2PTS            10000    // Offset for ADC to pedal torque sensor linear transformation
#endif

#ifndef PTS_OFFSET_PTS2TORQUE_STARTUP
    #define PTS_OFFSET_PTS2TORQUE_STARTUP    40    // Offset for pedal torque sensor to torque linear transformation during the startup in %
#endif

#ifndef PTS_OFFSET_STARTUP_SPEED_KMH
    #define PTS_OFFSET_STARTUP_SPEED_KMH      3    // Speed under which the Startup pedal torque sensor offset is used in km/h
#endif

#ifndef PTS_OFFSET_PTS2TORQUE
    #define PTS_OFFSET_PTS2TORQUE            10    // Offset for pedal torque sensor to torque linear transformation in %
#endif

#ifndef PTS_OFFSET_PTS2TORQUE_SAFETY
    #define PTS_OFFSET_PTS2TORQUE_SAFETY     40    // Offset for pedal torque sensor to torque linear transformation that is considered safe in %
#endif

#ifndef PTS_SPEED_FILTER_1                   
    #define PTS_SPEED_FILTER_1                3
#endif
#ifndef PTS_SPEED_FILTER_2                   
    #define PTS_SPEED_FILTER_2                6
#endif

#ifndef PTS_FILTER_BW1_1
    #define PTS_FILTER_BW1_1                   10    // BW coefficient for pedal torque sensor avereging
#endif

#ifndef PTS_FILTER_BW2_1
    #define PTS_FILTER_BW2_1                   25    // BW coefficient for pedal torque sensor avereging
#endif
#ifndef PTS_FILTER_BW1_2
    #define PTS_FILTER_BW1_2                   10    // BW coefficient for pedal torque sensor avereging
#endif

#ifndef PTS_FILTER_BW2_2
    #define PTS_FILTER_BW2_2                   25    // BW coefficient for pedal torque sensor avereging
#endif
#ifndef PTS_FILTER_BW1_3
    #define PTS_FILTER_BW1_3                   10    // BW coefficient for pedal torque sensor avereging
#endif

#ifndef PTS_FILTER_BW2_3
    #define PTS_FILTER_BW2_3                   25    // BW coefficient for pedal torque sensor avereging
#endif

#ifndef TORQUE_SPEED_LIMIT_GAIN
    #define TORQUE_SPEED_LIMIT_GAIN 100
#endif

#ifndef REAR_LIGHT_BLINK_ON_BRAKE
    #define REAR_LIGHT_BLINK_ON_BRAKE     true // If set to true rear light will blink when brake is pressed
#endif

// Minimum Number of pulse, inside a specific time, to the detect PAS on cadence
#ifndef PEDALSPEEDSENSOR_MIN_PULSE_STARTUP
   #define PEDALSPEEDSENSOR_MIN_PULSE_STARTUP  6         
#endif
    
//cadence detection windows on ms when starting to use the pedal.
#ifndef PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS
    #define PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS 500           
#endif

// Minimum Number of pulse, inside a specific time, to the detect PAS on cadence when bike is running 
#ifndef PEDALSPEEDSENSOR_MIN_PULSE_RUNNING
    #define PEDALSPEEDSENSOR_MIN_PULSE_RUNNING  6      
#endif
    
//cadence detection windows on ms when running. 
#ifndef PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS
    #define PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS 150
#endif
    
#ifndef THROTTLE_BLOCK_OFF
    #define THROTTLE_BLOCK_OFF            false        //If set to true, the throttle will be blocked.
#endif
    
#ifndef PAS_OVER_THROTTLE
    #define PAS_OVER_THROTTLE             false
#endif

#ifndef THROTTLE_TOP_SPEED
    #define THROTTLE_TOP_SPEED VEHICLE_TOP_SPEED_KMH
#endif
    
#endif /*__VC_PARAMETERS_H*/

