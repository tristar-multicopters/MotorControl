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

#elif VEHICLE_SELECTION ==  VEHICLE_SUPER73

#include "vc_parameters_super73.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC_CITI_500W

#include "vc_parameters_velec_citi_500w.h"

#elif VEHICLE_SELECTION == VEHICLE_MILEBOX

#include "vc_parameters_milebox.h"

#endif

#ifndef  VEHICLE_TOP_SPEED_KMH
    #define VEHICLE_TOP_SPEED_KMH   32    
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
    #define PAS_LEVEL_SPEED_1       (uint8_t)32     /* Maximum Speed for PAS Level 1 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_2
    #define PAS_LEVEL_SPEED_2       (uint8_t)32     /* Maximum Speed for PAS Level 2 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_3
    #define PAS_LEVEL_SPEED_3       (uint8_t)32     /* Maximum Speed for PAS Level 3 in Km/h */
#endif

#ifndef PAS_LEVEL_SPEED_4
    #define PAS_LEVEL_SPEED_4       (uint8_t)32     /* Maximum Speed for PAS Level 4 in Km/h */
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


// PAS Torque sesnor parameters as default values

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

//PAS ramps default values

#ifndef PAS1_ACCEL_RAMP_TYPE
    #define PAS1_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS1_ACCEL_RAMP_ARG1
    #define PAS1_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS2_ACCEL_RAMP_TYPE
    #define PAS2_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS2_ACCEL_RAMP_ARG1
    #define PAS2_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS3_ACCEL_RAMP_TYPE
    #define PAS3_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS3_ACCEL_RAMP_ARG1
    #define PAS3_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS4_ACCEL_RAMP_TYPE
    #define PAS4_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS4_ACCEL_RAMP_ARG1
    #define PAS4_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS5_ACCEL_RAMP_TYPE
    #define PAS5_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS5_ACCEL_RAMP_ARG1
    #define PAS5_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS6_ACCEL_RAMP_TYPE
    #define PAS6_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS6_ACCEL_RAMP_ARG1
    #define PAS6_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS7_ACCEL_RAMP_TYPE
    #define PAS7_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS7_ACCEL_RAMP_ARG1
    #define PAS7_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS8_ACCEL_RAMP_TYPE
    #define PAS8_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS8_ACCEL_RAMP_ARG1
    #define PAS8_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS9_ACCEL_RAMP_TYPE
    #define PAS9_ACCEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS9_ACCEL_RAMP_ARG1
    #define PAS9_ACCEL_RAMP_ARG1                    0    
#endif

#ifndef PAS1_DECEL_RAMP_TYPE
    #define PAS1_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS1_DECEL_RAMP_ARG1
    #define PAS1_DECEL_RAMP_ARG1                    0    
#endif

#ifndef PAS2_DECEL_RAMP_TYPE
    #define PAS2_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS2_DECEL_RAMP_ARG1
    #define PAS2_DECEL_RAMP_ARG1                    0    
#endif

#ifndef PAS3_DECEL_RAMP_TYPE
    #define PAS3_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS3_DECEL_RAMP_ARG1
    #define PAS3_DECEL_RAMP_ARG1                    0    
#endif

#ifndef PAS4_DECEL_RAMP_TYPE
    #define PAS4_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS4_DECEL_RAMP_ARG1
    #define PAS4_DECEL_RAMP_ARG1                    0    
#endif

#ifndef PAS5_DECEL_RAMP_TYPE
    #define PAS5_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS5_DECEL_RAMP_ARG1
    #define PAS5_DECEL_RAMP_ARG1                    0    
#endif

#ifndef PAS6_DECEL_RAMP_TYPE
    #define PAS6_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS6_DECEL_RAMP_ARG1
    #define PAS6_DECEL_RAMP_ARG1                    0    
#endif

#ifndef PAS7_DECEL_RAMP_TYPE
    #define PAS7_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS7_DECEL_RAMP_ARG1
    #define PAS7_DECEL_RAMP_ARG1                    0    
#endif

#ifndef PAS8_DECEL_RAMP_TYPE
    #define PAS8_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS8_DECEL_RAMP_ARG1
    #define PAS8_DECEL_RAMP_ARG1                    0    
#endif

#ifndef PAS9_DECEL_RAMP_TYPE
    #define PAS9_DECEL_RAMP_TYPE              NO_RAMP    
#endif

#ifndef PAS9_DECEL_RAMP_ARG1
    #define PAS9_DECEL_RAMP_ARG1                    0    
#endif


#ifndef WALKMODE_ACCEL_RAMP_TYPE
    #define WALKMODE_ACCEL_RAMP_TYPE          NO_RAMP    
#endif

#ifndef WALKMODE_ACCEL_RAMP_ARG1
    #define WALKMODE_ACCEL_RAMP_ARG1                0    
#endif

#ifndef THROTTLE_ACCEL_RAMP_TYPE
    #define THROTTLE_ACCEL_RAMP_TYPE          NO_RAMP    
#endif

#ifndef THROTTLE_ACCEL_RAMP_ARG1
    #define THROTTLE_ACCEL_RAMP_ARG1                0    
#endif

#ifndef TORQUE_SPEED_LIMIT_GAIN
    #define TORQUE_SPEED_LIMIT_GAIN 100
#endif

#ifndef REAR_LIGHT_BLINK_ON_BRAKE
    #define REAR_LIGHT_BLINK_ON_BRAKE     true // If set to true rear light will blink when brake is pressed
#endif
    
//cadence detection windows on ms when starting to use the pedal.
#ifndef PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS
    #define PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS 150
    
#endif    
//cadence detection windows on ms when running. 
#ifndef PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS
    #define PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS 150
#endif
    
#ifndef THROTTLE_BLOCK_OFF
    #define THROTTLE_BLOCK_OFF            true        //If set to true, the throttle will be blocked.
#endif
    
#endif /*__VC_PARAMETERS_H*/

