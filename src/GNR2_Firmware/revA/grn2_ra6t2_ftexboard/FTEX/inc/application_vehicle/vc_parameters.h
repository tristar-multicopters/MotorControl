/**
  * @file    vc_parameters.h
	* @author	 Sami Bouzid, FTEX inc	 
  * @brief   This file contains the parameters needed in order to configure vehicle control layer.
  *
*/

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_H
#define __VC_PARAMETERS_H

#include "gnr_parameters.h"


#if VEHICLE_SELECTION == VEHICLE_E_CELLS

#include "vc_parameters_e_cells.h"

#elif VEHICLE_SELECTION == VEHICLE_R48_750W

#include "vc_parameters_r48_750w.h"

#elif VEHICLE_SELECTION == VEHICLE_TSUGAWA

#include "vc_parameters_tsugawa.h"

#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#include "vc_parameters_nidec.h"

#elif VEHICLE_SELECTION == VEHICLE_QUIETKAT

#include "vc_parameters_quietkat.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC_CITI_500W

#include "vc_parameters_velec_citi_500w.h"

#elif VEHICLE_SELECTION == VEHICLE_A2_350W 

#include "vc_parameters_a2_350w.h"

#elif VEHICLE_SELECTION == VEHICLE_UTK_350W 

#include "vc_parameters_utk_350w.h"

#elif VEHICLE_SELECTION == VEHICLE_A2_500W 

#include "vc_parameters_a2_500w.h"


#endif

// PAS Cadence sesnor Torque power in % default values

#ifndef PAS_C_6_POWER_PERCENT   // PAS C (Cadence) 6 has a ratio of 0%
    #define PAS_C_6_POWER_PERCENT 0   
#endif

#ifndef PAS_C_7_POWER_PERCENT   // PAS C (Cadence) 7 has a ratio of 0%
    #define PAS_C_7_POWER_PERCENT 0   
#endif

#ifndef PAS_C_8_POWER_PERCENT   // PAS C (Cadence) 8 has a ratio of 0%
    #define PAS_C_8_POWER_PERCENT 0   
#endif

#ifndef PAS_C_9_POWER_PERCENT   // PAS C (Cadence) 9 has a ratio of 0%
    #define PAS_C_9_POWER_PERCENT 0   
#endif

// PAS Torque sesnor Torque power in % default values

#ifndef PAS_T_0_POWER_PERCENT
    #define PAS_T_0_POWER_PERCENT      0
#endif

#ifndef PAS_T_1_POWER_PERCENT
    #define PAS_T_1_POWER_PERCENT     20
#endif

#ifndef PAS_T_2_POWER_PERCENT
    #define PAS_T_2_POWER_PERCENT     40
#endif

#ifndef PAS_T_3_POWER_PERCENT
    #define PAS_T_3_POWER_PERCENT     60
#endif

#ifndef PAS_T_4_POWER_PERCENT
    #define PAS_T_4_POWER_PERCENT     80
#endif

#ifndef PAS_T_5_POWER_PERCENT
    #define PAS_T_5_POWER_PERCENT    100 
#endif

#ifndef PAS_T_6_POWER_PERCENT
    #define PAS_T_6_POWER_PERCENT      0
#endif

#ifndef PAS_T_7_POWER_PERCENT
    #define PAS_T_7_POWER_PERCENT      0
#endif

#ifndef PAS_T_8_POWER_PERCENT
    #define PAS_T_8_POWER_PERCENT      0 
#endif

#ifndef PAS_T_9_POWER_PERCENT
    #define PAS_T_9_POWER_PERCENT      0 
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

#ifndef PTS_FILTER_BW1
    #define PTS_FILTER_BW1                   10    // BW coefficient for pedal torque sensor avereging
#endif

#ifndef PTS_FILTER_BW2
    #define PTS_FILTER_BW2                   25    // BW coefficient for pedal torque sensor avereging
#endif

#endif /*__VC_PARAMETERS_H*/

