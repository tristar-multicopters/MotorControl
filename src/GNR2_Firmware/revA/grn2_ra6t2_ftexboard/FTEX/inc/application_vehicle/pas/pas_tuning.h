/**
  * @file    pas_tuning.h
  * @brief   This file contains the parameters needed in order to configure the default PAS behaviour,
  *          according to the bike selected.
**/
#include "gnr_parameters.h"

#define PAS_DEFAULT               0
#define PAS_R48_750W              1
#define PAS_A2_350W               2
#define PAS_A2_500W               3
#define PAS_QUIETKAT              4
#define PAS_PEGATRON              5
#define PAS_E_CELLS               6
#define PAS_NIDEC                 7   
#define PAS_VELEC_CITI_500W       8
#define PAS_SUPER73_Z             9
#define PAS_SUPER73_R            10
#define PAS_MILEBOX              11
#define PAS_VELEC21_500W         12

#define PAS_SELECTION            PAS_R48_750W

// Flag used to detect PAS with cadence AND torque.
// 0: Cadence OR Torque
// 1: Cadence AND Torque
#define CADENCE_AND_OR_TORQUE     1 

#if PAS_SELECTION == PAS_A2_350W
    #include "pas_tuning_a2_350w.h"
#endif

#if PAS_SELECTION == PAS_A2_500W
    #include "pas_tuning_a2_500w.h"
#endif

#if PAS_SELECTION == PAS_E_CELLS
    #include "pas_tuning_e_cells.h"
#endif

#if PAS_SELECTION == PAS_MILEBOX
    #include "pas_tuning_milebox.h"
#endif

#if PAS_SELECTION == PAS_NIDEC
    #include "pas_tuning_nidec.h"
#endif

#if PAS_SELECTION == PAS_PEGATRON
    #include "pas_tuning_pegatron.h"
#endif

#if PAS_SELECTION == PAS_QUIETKAT
    #include "pas_tuning_quietkat.h"
#endif

#if PAS_SELECTION == VEHICLE_R48_750W
    #include "pas_tuning_r48_750w.h"
#endif

#if PAS_SELECTION == PAS_SUPER73_R
    #include "pas_tuning_super73_s.h"
#endif

#if PAS_SELECTION == PAS_SUPER73_Z
    #include "pas_tuning_super73_z.h"
#endif

#if PAS_SELECTION == PAS_VELEC21_500W
    #include "pas_tuning_vel21_500w.h"
#endif

#if PAS_SELECTION == PAS_VELEC_CITI_500W
    #include "pas_tuning_velec_citi_500w.h"
#endif

// If no bike is defined, we choose the default speed threshold values
#if PAS_SELECTION == PAS_DEFAULT
    #define RUNTIME_PAS_SPEED_THRESHOLD       (float)5          // Speed threshold to enable startup PAS power in km/h 
    #define STARTUP_PAS_SPEED_THRESHOLD       (float)1          // Speed threshold to enable runtime PAS power in km/h
    #define TORQUE_STARTUP_VALUE_THRESHOLD    (uint16_t)80      // Torque value (%) that needs to be provided to have a startup detection
    #define STARTUP_PULSE_NUMBER              (uint32_t)5       // Number of pulses that needs to be detected to trigger startup detection
    #define STARTUP_TIME_WINDOW               (uint16_t)1000    // Time window (ms) in which the startup pulse number is counted
    #define RUNTIME_PULSE_NUMBER              (uint32_t)1       // Number of pulses that needs to be detected to trigger runtime detection
    #define RUNTIME_TIME_WINDOW               (uint16_t)150     // Time window (ms) in which the runtime pulse number is counted
#endif
