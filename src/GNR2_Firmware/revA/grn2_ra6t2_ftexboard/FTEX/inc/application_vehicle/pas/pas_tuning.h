/**
  * @file    pas_tuning.h
  * @brief   This file contains the parameters needed in order to configure the default PAS behaviour,
  *          according to the bike selected.
**/
#include "gnr_parameters.h"

// Flag used to detect PAS with cadence AND torque.
// 0: Cadence OR Torque
// 1: Cadence AND Torque
#define DEFAULT_CADENCE_AND_TORQUE                  0 

#if VEHICLE_SELECTION == VEHICLE_A2_350W
    #include "pas_tuning_a2_350w.h"
#elif VEHICULE_SELECTION == VEHICLE_A2_500W
    #include "pas_tuning_a2_500w.h"
#elif VEHICULE_SELECTION == VEHICLE_E_CELLS
    #include "pas_tuning_e_cells.h"
#elif VEHICULE_SELECTION == VEHICLE_MILEBOX
    #include "pas_tuning_milebox.h"
#elif VEHICULE_SELECTION == VEHICLE_NIDEC
    #include "pas_tuning_nidec.h"
#elif VEHICULE_SELECTION == VEHICLE_PEGATRON
    #include "pas_tuning_pegatron.h"
#elif VEHICULE_SELECTION == VEHICLE_QUIETKAT
    #include "pas_tuning_quietkat.h"
#elif VEHICULE_SELECTION == VEHICLE_R48_750W
    #include "pas_tuning_r48_750w.h"
#elif VEHICULE_SELECTION == VEHICLE_SUPER73_S
    #include "pas_tuning_super73_s.h"
#elif VEHICULE_SELECTION == VEHICLE_SUPER73_Z
    #include "pas_tuning_super73_z.h"
#elif VEHICULE_SELECTION == VEHICLE_VELEC21_500W
    #include "pas_tuning_vel21_500w.h"
#elif VEHICULE_SELECTION == VEHICLE_VELEC_CITI_500W
    #include "pas_tuning_velec_citi_500w.h"

// If no bike is defined, we choose the default speed threshold values
#else
    #define RUNTIME_PAS_SPEED_THRESHOLD       (float)5          // Speed threshold to enable startup PAS power in km/h 
    #define STARTUP_PAS_SPEED_THRESHOLD       (float)1          // Speed threshold to enable runtime PAS power in km/h
    #define TORQUE_STARTUP_VALUE_THRESHOLD    (uint16_t)80      // Torque value (%) that needs to be provided to have a startup detection
    #define STARTUP_PULSE_NUMBER              (uint32_t)5       // Number of pulses that needs to be detected to trigger startup detection
    #define STARTUP_TIME_WINDOW               (uint16_t)1000    // Time window (ms) in which the startup pulse number is counted
    #define RUNTIME_PULSE_NUMBER              (uint32_t)1       // Number of pulses that needs to be detected to trigger runtime detection
    #define RUNTIME_TIME_WINDOW               (uint16_t)150     // Time window (ms) in which the runtime pulse number is counted
#endif
