/**
  * @file    pas_tuning_r48_750w.h
  * @brief   This file contains the parameters needed in order to configure the default PAS behaviour,
  *          on the R48 750W Bike.
**/

#define STARTUP_PAS_SPEED_THRESHOLD         (float)5         // Speed threshold to enable startup PAS power in km/h
#define RUNTIME_PAS_SPEED_THRESHOLD         (float)1          // Speed threshold to enable runtime PAS power in km/h
#define TORQUE_STARTUP_VALUE_THRESHOLD      (uint16_t) 10     // Torque value (%) that needs to be provided to have a startup detection
#define STARTUP_PULSE_NUMBER                (uint32_t) 1      // Number of pulses that needs to be detected to trigger startup detection
#define STARTUP_TIME_WINDOW                 (uint16_t) 1000   // Time window (ms) in which the startup pulse number is counted
#define RUNTIME_PULSE_NUMBER                (uint32_t) 1      // Number of pulses that needs to be detected to trigger runtime detection           
#define RUNTIME_TIME_WINDOW                 (uint16_t) 150    // Time window (ms) in which the runtime pulse number is counted