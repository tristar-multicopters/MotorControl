#ifndef __QUIETKAT_BATTERY_H
#define __QUIETKAT_BATTERY_H

// Chosen battery for Quietkat

/******************************** BATTERY MONITORING PARAMETERS ******************************/

#define BATTERY_FULL_VOLT_X_100   5300
#define BATTERY_EMPTY_VOLT_X_100  3800

#define BATTERY_SOC_LOW_PERCENT      5    // Battery SOC in % for which we set the battery low flag (stops powertrain form pushing power)
#define BATTERY_SOC_OK_PERCENT      25    // Battery SOC in % for which we clear the battery low flag


/******************************** BATTERY POWER PARAMETERS ******************************/

#define POWER_LIMIT_REF                     MAX_CURRENT_LIMIT   // defines if the code should use MAX_APPLICATION_POSITIVE_POWER or MAX_APPLICATION_CURRENT
#define MAX_APPLICATION_POSITIVE_POWER      750                 // Refers to maximum power in watts that drive can push to the motor
#define MAX_APPLICATION_NEGATIVE_POWER      750                 // Refers to maximum power in watts that drive can accept from the motor
#define MAX_APPLICATION_CURRENT             15                  // Refers to maximum battery current in amps that drive can accept from the motor

/******************************** BATTERY  PARAMETERS ******************************/

#define UD_VOLTAGE_THRESHOLD_BATT_V             36                   // Under-voltage threshold to prevent BMS shutdown


#endif