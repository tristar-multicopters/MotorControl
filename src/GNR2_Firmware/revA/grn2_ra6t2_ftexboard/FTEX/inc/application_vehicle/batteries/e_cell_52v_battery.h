#ifndef __E_CELL_52V_BATTERY_H
#define __E_CELL_52V_BATTERY_H

/******************************** BATTERY MONITORING PARAMETERS ******************************/

#define BATTERY_FULL_VOLT_X_100   5200
#define BATTERY_EMPTY_VOLT_X_100  3900

#define BATTERY_SOC_LOW_PERCENT      5   // Battery SOC in % for which we set the battery low flag (stops powertrain form pushing power)
#define BATTERY_SOC_OK_PERCENT      25   // Battery SOC in % for which we clear the battery low flag

/******************************** BATTERY POWER PARAMETERS ******************************/

#define POWER_LIMIT_REF                     MAX_CURRENT_LIMIT // Defines if the code should use MAX_APPLICATION_POSITIVE_POWER or MAX_APPLICATION_CURRENT
#define MAX_APPLICATION_POSITIVE_POWER      1100            // Refers to maximum power in watts that drive can push to the motor
#define MAX_APPLICATION_NEGATIVE_POWER      1100 // Refers to maximum power in watts that drive can accept from the motor
#define MAX_APPLICATION_CURRENT             23              // Refers to maximum battery current in amps that drive can accept from the motor

#define ENABLE_LV_TORQUE_LIMIT              true            // Enable or disable the low voltage torque limit
#define LOW_VOLTAGE_THRESHOLD_PERCENTAGE    10              // The threshold percentage of battery voltage before limiting torque
#define LOW_BATTERY_TORQUE                  150

#define ENABLE_MAX_POWER_LIMIT              true            // To enable or disable the foldback
#define MAX_TIME_BMS_TOLERANT               20000           // End time of derating for BMS protection in ms
#define MAX_POWER_LIMIT_TIMEOUT             10000           // Start time of derating for BMS protection in ms
#define MAX_BMS_POSITIVE_POWER              750             // Maximum Power at the end point of foldback
#define MAX_BMS_CONTINUOUS_CURRENT          15              // Maximum Power at the end point of foldback in amps

/******************************** BATTERY  PARAMETERS ******************************/

#define UD_VOLTAGE_THRESHOLD_BATT_V             36                   // Under-voltage threshold to prevent BMS shutdown


#endif
