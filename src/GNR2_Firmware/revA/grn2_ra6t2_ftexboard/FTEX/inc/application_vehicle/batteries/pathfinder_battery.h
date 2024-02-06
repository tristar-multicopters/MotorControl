#ifndef __QUIETKAT_BATTERY_H
#define __QUIETKAT_BATTERY_H

// Chosen battery for Quietkat

/******************************** BATTERY MONITORING PARAMETERS ******************************/

#define BATTERY_FULL_VOLT_X_100   5300
#define BATTERY_EMPTY_VOLT_X_100  3800

#define BATTERY_SOC_LOW_PERCENT      5    // Battery SOC in % for which we set the battery low flag (stops powertrain form pushing power)
#define BATTERY_SOC_OK_PERCENT      25    // Battery SOC in % for which we clear the battery low flag

#endif