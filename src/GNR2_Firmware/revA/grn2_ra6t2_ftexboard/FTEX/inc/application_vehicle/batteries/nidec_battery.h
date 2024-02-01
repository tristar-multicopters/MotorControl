#ifndef __NIDEC_BATTERY_H
#define __NIDEC_BATTERY_H

/******************************** BATTERY MONITORING PARAMETERS ******************************/

#define BATTERY_FULL_VOLT_X_100   4000
#define BATTERY_EMPTY_VOLT_X_100  3200

#define BATTERY_SOC_LOW_PERCENT      5   // Battery SOC in % for which we set the battery low flag (stops powertrain form pushing power)
#define BATTERY_SOC_OK_PERCENT      10   // Battery SOC in % for which we clear the battery low flag

#endif