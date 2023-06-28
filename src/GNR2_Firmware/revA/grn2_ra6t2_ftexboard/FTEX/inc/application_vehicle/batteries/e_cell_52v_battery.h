#ifndef __E_CELL_52V_BATTERY_H
#define __E_CELL_52V_BATTERY_H

/******************************** BATTERY MONITORING PARAMETERS ******************************/

#define BATTERY_FULL_VOLT           52
#define BATTERY_EMPTY_VOLT          46

#define BATTERY_SOC_LOW_PERCENT     5   // Battery SOC in % for which we set the battery low flag (stops powertrain form pushing power)
#define BATTERY_SOC_OK_PERCENT      25  // Battery SOC in % for which we clear the battery low flag

#endif