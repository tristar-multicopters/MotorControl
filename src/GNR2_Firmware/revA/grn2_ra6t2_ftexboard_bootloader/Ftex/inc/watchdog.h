/**
  * @file    watchdog.c
  * @brief   This file contains the defines, includes and function prototypes needed to manage the watchdog
  *
  */


#ifndef __WATCHDOG
#define __WATCHDOG

#include "hal_data.h"

#define REFRESH_DELAY_MS 500

#define STABILIZATION_CLOCK_TIME_TO_WDT_MS  200

/**
  * @brief  Function used to Initialize the Watchdog
  * @param void
  * @return bool return 0 is no error.
  */
bool WatchdogInit(void);

/**
 * @brief Manually refreshes the watchdog
 * @param void
 * @return void
 */
void Watchdog_Refresh(void);

#endif