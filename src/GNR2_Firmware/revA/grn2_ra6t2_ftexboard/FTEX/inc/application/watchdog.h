/**
  * @file    watchdog.c
  * @brief   This file contains the defines, includes and function prototypes needed to manage the watchdog
  *
  */

#include "gnr_main.h"


#ifndef __WATCHDOG
#define __WATCHDOG


#define REFRESH_DELAY_MS 500

/**
  * @brief  Function used to Initialize the Watchdog
  */
bool WatchdogInit(void);

/**
 * @brief Manually refreshes the watchdog
 * @param void
 * @return void
 */
void Watchdog_Refresh(void);

#endif