/**
  * @file    watchdog.c
  * @brief   This file contains the defines, includes and function prototypes needed to manage the watchdog
  *
  */

#include "gnr_main.h"

#ifndef __WATCHDOG
#define __WATCHDOG


#define REFRESH_DELAY_MS 500

/* Refresh register values */
//all value here came from the hardware used guide of the RAT6
//microcontroller.

//first value to be write in the wdt refresh register
#define WDT_REFRESH_STEP_1               (0U)
//second value to be write in the wdt refresh register
#define WDT_REFRESH_STEP_2               (0xFFU)

/**
 * @brief Manually refreshes the watchdog
 * @param void
 * @return void
 */
void Watchdog_Refresh(void);

#endif