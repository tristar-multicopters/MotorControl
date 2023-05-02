/**
  * @file    watchdog.c
  * @brief   This file contains the task managing the watchdog and watchdog wraper functions
  *
  */


#include "watchdog.h"


/**
  * @brief  Function used to Initialize the Watchdog
  */
bool WatchdogInit(void)
{
    uint8_t uIsError = false;
    uIsError |= R_WDT_Open(&g_wdt0_ctrl, &g_wdt0_cfg);
    
    ASSERT(FSP_SUCCESS == uIsError);
    
    return (bool) uIsError;
}


/**
  Manually refreshes the watchdog
 */
void Watchdog_Refresh(void)
{
    R_WDT_Refresh(&g_wdt0_ctrl);
}
