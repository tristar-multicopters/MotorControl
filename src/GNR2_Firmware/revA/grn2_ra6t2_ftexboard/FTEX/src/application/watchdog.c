/**
  * @file    watchdog.c
  * @brief   This file contains the task managing the watchdog and watchdog wraper functions
  *
  */


#include "watchdog.h"
#include "vc_errors_management.h"

extern osThreadId_t Watchdog_handle;


/**
  Task that periodically refreshs the watchdog
  (very low priority task)
 */
__NO_RETURN void Watchdog (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter); 

    while(1)
    {
        //enable wdt kick to avoid wdt reset
        #if FIRMWARE_RELEASE
        //used to refresh the wdt count. avoid wdt reset
        //wdt was configured to reset in 2.2 seconds.
        //must to be refreshed before 2.2 seconds.
        //this must be uncommented when bootloader+main will be 
        //flashed.
     //   Watchdog_Refresh();
        #endif
        
        osDelay(REFRESH_DELAY_MS); // Delay between refresh of watchdog
    }   
}

/**
  Manually refreshes the watchdog
 */
void Watchdog_Refresh(void)
{
    R_WDT->WDTRR = WDT_REFRESH_STEP_1;
    R_WDT->WDTRR = WDT_REFRESH_STEP_2;
}
