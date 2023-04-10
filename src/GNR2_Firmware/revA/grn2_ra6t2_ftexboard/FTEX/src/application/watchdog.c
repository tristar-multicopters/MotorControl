/**
  * @file    watchdog.c
  * @brief   This file contains the task managing the watchdog and watchdog wraper functions
  *
  */


#include "watchdog.h"
#include "vc_errors_management.h"

extern osThreadId_t Watchdog_handle;


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
  Task that periodically refreshs the watchdog
  (very low priority task)
 */
__NO_RETURN void Watchdog (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter); 
    uint32_t WatchdogCount;
    
    
    while(1)
    {
        //Read the current count value of the WDT
        ASSERT(R_WDT_CounterGet(&g_wdt0_ctrl,&WatchdogCount) == FSP_SUCCESS);

        //used to refresh the wdt count. avoid wdt reset
        //wdt was configured to reset in 2.2 seconds.
        //must to be refreshed before 2.2 seconds.
        R_WDT_Refresh(&g_wdt0_ctrl);
        
        osDelay(REFRESH_DELAY_MS); // Delay between refresh of watchdog
    }   
}

/**
  Manually refreshes the watchdog
 */
void Watchdog_Refresh(void)
{
    R_WDT_Refresh(&g_wdt0_ctrl);
}
