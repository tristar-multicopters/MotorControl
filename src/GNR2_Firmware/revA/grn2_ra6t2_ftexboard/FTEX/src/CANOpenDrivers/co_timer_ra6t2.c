/******************************************************************************
   Copyright 2020 Embedded Office GmbH & Co. KG

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
******************************************************************************/
/**
*  co_timer_ra6t2.c
*  Abstraction Layer module for Timer 1 used for CANOpen stack
*/

// ================================== INCLUDES ================================== //

#include "co_timer_ra6t2.h"

// ================================== PRIVATE FUNCTIONS ================================== //

/**
  @brief Function used to initialise the timer driver
         It initializes the timer, the counter and keeps timer stopped
  @param freq of timer in Hz
  @return void
*/
static void     CANo_TIM_Init   (uint32_t freq);

/**
  @brief Function used to start the hardware Timer 1 driver
  @return void
*/
static void     CANo_TIM_Start  (void);

/**
  @brief Function used to update the Timer 1 driver
  @return return 1 if timer event is elapsed, otherwise 0
*/
static uint8_t  CANo_TIM_Update (void);

/**
  @brief Function to return current Timer 1 counter value
  @return: Current timer counter value
*/
static uint32_t CANo_TIM_Delay  (void);

/**
  @brief Function to reload timer counter value
  @param:  reload value
  @return: void
*/
static void     CANo_TIM_Reload (uint32_t reload);

/**
  @brief Function to stop the counter value
  @return: void
*/
static void     CANo_TIM_Stop   (void);

// ================================== PUBLIC VARIABLES ================================== //

const CO_IF_TIMER_DRV CoTimerDriver = {
    CANo_TIM_Init,
    CANo_TIM_Reload,
    CANo_TIM_Delay,
    CANo_TIM_Stop,
    CANo_TIM_Start,
    CANo_TIM_Update
};

// ================================== PRIVATE VARIABLES ================================== //

uint32_t COTimerCounter = 0;
uint32_t COTimerPeriod = 0;

// ================================== PRIVATE FUNCTIONS ================================== //

/**
  Function used to initialise the timer
*/
static void CANo_TIM_Init(uint32_t freq)
{
    COTimerCounter = 0;
    // g_timer_a1_ctrl handle must be set to periodic mode prior to initialization. The period is then set below.
	R_AGT_Stop(&g_timer_a1_ctrl);
	R_AGT_Reset(&g_timer_a1_ctrl);
    R_AGT_PeriodSet(&g_timer_a1_ctrl, CO_HARDWARE_TIMER_FREQ_HZ/freq);
    R_AGT_Enable(&g_timer_a1_ctrl);
}

/**
  Function used to start the timer
*/
static void CANo_TIM_Start(void)
{
	R_AGT_Start(&g_timer_a1_ctrl);
}

/**
  Function used to update the timer
*/
static uint8_t CANo_TIM_Update(void)
{
	uint8_t elapsed = 0;
    
	if(COTimerCounter >= (COTimerPeriod-1))
	{
		elapsed = 1;
	}
	else
	{
		elapsed = 0;
	}

	return elapsed;
}

/**
  Function to return current timer counter value
*/
static uint32_t CANo_TIM_Delay(void)
{
	if(COTimerPeriod > COTimerCounter)
	{
		return (COTimerPeriod - COTimerCounter - 1);
	}
	else
	{
		return 0;
	}
}

/**
  Function to reload timer counter value
*/
static void CANo_TIM_Reload(uint32_t reload)
{
    COTimerPeriod = reload;
}

/**
  Function to Stop timer counter value
*/
static void CANo_TIM_Stop(void)
{
	R_AGT_Stop(&g_timer_a1_ctrl);
	R_AGT_Reset(&g_timer_a1_ctrl);
    COTimerCounter = 0;
}

// ================================== PUBLIC FUNCTIONS ================================== //

/**
  Function to manage the callback of the timer (defined on gnr_ra6t2_it.c)
*/
void COTimerCallback(CO_TMR *tmr)
{
    if(COTimerCounter >= (COTimerPeriod-1))
    {
        if(COTmrService(tmr))
        {
            osSemaphoreRelease(canTmrSemaphore);
        }
        COTimerCounter = 0;
	}
    else
    {
        COTimerCounter++;
    }
    
}
    