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

// ================================== INCLUDES ================================== //

#include "co_timer_ra6t2.h"
#include "co_core.h"

#include "hal_data.h"
#include <cmsis_os2.h>

/* get external CANopen node */
extern CO_NODE GnR2Module;
extern osSemaphoreId_t canTmrSemaphore;

// ================================== PRIVATE FUNCTIONS ================================== //

void		    g_co_TIM_callback(	timer_callback_args_t * );
static void     DrvTimerInit   (uint32_t freq);
static void     DrvTimerStart  (void);
static uint8_t  DrvTimerUpdate (void);
static uint32_t DrvTimerDelay  (void);
static void     DrvTimerReload (uint32_t reload);
static void     DrvTimerStop   (void);
// ================================== PUBLIC VARIABLES ================================== //

/* TODO: rename the variable to match the naming convention:
 *   <device>TimerDriver
 */
const CO_IF_TIMER_DRV CoTimerDriver = {
    DrvTimerInit,
    DrvTimerReload,
    DrvTimerDelay,
    DrvTimerStop,
    DrvTimerStart,
    DrvTimerUpdate
};

// ================================== PRIVATE FUNCTIONS ================================== //
uint32_t m_counter = 0;
uint32_t m_period = 0;

void g_co_TIM_callback(	timer_callback_args_t * p_args )
{
	if( p_args->event == TIMER_EVENT_CYCLE_END)
	{
		/* collect elapsed timed actions */
		if(m_counter >= m_period)
		{
			if(COTmrService(&GnR2Module.Tmr))
			{
				osSemaphoreRelease(canTmrSemaphore);
			}
			m_counter = 0;
		}
		else
		{
			m_counter++;
		}
	}
}

static void DrvTimerInit(uint32_t freq)
{
	(void)freq;
	/* TODO: initialize timer, clear counter and keep timer stopped */
	m_counter = 0;
	R_AGT_Open(&g_timer1_ctrl, &g_timer1_cfg);
	R_AGT_Reset( &g_timer1_ctrl );
	R_AGT_Stop( &g_timer1_ctrl );
}

static void DrvTimerStart(void)
{
	/* TODO: start hardware timer */
	R_AGT_Start(&g_timer1_ctrl);
}

static uint8_t DrvTimerUpdate(void)
{
	/* TODO: return 1 if timer event is elapsed, otherwise 0 */
	uint8_t elapsed = 0;
	
	if(m_counter >= m_period)
	{
		elapsed = 1;
	}
	else
	{
		elapsed = 0;
	}
	
	return elapsed;
}

static uint32_t DrvTimerDelay(void)
{
  /* TODO: return current timer counter value */
	if(m_period > m_counter)
	{
		return (m_period - m_counter - 1);
	}
	else
	{
		return 0;
	}
}

static void DrvTimerReload(uint32_t reload)
{
	(void)reload;

	/* TODO: reload timer counter value with given reload value */
	m_period = reload;
}

static void DrvTimerStop(void)
{
	/* TODO: start hardware timer and clear counter value */
	R_AGT_Stop( &g_timer1_ctrl );
	R_AGT_Reset( &g_timer1_ctrl );
	m_counter = 0;
}
