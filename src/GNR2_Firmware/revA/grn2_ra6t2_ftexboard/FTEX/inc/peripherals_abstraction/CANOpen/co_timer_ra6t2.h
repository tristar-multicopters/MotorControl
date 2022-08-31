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
* @file   co_timer_ra6t2.h
* @author FTEX inc
* @brief  uController Abstraction Layer for hardware timer used by CANOpen stack
*
* This module is used to interact with the timer
* It is the bridge between the timer interface and the CANOpen stack
*
*/
#ifndef CO_TIMER_RA6T2_H
#define CO_TIMER_RA6T2_H

#ifdef __cplusplus               /* for compatibility with C++ environments  */
extern "C" {
#endif

// ================================== INCLUDES ================================== //
#include "co_if.h"
#include "co_core.h"
#include "hal_data.h"
#include <cmsis_os2.h>
#include "ASSERT_FTEX.h"

// ================================== DEFINES ================================== //
#define CO_HARDWARE_TIMER_FREQ_HZ    60000000  // Hardware timer frequency chosen in RASC configurator

// ================================== PUBLIC SYMBOLS ============================ //
void COTimerCallback(CO_TMR *tmr);
/* Get external entities */
extern const CO_IF_TIMER_DRV CoTimerDriver;
extern osSemaphoreId_t canTmrSemaphore;

#ifdef __cplusplus               /* for compatibility with C++ environments  */
}
#endif

#endif
