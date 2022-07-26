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
* @file   co_can_ra6t2.h
* @author Ftex
* @brief  uController Abstraction Layer for CAN
*
* This module is used to interact with the CANFD interface. 
* It is the bridge between the CAN interface and the CANOpen stack
*
*/
#ifndef CO_CAN_RA6T2_H
#define CO_CAN_RA6T2_H

#ifdef __cplusplus               /* for compatibility with C++ environments  */
extern "C" {
#endif

/******************************************************************************
* INCLUDES
******************************************************************************/

#include "co_if.h"
#include "hal_data.h"
#include "co_core.h"
#include "gnr_parameters.h" // To be able to use ENABLE_CAN_LOGGER flag

#include <cmsis_os2.h>

#define CAN_GNR2_ID     0x601
#define CAN_OK          0x00
#define CAN_FAIL        0x01
/******************************************************************************
* PUBLIC SYMBOLS
******************************************************************************/

typedef struct
{
    CO_NODE canNode;
    uint16_t hError;
    bool bTxFlag;
}CAN_Handler_t;

#if ENABLE_CAN_LOGGER
void CANo_DrvInit(void);
uint8_t CAN_SendMsg(can_frame_t msg_to_send);
#else
/**
*  @brief Function used to process a message received
*         from callback function of the CANbus
*  @param pHandler: Handle of the CAN interface
*  @param rxFrame: frame received from the CAN callback function
*  @return void
*/
void uCAL_CAN_ProcessRxMessage(CAN_Handler_t * pHandler, can_frame_t rxFrame);
#endif
/* TODO: rename the extern variable declaration to match the naming convention:
 *   <device-name>CanDriver
 */
extern const CO_IF_CAN_DRV CoCanDriver;

#ifdef __cplusplus               /* for compatibility with C++ environments  */
}
#endif

#endif
