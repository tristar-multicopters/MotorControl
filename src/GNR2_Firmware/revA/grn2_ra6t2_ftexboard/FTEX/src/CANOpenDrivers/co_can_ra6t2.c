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
*  "co_can_ra6t2.c"
*  Abstraction Layer module for CAN interface
*/

#include "co_can_ra6t2.h"

// ================================== Private Functions Prototypes ====================== //

/**
  @brief Function used to initialise the CAN driver
  @return void
*/
static void    CANo_DrvInit   (void);

/**
  @brief Function used to enable the CAN driver
  @param Receives CAN baud rate
  @return void
*/
static void    CANo_DrvEnable (uint32_t baudrate);

/**
  @brief Function used to Send a CAN message
  @param Receives CAN frame
  @return void
*/
static int16_t CANo_DrvSend   (CO_IF_FRM *frm);

/**
  @brief Function used to read the CAN bus
  @return 0
*/
static int16_t CANo_DrvRead   (CO_IF_FRM *frm);

/**
  @brief Function used to reset the CAN interface
  @return void
*/
static void    CANo_DrvReset  (void);

/**
  @brief Function used to close the CAN interface
  @return void
*/
static void    CANo_DrvClose  (void);

// ================================== Private variables ====================== //

uint32_t wTxBufferNum = CANFD_TX_MB_0;

// ================================== Public Variables ================================== //

const CO_IF_CAN_DRV CoCanDriver = {
    CANo_DrvInit,
    CANo_DrvEnable,
    CANo_DrvRead,
    CANo_DrvSend,
    CANo_DrvReset,
    CANo_DrvClose
};


//Filter list for which frame to accept
const canfd_afl_entry_t CANFD_FilterListArray[CANFD_CFG_AFL_CH0_RULE_NUM] =
{
    {
        .id =
        {
            // Specify the ID, ID type and frame type to accept.
            .id         = 0,
            .frame_type = CAN_FRAME_TYPE_DATA,
            .id_mode    = CAN_ID_MODE_STANDARD,
        },
        .mask =
        {
            // These values mask which ID/mode bits to compare when filtering messages. 
            .mask_id         = 0,
            .mask_frame_type = 1,
            .mask_id_mode    = 1,
        },
        .destination =
        {
            // If DLC checking is enabled any messages shorter than the below setting will be rejected. 
            .minimum_dlc = CANFD_MINIMUM_DLC_0,
            // Optionally specify a Receive Message Buffer (RX MB) to store accepted frames. RX MBs do not have an
            //  interrupt or overwrite protection and must be checked with R_CANFD_InfoGet and R_CANFD_Read. 
            .rx_buffer   = CANFD_RX_MB_0,
            // Specify which FIFO(s) to send filtered messages to. Multiple FIFOs can be OR'd together. 
            .fifo_select_flags = CANFD_RX_FIFO_0,
        }
    }
};

// ================================== Private Functions Definitions ================================== //

/**
  Function used to initialise the CAN interface
*/
static void CANo_DrvInit(void)
{
    R_CANFD_ModeTransition(&g_canfd0_ctrl,CAN_OPERATION_MODE_HALT, CAN_TEST_MODE_DISABLED);
    		
	//osDelay(10); //WTF WHY
}

/**
  Function used to enable the CAN interface
*/
static void CANo_DrvEnable(uint32_t baudrate)
{
	(void)baudrate;

	/* TODO: set the given baudrate to the CAN controller */
	R_CANFD_ModeTransition(&g_canfd0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);
}

/**
  Function used to Send messages through the CAN interface
*/
static int16_t CANo_DrvSend(CO_IF_FRM *frm)
{
	can_frame_t tx_frame =
	{
		.id = frm->Identifier,
		.id_mode = CAN_ID_MODE_STANDARD,
		.type = CAN_FRAME_TYPE_DATA,
		.data_length_code = frm->DLC
	};
	
	memcpy(tx_frame.data, frm->Data, 8);
	
    can_info_t CANInfo;
    R_CANFD_InfoGet(&g_canfd0_ctrl, &CANInfo);
    if (CANInfo.status & R_CANFD_CFDC_STS_TRMSTS_Msk) //If CAN driver is already transmitting something, use another TX buffer
	{
        if (wTxBufferNum == CANFD_TX_MB_3)
        {
            wTxBufferNum = CANFD_TX_MB_0;
        }
        else
        {
            wTxBufferNum++;
        }
    }
    R_CANFD_Write(&g_canfd0_ctrl, wTxBufferNum, &tx_frame);
	return (0u);
}

/**
  Function used to read messages through the CAN interface
*/
static int16_t CANo_DrvRead (CO_IF_FRM *frm)
{
    can_frame_t rx_frame;
    
    R_CANFD_Read(&g_canfd0_ctrl, CANFD_RX_BUFFER_MB_0, &rx_frame);
    
	frm->Identifier = rx_frame.id;
	frm->DLC = rx_frame.data_length_code;
	
	memcpy(frm->Data, rx_frame.data, 8);
	
	return (sizeof(CO_IF_FRM));
}

/**
  Function used to reset the CAN interface
*/
static void CANo_DrvReset(void)
{
  /* TODO: reset CAN controller while keeping baudrate */
	R_CANFD_ModeTransition(&g_canfd0_ctrl, CAN_OPERATION_MODE_RESET, CAN_TEST_MODE_DISABLED);
}

/**
  Function used to close the CAN interface
*/
static void CANo_DrvClose(void)
{
	/* TODO: remove CAN controller from CAN network */
    R_CANFD_ModeTransition(&g_canfd0_ctrl, CAN_OPERATION_MODE_RESET, CAN_TEST_MODE_DISABLED);
	R_CANFD_Close(&g_canfd0_ctrl);
}

