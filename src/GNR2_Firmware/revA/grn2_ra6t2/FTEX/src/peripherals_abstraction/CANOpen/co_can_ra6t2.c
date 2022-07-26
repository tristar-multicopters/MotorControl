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
#if !ENABLE_CAN_LOGGER

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

// ================================== Private variables Prototypes ====================== //
static can_frame_t mRxCanFrame;

// ================================== Public Variables ================================== //

const CO_IF_CAN_DRV CoCanDriver = {
    CANo_DrvInit,
    CANo_DrvEnable,
    CANo_DrvRead,
    CANo_DrvSend,
    CANo_DrvReset,
    CANo_DrvClose
};


#endif


//Filter list for which frame to accept
const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM] =
{
    {
        .id =
        {
            // Specify the ID, ID type and frame type to accept.
            .id         = CAN_GNR2_ID,
            .frame_type = CAN_FRAME_TYPE_DATA,
            .id_mode    = CAN_ID_MODE_STANDARD,
        },
        .mask =
        {
            // These values mask which ID/mode bits to compare when filtering messages. 
            .mask_id         = 0x1FFFFFFF,
            .mask_frame_type = 1,
            .mask_id_mode    = 1,
        },
        .destination =
        {
            // If DLC checking is enabled any messages shorter than the below setting will be rejected. 
            .minimum_dlc = CANFD_MINIMUM_DLC_0,
            // Optionally specify a Receive Message Buffer (RX MB) to store accepted frames. RX MBs do not have an
            //  interrupt or overwrite protection and must be checked with R_CANFD_InfoGet and R_CANFD_Read. 
            .rx_buffer   = CANFD_RX_MB_NONE,
            // Specify which FIFO(s) to send filtered messages to. Multiple FIFOs can be OR'd together. 
            .fifo_select_flags = CANFD_RX_FIFO_0,
        }
    }
};

// ================================== Private Functions Definitions ================================== //

#if !ENABLE_CAN_LOGGER

/**
  Function used to initialise the CAN interface
*/
static void CANo_DrvInit(void)
{

    //Use the external loopback mode for initial testing
    R_CANFD_ModeTransition(&g_canfd0_ctrl,CAN_OPERATION_MODE_HALT,CAN_TEST_MODE_DISABLED);
    		
	osDelay(10);
}

/**
  Function used to enable the CAN interface
*/
static void CANo_DrvEnable(uint32_t baudrate)
{
	(void)baudrate;

	/* TODO: set the given baudrate to the CAN controller */
	R_CANFD_ModeTransition(&g_canfd0_ctrl,CAN_OPERATION_MODE_NORMAL,CAN_TEST_MODE_DISABLED);
}

/**
  Function used to Send messages through the CAN interface
*/
static int16_t CANo_DrvSend(CO_IF_FRM *frm)
{
	(void)frm;
	uint32_t mailbox = CANFD_TX_MB_0;
	can_frame_t c_frame =
	{
		.id = frm->Identifier,
		.id_mode = CAN_ID_MODE_STANDARD,
		.type = CAN_FRAME_TYPE_DATA,
		.data_length_code = frm->DLC
	};
	
	memcpy(c_frame.data, frm->Data, 8);
	
	/* TODO: wait for free CAN message slot and send the given CAN frame */
	R_CANFD_Write(&g_canfd0_ctrl, mailbox, &c_frame);
	return (0u);
}

/**
  Function used to read messages through the CAN interface
*/
static int16_t CANo_DrvRead (CO_IF_FRM *frm)
{
	(void)frm;
		
	frm->Identifier = mRxCanFrame.id;
	frm->DLC = mRxCanFrame.data_length_code;
	
	memcpy(frm->Data, mRxCanFrame.data, 8);
	
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
	R_CANFD_Close	(	&g_canfd0_ctrl );
}

// ================================== Public Functions (for CAN Logger only) ================================== //

/**
*  Function used to process a message received 
*  from callback function of the CANbus
*/
void uCAL_CAN_ProcessRxMessage(CAN_Handler_t * pHandler, can_frame_t rxFrame)
{
    mRxCanFrame = rxFrame;
    CONodeProcess(&pHandler->canNode);
}

#else
/**
  Function used to initialise the CAN interface (used only for CAN logger)
*/
void CANo_DrvInit(void)
{
    //Use the external loopback mode for initial testing
    R_CANFD_ModeTransition(&g_canfd0_ctrl,CAN_OPERATION_MODE_NORMAL,CAN_TEST_MODE_DISABLED);
}

/**
  Function used to send a message over CAN bus (used only for CAN logger)
*/
uint8_t CANo_SendMsg(can_frame_t msg_to_send)
{
    fsp_err_t err;
    
    err = R_CANFD_Write(&g_canfd0_ctrl, CANFD_TX_MB_0, &msg_to_send);
    
    if(err != FSP_SUCCESS)
    {
        return CAN_FAIL;
    }
    
    else
        return CAN_OK;
}
#endif
