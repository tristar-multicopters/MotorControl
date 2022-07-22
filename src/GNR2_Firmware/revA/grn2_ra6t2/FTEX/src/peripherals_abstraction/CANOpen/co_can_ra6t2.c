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
*  co_can_ra6t2.c
*  Abstraction Layer module for CAN interface
*/

#include "co_can_ra6t2.h"
#include "co_core.h"

extern CO_NODE GnR2Module;
//static fsp_err_t m_err;
    
/* TODO: place here your CAN controller register definitions */

// ================================== Private Functions Prototypes ================================== //

static void    DrvCanInit   (void);
static void    DrvCanEnable (uint32_t baudrate);
static int16_t DrvCanSend   (CO_IF_FRM *frm);
static int16_t DrvCanRead   (CO_IF_FRM *frm);
static void    DrvCanReset  (void);
static void    DrvCanClose  (void);

// ================================== Public Variables ================================== //

const CO_IF_CAN_DRV CoCanDriver = {
    DrvCanInit,
    DrvCanEnable,
    DrvCanRead,
    DrvCanSend,
    DrvCanReset,
    DrvCanClose
};

can_frame_t g_can_tx_frame;
can_frame_t g_can_rx_frame;
volatile bool     g_rx_flag  = false;
volatile bool     g_tx_flag  = false;
volatile bool     g_err_flag = false;
volatile uint32_t g_rx_id;

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

// Callback function for CAN events
void canfd0_callback( can_callback_args_t *p_args )
{
	switch (p_args->event)
	{
		case CAN_EVENT_RX_COMPLETE:    /* Receive complete event. */
		{
			g_can_rx_frame = p_args->frame;
			
			CONodeProcess(&GnR2Module);
			break;
		}
		case CAN_EVENT_TX_COMPLETE:    /* Transmit complete event. */
		{
			g_tx_flag = true;
			break;
		}
		case CAN_EVENT_ERR_BUS_OFF:          /* Bus error event. (bus off) */
		case CAN_EVENT_ERR_PASSIVE:          /* Bus error event. (error passive) */
		case CAN_EVENT_ERR_WARNING:          /* Bus error event. (error warning) */
		case CAN_EVENT_BUS_RECOVERY:         /* Bus error event. (bus recovery) */
		case CAN_EVENT_MAILBOX_MESSAGE_LOST: /* Overwrite/overrun error */
		{
			/* Set error flag */
			g_err_flag = true;
			break;
		}
		default:
		{
			break;
		}
	}
	return;
}

// ================================== Private Functions Definitions ================================== //

static void DrvCanInit(void)
{

    //Use the external loopback mode for initial testing
    R_CANFD_ModeTransition(&g_canfd0_ctrl,CAN_OPERATION_MODE_HALT,CAN_TEST_MODE_DISABLED);
    		
	osDelay(10);
}

static void DrvCanEnable(uint32_t baudrate)
{
	(void)baudrate;

	/* TODO: set the given baudrate to the CAN controller */
	R_CANFD_ModeTransition(&g_canfd0_ctrl,CAN_OPERATION_MODE_NORMAL,CAN_TEST_MODE_DISABLED);
}

static int16_t DrvCanSend(CO_IF_FRM *frm)
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

static int16_t DrvCanRead (CO_IF_FRM *frm)
{
	(void)frm;
		
	frm->Identifier = g_can_rx_frame.id;
	frm->DLC = g_can_rx_frame.data_length_code;
	
	memcpy(frm->Data, g_can_rx_frame.data, 8);
	
	return (sizeof(CO_IF_FRM));
}

static void DrvCanReset(void)
{
  /* TODO: reset CAN controller while keeping baudrate */
	R_CANFD_ModeTransition(&g_canfd0_ctrl, CAN_OPERATION_MODE_RESET, CAN_TEST_MODE_DISABLED);
}

static void DrvCanClose(void)
{
	/* TODO: remove CAN controller from CAN network */
	R_CANFD_Close	(	&g_canfd0_ctrl );
}

// ================================== Public Functions (for CAN Logger only) ================================== //

/**
  Function used to initialise CAN bus for can logger.
*/
void CAN_initInterface(void)
{
    R_CANFD_ModeTransition(&g_canfd0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);
}

/**
  Function used to send a message over CAN bus (for can logger only)
*/
uint8_t CAN_SendMsg(can_frame_t msg_to_send)
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
