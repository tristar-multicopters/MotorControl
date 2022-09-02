/**
  * @file    gnr_ra6t2_it.c
  * @brief   Interrupt Service Routines.
*/

/* Includes ------------------------------------------------------------------*/
#include "gnr_main.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "vc_config.h"
#include "uCAL_UART.h"
#include "comm_config.h"
#include "board_hardware.h"
#include "comm_tasks.h"

/**
  * @brief  Interrupt routine of ADC hardware.
  * @param  p_args: ADC callback function arguments.
  */
void ADC_IRQHandler(adc_callback_args_t * p_args)
{
	if (p_args->event == ADC_EVENT_SCAN_COMPLETE && p_args->group_mask == ADC_GROUP_MASK_0)
	{
		/* Run motor control high frequency task */
		//R_IOPORT_PinWrite(g_ioport.p_ctrl, BSP_IO_PORT_13_PIN_01, BSP_IO_LEVEL_HIGH);
		//R_IOPORT_PinWrite(g_ioport.p_ctrl, BSP_IO_PORT_13_PIN_01, BSP_IO_LEVEL_LOW);
		MC_HighFrequencyTask();
		//R_IOPORT_PinWrite(g_ioport.p_ctrl, BSP_IO_PORT_13_PIN_01, BSP_IO_LEVEL_HIGH);
		//R_IOPORT_PinWrite(g_ioport.p_ctrl, BSP_IO_PORT_13_PIN_01, BSP_IO_LEVEL_LOW);
	}

}

/**
  * @brief  Interrupt routine of PWM timer hardware.
  * @param  p_args: Timer callback function arguments.
  */
void PWMTimer_IRQHandler(timer_callback_args_t * p_args)
{
	if (p_args->event == TIMER_EVENT_TROUGH)
	{
	}

	if (p_args->event == TIMER_EVENT_CREST)
	{
		/* Run motor control timer update routine */
		//R_IOPORT_PinWrite(g_ioport.p_ctrl, BSP_IO_PORT_13_PIN_02, BSP_IO_LEVEL_HIGH);
		//R_IOPORT_PinWrite(g_ioport.p_ctrl, BSP_IO_PORT_13_PIN_02, BSP_IO_LEVEL_LOW);
		PWMInsulCurrSensorFdbk_TIMx_UP_IRQHandler(&PWMInsulCurrSensorFdbkHandleM1);
	}
}

/**
  * @brief  Interrupt routine when POEG pin is pulled down. It means an overcurrent
	*					condition is detected and consequently PWM outputs were forcefully stopped.
  * @param  p_args: POEG callback function arguments, not used.
  */
void PWMBreak1_IRQHandler(poeg_callback_args_t * p_args)
{
    if(NULL != p_args)
    {
        /* Stop POEG module so it does not reenter the interrupt twice */
        R_POEG_Close((PWM_POEG0_HANDLE_ADDRESS)->p_ctrl);
        /* Run motor control PWM break routine */
        PWMInsulCurrSensorFdbk_BRK_IRQHandler(&PWMInsulCurrSensorFdbkHandleM1);
    }
}

/**
  * @brief  Interrupt routine when POEG pin is pulled down. It means an overcurrent
	*					condition is detected and consequently PWM outputs were forcefully stopped.
  * @param  p_args: POEG callback function arguments, not used.
  */
void PWMBreak2_IRQHandler(poeg_callback_args_t * p_args)
{
    if(NULL != p_args)
    {
    }
}

/**
  * @brief  Interrupt routine of hall timer hardware.
  * @param  p_args: Timer callback function arguments.
  */
void HallTimer_IRQHandler(timer_callback_args_t * p_args)
{
    if (NULL != p_args)
    {
        switch (p_args->event)
        {
            case TIMER_EVENT_CYCLE_END:
                HallPosSensor_TIMx_UP_IRQHandler(&HallPosSensorM1);
                break;
            case TIMER_EVENT_CAPTURE_A:
                HallPosSensor_TIMx_CC_IRQHandler(&HallPosSensorM1,&p_args->capture);
                break;
            case TIMER_EVENT_CAPTURE_B:
                break;
            case TIMER_EVENT_TROUGH:
                break;
        }
    }
}

/**
  * @brief  Interrupt routine of pedal speed timer
  * @param  p_args: callback function arguments
  */
void PedalSpeedTimer_IRQHandler(timer_callback_args_t * p_args)
{
	if (NULL != p_args)
	{
		/* Check for the event */
		switch(p_args->event)
		{
			case TIMER_EVENT_CAPTURE_A :
			/* Call ISR AGT Capture function */
			PulseFrequency_IsrCallUpdate(VCInterfaceHandle.pPowertrain->pPSS->pPulseFrequency,p_args->capture);				
			break;
			case TIMER_EVENT_CYCLE_END:
			/* An overflow occurred during capture. */
			PulseFrequency_ISROverflowUpdate(VCInterfaceHandle.pPowertrain->pPSS->pPulseFrequency);
			break;
			default:
			break;
		}
	}
}

/**
  * @brief  Interrupt routine of wheel speed timer
  * @param  p_args: callback function arguments
  */
void WheelSpeedTimer_IRQHandler(timer_callback_args_t * p_args)
{
	if (NULL != p_args)
	{
			/* Check for the event */
			switch(p_args->event)
			{
				case TIMER_EVENT_CAPTURE_B :
				/* Call ISR GPT Capture function */
				PulseFrequency_IsrCallUpdate(VCInterfaceHandle.pPowertrain->pWSS->pPulseFrequency, p_args->capture);	 					
				break;
				case TIMER_EVENT_CYCLE_END:
				/* An overflow occurred during capture. */
				PulseFrequency_ISROverflowUpdate(VCInterfaceHandle.pPowertrain->pWSS->pPulseFrequency); 
				break;
				default:
				break;
			}
	}		
}

/**
  * @brief  Interrupt routine of UART module
  * @param  p_args: UART callback function arguments.
  */
void UART_IRQHandler(uart_callback_args_t * p_args)
{
    UNUSED_PARAMETER(p_args);
//    // Handle the UART event
//    switch (p_args->event)
//    {
//        // Received a character (unplanned reception)
//        case UART_EVENT_RX_CHAR:

//            break;
//        // Receive complete
//        case UART_EVENT_RX_COMPLETE:

//            UART0Handle.pRxCallback(UART0Handle.Super);
//            break;
//        // Transmit complete
//        case UART_EVENT_TX_COMPLETE:

//            UART0Handle.pTxCallback(UART0Handle.Super);
//            break;
//        default:
//            break;
//    }
}

/**
  * @brief  Interrupt routine of CAN Interface
  * @param  p_args: UART callback function arguments.
*/
void CANFD_IRQhandler(can_callback_args_t *p_args)
{
    UNUSED_PARAMETER(p_args);
    switch (p_args->event)
    {
        case CAN_EVENT_RX_COMPLETE:    /* Receive complete event. */
        {
            break;
        }
        case CAN_EVENT_TX_COMPLETE:    /* Transmit complete event. */
        {
            CAN_SendNextFrame();
            break;
        }
        case CAN_EVENT_ERR_BUS_OFF:          /* Bus error event. (bus off) */
        case CAN_EVENT_ERR_PASSIVE:          /* Bus error event. (error passive) */
        case CAN_EVENT_ERR_WARNING:          /* Bus error event. (error warning) */
        case CAN_EVENT_BUS_RECOVERY:         /* Bus error event. (bus recovery) */
        case CAN_EVENT_MAILBOX_MESSAGE_LOST: /* Overwrite/overrun error */
        {
            break;
        }
        default:
        {
            break;
        }
    }
    return;
}
/**
  * @brief  Interrupt routine of timer used for CANOpen Stack
  * @param  p_args: UART callback function arguments.
*/
void CANTimer_IRQHandler(timer_callback_args_t * p_args)
{
    if( p_args->event == TIMER_EVENT_CYCLE_END)
    {
        can_info_t CANinfo;
        R_CANFD_InfoGet(&g_canfd0_ctrl, &CANinfo);
        if (CANinfo.rx_mb_status != 0) // New message received
        {
            if (bCANOpenTaskBootUpCompleted == true)
            {
                CONodeProcess(&CONodeGNR);
            }
        }

        COTimerCallback(&CONodeGNR.Tmr);
        CAN_SendNextFrame();
    }
}
