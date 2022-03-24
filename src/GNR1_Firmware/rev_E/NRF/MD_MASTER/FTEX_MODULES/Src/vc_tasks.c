/**
  ******************************************************************************
  * @file    vc_tasks.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module gathers tasks of vehicule controller
  *
	******************************************************************************
	*/

#include "vc_tasks.h"
#include "EventRecorder.h"

osThreadId_t TSK_FastLoopMD_handle;
osThreadId_t TSK_SlowLoopMD_handle;
osThreadId_t TSK_VehicleStateMachine_handle;

static void getMonitoringReg_Fast(uint8_t motorSelection);
static void getMonitoringReg_Slow(uint8_t motorSelection);

#if CANBUS_ENABLE
static void sendVehicleMonitoringCANmsg(MCP25625_Handle_t * pCANHandle, VCI_Handle_t * pVCHandle);
static void sendMotorMonitoringCANmsg(MCP25625_Handle_t * pCANHandle, VCI_Handle_t * pVCHandle, uint8_t motorSelection);
#endif

/************* DEFINES ****************/

#define RETURN_TO_STANDBY_LOOPTICKS 10


/************* TASKS ****************/

static void CLK_Init(void)
{
	/* Configure clock to use 32Mhz crystal */
	nrf_drv_clock_handler_item_t VC_clock_handler =
	{
		.event_handler = NULL
	};
	nrf_drv_clock_hfclk_request(&VC_clock_handler);
	nrf_drv_clock_init();
}

void VC_BootUp(void)
{	
	VCI_Handle_t * pVCI = &VCInterfaceHandle;
	//LCD_handle_t * pLCD = &BafangScreenHandle;
	
	/* Initialize clock */
	CLK_Init();
	
	/* Initialize GPIO module */
	nrf_drv_gpiote_init();

	#if CANBUS_ENABLE
	/* Initialize SPI bus and CAN */
	SPI_Init(&SPI0Manager);
	
	// todo: handle returned result
	
	MCP25625_Init(&CANController);
	#endif
	
	/* Initialize vehicle controller components */
	VCSTM_Init(pVCI->pStateMachine);
	DRVT_Init(pVCI->pDrivetrain);
	
	/* Initialize ADC module */

	RCM_Init(&RegularConvertionManager);
	
	STRG_Init();
	//EVNC_init(&VCInterfaceHandle);

}

__NO_RETURN void TSK_FastLoopMD (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	//*******************************************//
	
	VCI_Handle_t * pVCI = &VCInterfaceHandle;
	
	uint32_t xLastWakeTime = osKernelGetTickCount();
	while (true)
	{		
		DRVT_CalcTorqueSpeed(pVCI->pDrivetrain);
		
		if ( DRVT_IsMotor1Used(pVCI->pDrivetrain) )
		{
			getMonitoringReg_Fast(M1);
		}
		if ( DRVT_IsMotor2Used(pVCI->pDrivetrain) )
		{
			getMonitoringReg_Fast(M2);
		}
		
		xLastWakeTime += TASK_VCFASTLOOP_SAMPLE_TIME_TICK;
		osDelayUntil(xLastWakeTime);
	}
}

__NO_RETURN void TSK_SlowLoopMD (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	//*******************************************//
	
	VCI_Handle_t * pVCI = &VCInterfaceHandle;
	
	#if CANBUS_ENABLE
	MCP25625_Handle_t * pCANController = &CANController;
	#endif
	
	uint32_t xLastWakeTime = osKernelGetTickCount();
																				
	while (true)
	{
		if ( DRVT_IsMotor1Used(pVCI->pDrivetrain) )
		{
			getMonitoringReg_Slow(M1);
			#if CANBUS_ENABLE
			sendMotorMonitoringCANmsg(pCANController, pVCI, M1);
			#endif
		}
		if ( DRVT_IsMotor2Used(pVCI->pDrivetrain) )
		{
			getMonitoringReg_Slow(M2);
			#if CANBUS_ENABLE
			sendMotorMonitoringCANmsg(pCANController, pVCI, M2);
			#endif
		}
		#if CANBUS_ENABLE
		sendVehicleMonitoringCANmsg(pCANController, pVCI);
		#endif
		
		xLastWakeTime += TASK_VCSLOWLOOP_SAMPLE_TIME_TICK;
		osDelayUntil(xLastWakeTime);
	}
}

__NO_RETURN void TSK_VehicleStateMachine (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(500);
	//*******************************************//

	VC_State_t StateVC;
	VCI_Handle_t * pVCI = &VCInterfaceHandle;
	
	uint32_t wCounter;
	uint16_t hVehicleFault;
	
	uint32_t xLastWakeTime = osKernelGetTickCount();
	while (true)
	{		
//		StateVC = VCSTM_GetState( pVCI->pStateMachine );
//		switch ( StateVC )
//		{
//			case V_IDLE:
//					osDelay(1000);
//					VCSTM_NextState( pVCI->pStateMachine, V_STANDBY );
//					break;
//			
//			case V_STANDBY:
//					hVehicleFault = DRVT_StandbyStateCheck(pVCI->pDrivetrain);
//					VCSTM_FaultProcessing( pVCI->pStateMachine, hVehicleFault, 0 );
//					if ( DRVT_CheckStartConditions(pVCI->pDrivetrain) )
//					{
//						VCSTM_NextState( pVCI->pStateMachine, V_STANDBY_START );
//					}
//					break;
//			
//			case V_STANDBY_START:
//					wCounter = 0;
//					VCSTM_NextState( pVCI->pStateMachine, V_START );
//					break;
//			
//			case V_START:
//					DRVT_StartMotors(pVCI->pDrivetrain);
//					hVehicleFault = DRVT_StartStateCheck(pVCI->pDrivetrain);
//					VCSTM_FaultProcessing( pVCI->pStateMachine, hVehicleFault, 0 );
//					if ( DRVT_IsDrivetrainActive(pVCI->pDrivetrain) )
//					{
//						wCounter = 0;
//						VCSTM_NextState( pVCI->pStateMachine, V_RUN );
//					}
//					wCounter++;
//					if ( wCounter > 100 )
//					{
//						wCounter = 0;
//						VCSTM_FaultProcessing( pVCI->pStateMachine, VC_START_TIMEOUT, 0 );
//					}
//					break;
//					
//			case V_RUN:
//					hVehicleFault = DRVT_RunStateCheck(pVCI->pDrivetrain);
//					VCSTM_FaultProcessing( pVCI->pStateMachine, hVehicleFault, 0 );
//					DRVT_UpdateMotorRamps(pVCI->pDrivetrain);
//					if ( DRVT_CheckStopConditions(pVCI->pDrivetrain) )
//					{
//						wCounter++;
//					}
//					else
//					{
//						wCounter = 0;
//					}

//					if (wCounter > RETURN_TO_STANDBY_LOOPTICKS)
//					{
//						wCounter = 0;
//						VCSTM_NextState( pVCI->pStateMachine, V_ANY_STOP );
//					}
//					break;
//					
//			case V_ANY_STOP:
//					wCounter = 0;
//					VCSTM_NextState( pVCI->pStateMachine, V_STOP );					
//					break;
//					
//			case V_STOP:
//					DRVT_StopMotors(pVCI->pDrivetrain);
//					hVehicleFault = DRVT_StopStateCheck(pVCI->pDrivetrain);
//					VCSTM_FaultProcessing( pVCI->pStateMachine, hVehicleFault, 0 );
//					if ( DRVT_IsDrivetrainStopped(pVCI->pDrivetrain) )
//					{
//						wCounter = 0;
//						VCSTM_NextState( pVCI->pStateMachine, V_STANDBY );
//					}
//					wCounter++;
//					if ( wCounter > 100 )
//					{
//						wCounter = 0;
//						VCSTM_FaultProcessing( pVCI->pStateMachine, VC_STOP_TIMEOUT, 0 );
//					}
//					break;
//					
//			case V_FAULT_NOW:
//					DRVT_StopMotors(pVCI->pDrivetrain);
//					if ( DRVT_IsDrivetrainStopped(pVCI->pDrivetrain) )
//					{
//						if ( !DRVT_MotorFaultManagement(pVCI->pDrivetrain) )
//						{
//							VCSTM_FaultProcessing( pVCI->pStateMachine, 0, VC_M1_FAULTS ); // Remove fault on M1
//							VCSTM_FaultProcessing( pVCI->pStateMachine, 0, VC_M2_FAULTS ); // Remove fault on M2
//						}
//					}
//					break;
//					
//			case V_FAULT_OVER:
//					VCSTM_FaultAcknowledged( pVCI->pStateMachine );
//					break;
//			
//			default:
//				break;
//		}
//	
		xLastWakeTime += TASK_VCSTM_SAMPLE_TIME_TICK;
		osDelayUntil(xLastWakeTime);
	}
}

__NO_RETURN void TSK_ProcessEUartFrames (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	switch(EUART_handle_t)
	{
		case EUART_EVIONICS:
			EVNC_init(&VCInterfaceHandle);
			break;
		
		case EUART_BAFANG:
			//LCD_BAF_init(&VCInterfaceHandle);
			break;
		
		case EUART_EGG:
			//LCD_EGG_init(&VCInterfaceHandle);
			break;
		
		default:
			break;
	}
	
	while(true)
	{
		osThreadFlagsWait(EUART_FLAG, osFlagsWaitAny, osWaitForever);
		switch(EUART_handle_t)
		{
			case EUART_EVIONICS:
				EVNC_frame_process();
				break;
			
			case EUART_BAFANG:
				//LCD_BAF_frame_Process();
				break;
			
			case EUART_EGG:
				//LCD_EGG_frame_Process();
				break;
			
			default:
				break;
		}
	}
}

#if CANBUS_ENABLE
static void sendVehicleMonitoringCANmsg(MCP25625_Handle_t * pCANHandle, VCI_Handle_t * pVCHandle)
{
	CAN_SendStatus(pCANHandle, pVCHandle, M_NONE);
	CAN_SendThrottleBrake(pCANHandle, pVCHandle);
	CAN_SendVbus(pCANHandle, pVCHandle);
}
#endif

#if CANBUS_ENABLE
static void sendMotorMonitoringCANmsg(MCP25625_Handle_t * pCANHandle, VCI_Handle_t * pVCHandle, uint8_t motorSelection)
{	
	CAN_SendSpeed(pCANHandle, pVCHandle, motorSelection);
	CAN_SendStatus(pCANHandle, pVCHandle, motorSelection);
	CAN_SendCurrent(pCANHandle, pVCHandle, motorSelection);
	CAN_SendTemperature(pCANHandle, pVCHandle, motorSelection);
}
#endif

static void getMonitoringReg_Slow(uint8_t motorSelection)
{
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_BUS_VOLTAGE);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_HEATS_TEMP);
}

static void getMonitoringReg_Fast(uint8_t motorSelection)
{
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_FLAGS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_STATUS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_TORQUE_MEAS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_FLUX_MEAS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_SPEED_MEAS);
}

