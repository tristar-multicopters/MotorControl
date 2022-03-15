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
static void sendVehicleMonitoringCANmsg(MCP25625_Handle_t * pCANHandle, VC_Handle_t * pVCHandle);
static void sendMotorMonitoringCANmsg(MCP25625_Handle_t * pCANHandle, VC_Handle_t * pVCHandle, uint8_t motorSelection);


/************* DEFINES ****************/

#define MINIMUM_STARTING_TORQUE 							1000
#define MAX_STARTUP_TIME 											1000
#define RETURN_TO_STANDBY_LOOPTICKS 					50

/************* STATE MACHINE TIMER DECLARATION ****************/

static const osTimerAttr_t TmrAtt_STMTimeoutTimer = {
	.name = "STM_TimeOut"
};
static osTimerId_t STMTimeoutTimer;
static bool flagSTMTimeout = false;

static void STMTimeoutCallback(void *argument)
{
	UNUSED_PARAMETER(argument);
	flagSTMTimeout = true;
}	


/************* TASKS ****************/

void VC_BootUp(void)
{	
	/* Configure clock to use 32Mhz crystal */
	nrf_drv_clock_handler_item_t VC_clock_handler =
	{
			.event_handler = NULL
	};
	nrf_drv_clock_hfclk_request(&VC_clock_handler);
	nrf_drv_clock_init();
	
	/* Initialize GPIO module */
	nrf_drv_gpiote_init();

	#if CANBUS_ENABLE
	/* Initialize SPI bus and CAN */
	SPI_Init(&SPI0Manager);
	MCP25625_Init(&CANController);
	#endif
	
	/* Initialize each motor drive and UART communication between NRF and STM */
	md_comm_init(VController.pMDComm);
	///////////////////////////////////////////////////
	LCD_Baf_Init(&VController);
	///////////////////////////////////////////////////
	#if HOSTCOMM_ENABLE
	/* Initialize communication with host */
	host_comm_init(&HOSTCommHandler, &VController);
	#endif
	
	/* Initialize vehicle controller components */
	Throttle_Init(VController.pThrottle);
	#if PAS_ENABLE
	PAS_Init(VController.pPedalAssist);
	#endif
	Brake_Init(VController.pBrake);
	VCSTM_Init(VController.pSTM);
	TD_Init(VController.pTorqueDistributor);
	
	/* Initialize ADC module */
	RCM_Init(&RegularConvertionManager);
}

__NO_RETURN void TSK_FastLoopMD (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	//*******************************************//
	
	VC_Handle_t * pVControl = &VController;
	Throttle_Handle_t * pThrottleSensor = VController.pThrottle;
	Brake_Handle_t * pBrakeSensor = VController.pBrake;
	TD_Handle_t * pTorqueDistributor = VController.pTorqueDistributor;
	PAS_Handle_t * pPedalAssist = VController.pPedalAssist;
	MD_Handle_t * pMDrive1 = VController.pMDComm->pMD[M1];
	MD_Handle_t * pMDrive2 = VController.pMDComm->pMD[M2];
	int16_t hTref;
	
	uint32_t xLastWakeTime = osKernelGetTickCount();
	
	while (true)
	{
		Throttle_CalcAvValue(pThrottleSensor);
		hTref = Throttle_CalcIqref(pThrottleSensor);
		#if PAS_ENABLE
		hTref = PAS_CalcIqref(pPedalAssist,hTref);
		#endif
		hTref = Brake_CalcIqref(pBrakeSensor, hTref);
		TD_DistributeTorque(pTorqueDistributor, hTref);
		
		#if USE_MOTOR1
		getMonitoringReg_Fast(M1);
		#endif
		#if USE_MOTOR2
		getMonitoringReg_Fast(M2);
		#endif
		
		xLastWakeTime += TASK_VCFASTLOOP_SAMPLE_TIME_TICK;
		osDelayUntil(xLastWakeTime);
	}
}

__NO_RETURN void TSK_SlowLoopMD (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	//*******************************************//
	
	VC_Handle_t * pVControl = &VController;
	MCP25625_Handle_t * pCANController = &CANController;
	
	uint32_t xLastWakeTime = osKernelGetTickCount();
																				
	while (true)
	{
		#if USE_MOTOR1
		getMonitoringReg_Slow(M1);
			#if CANBUS_ENABLE
			sendMotorMonitoringCANmsg(pCANController, pVControl, M1);
			#endif
		#endif
		#if USE_MOTOR2
		getMonitoringReg_Slow(M2);
			#if CANBUS_ENABLE
			sendMotorMonitoringCANmsg(pCANController, pVControl, M2);
			#endif
		#endif
			#if CANBUS_ENABLE
			sendVehicleMonitoringCANmsg(pCANController, pVControl);
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
	uint16_t hFaultCode;
	bool bStartupSuccess;
	uint32_t wReturn2StandbyCounter;
	int32_t wSpeedM1, wSpeedM2;
	int16_t hTref, hTref1, hTref2;
	int16_t hTemp1, hTemp2;

	VC_Handle_t * pVControl = &VController;
	TD_Handle_t * pTorqueDistributor = VController.pTorqueDistributor;
	MD_Handle_t * pMDrive1 = VController.pMDComm->pMD[M1];
	MD_Handle_t * pMDrive2 = VController.pMDComm->pMD[M2];
	
	STMTimeoutTimer = osTimerNew(&STMTimeoutCallback, osTimerPeriodic, NULL, &TmrAtt_STMTimeoutTimer);
	uint32_t xLastWakeTime = osKernelGetTickCount();
	
	while (true)
	{
		if ( md_isErrorOccured() )
		{
			VCSTM_FaultProcessing( VController.pSTM, VC_MC_COMM_ERROR, 0 );
		}

		wSpeedM1 = VC_getMotorSpeedMeas(pVControl, M1);
		wSpeedM2 = VC_getMotorSpeedMeas(pVControl, M2);
		hTref = TD_GetTorqueMainMotor(pTorqueDistributor);
		hTemp1 = VC_getInverterHeatsinkTemp(pVControl, M1);
		hTemp2 = VC_getInverterHeatsinkTemp(pVControl, M2);
		#if DERATING_ENABLE
		hTref1 = DRT_CalcDeratedTorque( &pMDrive1->DeratingHandler, TD_GetTorqueM1(pTorqueDistributor), hTemp1 );
		hTref2 = DRT_CalcDeratedTorque( &pMDrive2->DeratingHandler, TD_GetTorqueM2(pTorqueDistributor), hTemp2 );
		#else
		hTref1 = TD_GetTorqueM1(pTorqueDistributor);
		hTref2 = TD_GetTorqueM2(pTorqueDistributor);
		#endif
		
		StateVC = VCSTM_GetState( VController.pSTM );
		switch ( StateVC )
		{
			case V_IDLE:
					osDelay(1000);
					VCSTM_NextState( VController.pSTM, V_STANDBY );
					break;
			
			case V_STANDBY:
					if ( abs( hTref ) > MINIMUM_STARTING_TORQUE )
					{
						VCSTM_NextState( VController.pSTM, V_STANDBY_START );
					}
					break;
			
			case V_STANDBY_START:
					bStartupSuccess = false;
					flagSTMTimeout = false;
					if(osTimerIsRunning(STMTimeoutTimer))
						 osTimerStop(STMTimeoutTimer);
					wReturn2StandbyCounter = 0;
					#if USE_MOTOR1
					md_setTorqueRamp(M1, hTref1, 0);
					md_startMotor(M1);
					#endif
					#if USE_MOTOR2
					md_setTorqueRamp(M2, hTref2, 0);
					md_startMotor(M2);
					#endif
					osTimerStart(STMTimeoutTimer, MAX_STARTUP_TIME);
					VCSTM_NextState( VController.pSTM, V_START_FORWARD );
					break;
			
			case V_START_FORWARD:
					bStartupSuccess = true;
					#if USE_MOTOR1
					switch ( pMDrive1->MDStateMachine.bMState )
					{
						case M_RUN:
							bStartupSuccess &= true;
							break;
						case M_FAULT_NOW: 
						case M_FAULT_OVER:
							bStartupSuccess = false;
							flagSTMTimeout = false;
							if(osTimerIsRunning(STMTimeoutTimer))
								 osTimerStop(STMTimeoutTimer);
							VCSTM_FaultProcessing( VController.pSTM, VC_M1_FAULTS, 0 );
							break;
						default:
							bStartupSuccess = false;
							break;
					}
					#endif
					#if USE_MOTOR2
					switch ( pMDrive2->MDStateMachine.bMState )
					{
						case M_RUN:
							bStartupSuccess &= true;
							break;
						case M_FAULT_NOW:
						case M_FAULT_OVER:
							bStartupSuccess = false;
							flagSTMTimeout = false;
							if(osTimerIsRunning(STMTimeoutTimer))
								 osTimerStop(STMTimeoutTimer);
							VCSTM_FaultProcessing( VController.pSTM, VC_M2_FAULTS, 0 );
							break;
						default:
							bStartupSuccess = false;
							break;
					}
					#endif
					if ( flagSTMTimeout )
					{
						bStartupSuccess = false;
						flagSTMTimeout = false;
						if(osTimerIsRunning(STMTimeoutTimer))
							 osTimerStop(STMTimeoutTimer);
						VCSTM_FaultProcessing( VController.pSTM, VC_STARTUP_TIMEOUT, 0 );
					}
					if ( bStartupSuccess )
					{
						bStartupSuccess = false;
						flagSTMTimeout = false;
						if(osTimerIsRunning(STMTimeoutTimer))
							 osTimerStop(STMTimeoutTimer);
						VCSTM_NextState( VController.pSTM, V_RUN_FORWARD );
					}
					break;
			
			case V_RUN_FORWARD:
					#if USE_MOTOR1
					switch ( pMDrive1->MDStateMachine.bMState )
					{
						case M_RUN:
							md_setTorqueRamp(M1, hTref1, 0);
							break;
						case M_FAULT_NOW:
						case M_FAULT_OVER:
							VCSTM_FaultProcessing( VController.pSTM, VC_M1_FAULTS, 0 );
							break;
						default:
							VCSTM_FaultProcessing( VController.pSTM, VC_M1_UNEXPECTED_BEHAVIOR, 0 );
							break;
					}
					#endif
					#if USE_MOTOR2
					switch ( pMDrive2->MDStateMachine.bMState )
					{
						case M_RUN:
							md_setTorqueRamp(M2, hTref2, 0);
							break;
						case M_FAULT_NOW:
						case M_FAULT_OVER:
							VCSTM_FaultProcessing( VController.pSTM, VC_M2_FAULTS, 0 );
							break;
						default:
							VCSTM_FaultProcessing( VController.pSTM, VC_M2_UNEXPECTED_BEHAVIOR, 0 );
							break;
					}
					#endif
					if (hTref == 0 && wSpeedM1 == 0 && wSpeedM2 == 0)
					{
						wReturn2StandbyCounter++;
					}
					else
					{
						wReturn2StandbyCounter = 0;
					}
					if (wReturn2StandbyCounter > RETURN_TO_STANDBY_LOOPTICKS)
					{
						wReturn2StandbyCounter = 0;
						md_stopMotor(M1);
						md_stopMotor(M2);
						VCSTM_NextState( VController.pSTM, V_STANDBY );
					}
					break;
					
			case V_FAULT_NOW:
					#if USE_MOTOR1
					md_stopMotor(M1);
					#endif
					#if USE_MOTOR2
					md_stopMotor(M2);
					#endif
					hFaultCode = VC_getVehicleFaultNow(pVControl);
					if ( hFaultCode != VC_NO_FAULTS )
					{
						if ( hFaultCode & VC_M1_FAULTS )
						{
							if ( VC_getMotorState(pVControl, M1) == M_FAULT_OVER )
							{
								md_faultAcknowledge(M1);
								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M1_FAULTS );
							}
						}
						if ( hFaultCode & VC_M2_FAULTS )
						{
							if ( VC_getMotorState(pVControl, M2) == M_FAULT_OVER )
							{
								md_faultAcknowledge(M2);
								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M2_FAULTS );
							}
						}
						if ( hFaultCode & VC_SW_ERROR )
						{
							
						}
						if ( hFaultCode & VC_MC_COMM_ERROR )
						{
							
						}
						if ( hFaultCode & VC_M1_UNEXPECTED_BEHAVIOR )
						{
							
						}
						if ( hFaultCode & VC_M2_UNEXPECTED_BEHAVIOR )
						{
							
						}
						if ( hFaultCode & VC_STARTUP_TIMEOUT )
						{
							
						}
					}
					break;
			case V_FAULT_OVER:
					VCSTM_FaultAcknowledged( VController.pSTM );
					break;
			
			default:
				break;
		}
	
		xLastWakeTime += TASK_VCSTM_SAMPLE_TIME_TICK;
		osDelayUntil(xLastWakeTime);
	}
}

static void sendVehicleMonitoringCANmsg(MCP25625_Handle_t * pCANHandle, VC_Handle_t * pVCHandle)
{
	CAN_SendStatus(pCANHandle, pVCHandle, M_NONE);
	CAN_SendThrottleBrake(pCANHandle, pVCHandle);
	CAN_SendVbus(pCANHandle, pVCHandle);
}

static void sendMotorMonitoringCANmsg(MCP25625_Handle_t * pCANHandle, VC_Handle_t * pVCHandle, uint8_t motorSelection)
{	
	CAN_SendSpeed(pCANHandle, pVCHandle, motorSelection);
	CAN_SendStatus(pCANHandle, pVCHandle, motorSelection);
	CAN_SendCurrent(pCANHandle, pVCHandle, motorSelection);
	CAN_SendTemperature(pCANHandle, pVCHandle, motorSelection);
}

static void getMonitoringReg_Slow(uint8_t motorSelection)
{
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_BUS_VOLTAGE);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_HEATS_TEMP);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_SPEED_MEAS);
}

static void getMonitoringReg_Fast(uint8_t motorSelection)
{
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_FLAGS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_STATUS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_TORQUE_MEAS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_FLUX_MEAS);
}

