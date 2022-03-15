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
#define STARTUP_ENABLE 												1

#if USE_SENSORLESS
	#define MAX_STARTUP_TIME 											5000
	#define RETURN_TO_STANDBY_LOOPTICKS 					5
	#define SENSORLESS_MINIMUM_SPEED							350
#else
	#define MAX_STARTUP_TIME 											1000
	#define RETURN_TO_STANDBY_LOOPTICKS 					50
#endif

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
	////////////////FOR TESTING//////////////////////////
	VController.euart_type = EUART_EGG;
	////////////////////////////////////////////////////
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
	uint16_t hVFaultCode, hM1FaultCode, hM2FaultCode;
	bool bStartupSuccess;
	uint32_t wReturn2StandbyCounterM1, wReturn2StandbyCounterM2;
	int32_t wSpeedM1, wSpeedM2;
	int16_t hTref, hTref1, hTref2;
	int16_t hTemp1, hTemp2;
	bool bRestartAttemptedM1, bRestartAttemptedM2;

	VC_Handle_t * pVControl = &VController;
	TD_Handle_t * pTorqueDistributor = VController.pTorqueDistributor;
	MD_Handle_t * pMDrive1 = VController.pMDComm->pMD[M1];
	MD_Handle_t * pMDrive2 = VController.pMDComm->pMD[M2];
	
	STMTimeoutTimer = osTimerNew(&STMTimeoutCallback, osTimerOnce, NULL, &TmrAtt_STMTimeoutTimer);
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
					bRestartAttemptedM1 = false, bRestartAttemptedM2 = false;
					if(osTimerIsRunning(STMTimeoutTimer))
						 osTimerStop(STMTimeoutTimer);
					wReturn2StandbyCounterM1 = 0;
					wReturn2StandbyCounterM2 = 0;
					#if USE_MOTOR1
					md_setTorqueRamp(M1, hTref1, 0);
					md_startMotor(M1);
					#endif
					#if USE_MOTOR2
					md_setTorqueRamp(M2, hTref2, 0);
					md_startMotor(M2);
					#endif
					#if STARTUP_ENABLE
					osTimerStart(STMTimeoutTimer, MAX_STARTUP_TIME);
					#endif
					VCSTM_NextState( VController.pSTM, V_START_FORWARD );
					break;
			
			case V_START_FORWARD:
					#if STARTUP_ENABLE
					bStartupSuccess = true;
					#if USE_MOTOR1
					switch ( pMDrive1->MDStateMachine.bMState )
					{
						case M_RUN:
							bStartupSuccess &= true;
							break;
						case M_IDLE:
							if ( !bRestartAttemptedM1 )
							{
								md_setTorqueRamp(M1, hTref1, 0);
								md_startMotor(M1);
							}
							bRestartAttemptedM1 = true;
							bStartupSuccess = false;
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
						case M_IDLE:
							if ( !bRestartAttemptedM2 )
							{
								md_setTorqueRamp(M2, hTref2, 0);
								md_startMotor(M2);
							}
							bRestartAttemptedM2 = true;
							bStartupSuccess = false;
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
					#else
					VCSTM_NextState( VController.pSTM, V_RUN_FORWARD );
					#endif
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
					#if USE_MOTOR1
						#if USE_SENSORLESS
						if ( abs(wSpeedM1) <= SENSORLESS_MINIMUM_SPEED )
						#else
						if (hTref == 0 && wSpeedM1 == 0 && wSpeedM2 == 0)
						#endif
						{
							wReturn2StandbyCounterM1++;
						}
						else
						{
							wReturn2StandbyCounterM1 = 0;
						}
						if (wReturn2StandbyCounterM1 > RETURN_TO_STANDBY_LOOPTICKS)
						{
							wReturn2StandbyCounterM1 = 0;
							#if USE_MOTOR1
							md_stopMotor(M1);
							#endif
							#if USE_MOTOR2
							md_stopMotor(M2);
							#endif
							VCSTM_NextState( VController.pSTM, V_STANDBY );
						}
					#endif
					#if USE_MOTOR2
						#if USE_SENSORLESS
						if ( abs(wSpeedM2) <= SENSORLESS_MINIMUM_SPEED )
						#else
						if (hTref == 0 && wSpeedM2 == 0 && wSpeedM1 == 0)
						#endif
						{
							wReturn2StandbyCounterM2++;
						}
						else
						{
							wReturn2StandbyCounterM2 = 0;
						}
						if (wReturn2StandbyCounterM2 > RETURN_TO_STANDBY_LOOPTICKS)
						{
							wReturn2StandbyCounterM2 = 0;
							#if USE_MOTOR1
							md_stopMotor(M1);
							#endif
							#if USE_MOTOR2
							md_stopMotor(M2);
							#endif
							VCSTM_NextState( VController.pSTM, V_STANDBY );
						}
					#endif
					break;
					
			case V_FAULT_NOW:
					#if USE_MOTOR1
					md_stopMotor(M1);
					#endif
					#if USE_MOTOR2
					md_stopMotor(M2);
					#endif
			
					hVFaultCode = VC_getVehicleFaultNow(pVControl);
					hM1FaultCode = VC_getMotorFaultNow(pVControl, M1);
					hM2FaultCode = VC_getMotorFaultNow(pVControl, M2);
			
					#if ERROR_MANAGEMENT_ENABLE
					#if USE_MOTOR1
					if ( hVFaultCode & VC_M1_FAULTS )
					{
						if ( VC_getMotorState(pVControl, M1) == M_FAULT_OVER )
						{
							if ( hM1FaultCode == MC_BREAK_IN )
							{
									/* In case of motor overcurrent... */
//								md_faultAcknowledge(M1);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M1_FAULTS );
							}
							if ( hM1FaultCode == MC_SPEED_FDBK )
							{
									/* In case of speed sensor failure... */
//								md_faultAcknowledge(M1);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M1_FAULTS );
							}
							if ( hM1FaultCode == MC_START_UP )
							{
									/* In case of motor startup failure... */
//								md_faultAcknowledge(M1);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M1_FAULTS );
							}
							if ( hM1FaultCode == MC_OVER_TEMP )
							{
									/* In case of overtemperature... */
//								md_faultAcknowledge(M1);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M1_FAULTS );
							}
							if ( hM1FaultCode == MC_OVER_VOLT )
							{
									/* In case of DCbus overvoltage... */
//								md_faultAcknowledge(M1);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M1_FAULTS );
							}
							if ( hM1FaultCode == MC_UNDER_VOLT )
							{
									/* In case of DCbus undervoltage... */
//								md_faultAcknowledge(M1);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M1_FAULTS );
							}
						}
					}
					#endif
					#if USE_MOTOR2
					if ( hVFaultCode & VC_M2_FAULTS )
					{
						if ( VC_getMotorState(pVControl, M2) == M_FAULT_OVER )
						{
							if ( hM2FaultCode == MC_BREAK_IN )
							{
									/* In case of motor overcurrent... */
//								md_faultAcknowledge(M2);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M2_FAULTS );
							}
							if ( hM2FaultCode == MC_SPEED_FDBK )
							{
									/* In case of speed sensor failure... */
//								md_faultAcknowledge(M2);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M2_FAULTS );
							}
							if ( hM2FaultCode == MC_START_UP )
							{
									/* In case of motor startup failure... */
//								md_faultAcknowledge(M2);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M2_FAULTS );
							}
							if ( hM2FaultCode == MC_OVER_TEMP )
							{
									/* In case of overtemperature... */
//								md_faultAcknowledge(M2);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M2_FAULTS );
							}
							if ( hM2FaultCode == MC_OVER_VOLT )
							{
									/* In case of DCbus overvoltage... */
//								md_faultAcknowledge(M2);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M2_FAULTS );
							}
							if ( hM2FaultCode == MC_UNDER_VOLT )
							{
									/* In case of DCbus undervoltage... */
//								md_faultAcknowledge(M2);
//								VCSTM_FaultProcessing( VController.pSTM, 0, VC_M2_FAULTS );
							}
						}
					}
					#endif
					if ( hVFaultCode & VC_SW_ERROR )
					{
						
					}
					if ( hVFaultCode & VC_MC_COMM_ERROR )
					{
						
					}
					if ( hVFaultCode & VC_M1_UNEXPECTED_BEHAVIOR )
					{
						
					}
					if ( hVFaultCode & VC_M2_UNEXPECTED_BEHAVIOR )
					{
						
					}
					if ( hVFaultCode & VC_STARTUP_TIMEOUT )
					{
						
					}
					#endif
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

__NO_RETURN void TSK_eUART (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	
	eUART_protocol_t dev_type = VController.euart_type;
	switch(dev_type)
	{
		case EUART_HOST:
			HOST_comm_init(&VController);
			break;
		
		case EUART_BAFANG:
			LCD_BAF_init(&VController);
			break;
		
		case EUART_EGG:
			LCD_EGG_init(&VController);
			break;
		
		default:
			break;
	}
	
	while(true)
	{
		osThreadFlagsWait(EUART_FLAG, osFlagsWaitAny, osWaitForever);
		switch(dev_type)
		{
			case EUART_HOST:
				HOST_frame_received_protocol();
				break;
			
			case EUART_BAFANG:
				LCD_BAF_frame_Process();
				break;
			
			case EUART_EGG:
				LCD_EGG_frame_Process();
				break;
			
			default:
				break;
		}
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
}

static void getMonitoringReg_Fast(uint8_t motorSelection)
{
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_FLAGS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_STATUS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_TORQUE_MEAS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_FLUX_MEAS);
	md_getMDReg(motorSelection, MC_PROTOCOL_REG_SPEED_MEAS);
}

