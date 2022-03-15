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

static void getMonitoringReg_Fast(uint8_t motorSelection);
static void getMonitoringReg_Slow(uint8_t motorSelection);
static void sendDiagnosticCANmessages(uint8_t motorSelection);

/************* TIMER DECLARATION ****************/
static const osTimerAttr_t TmrAtt_STMTimeoutTimer = {
	.name = "STM_TimeOut"
};
static osTimerId_t STMTimeoutTimer;
static bool flagTimeout = false;

static void STMTimeoutCallback(void *argument)
{
	UNUSED_PARAMETER(argument);
	flagTimeout = true;
}	
/************************************************/

#define MINIMUM_STARTING_TORQUE 1000
#define MAX_STARTUP_TIME 10000
#define RETURN_TO_STANDBY_LOOPTICKS 	25

static void VC_init_clock( void )
{
	static nrf_drv_clock_handler_item_t VC_clock_handler =
	{
			.event_handler = NULL
	};
	nrf_drv_clock_hfclk_request(&VC_clock_handler);
	
	APP_ERROR_CHECK( nrf_drv_clock_init());
}

void VC_BootUp(void)
{	
	VC_init_clock();
	
	APP_ERROR_CHECK( nrf_drv_gpiote_init() );

	VControl.pMDComm->pMD[M1] = MotorDrive1;
	VControl.pMDComm->pMD[M2] = MotorDrive2;
	Brake_Init(VControl.pBrake);
	VCSTM_Init(VControl.pSTM);
	md_comm_init(VControl.pMDComm);  
	Throttle_Init(VControl.pThrottle);
	TD_Init(VControl.pTorqueDistributor);
	RCM_Init(VControl.pRegularConvertionManager);
  SPI_Init(VControl.pMCP->pSPI);
	CAN_Init(&VControl);
}

__NO_RETURN void TSK_FastLoopMD (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	osDelay(250);

	VC_State_t StateVC;
	int16_t hTref;
	int32_t wSpeedM1;
	int32_t wSpeedM2;
	uint16_t hFaultCode;
	bool bStartupSuccess;
	uint32_t wReturn2StandbyCounter;

	VC_Handle_t * pVControl = &VControl;
	Throttle_Handle_t * pThrottleSensor = VControl.pThrottle;
	Brake_Handle_t * pBrakeSensor = VControl.pBrake;
	TD_Handle_t * pTorqueDistributor = VControl.pTorqueDistributor;
	MD_Handle_t * pMDrive1 = &(VControl.pMDComm->pMD[M1]);
	#ifdef USE_MOTOR2
	MD_Handle_t * pMDrive2 = &(VControl.pMDComm->pMD[M2]);
	#endif
	
	STMTimeoutTimer = osTimerNew(&STMTimeoutCallback, osTimerPeriodic, NULL, &TmrAtt_STMTimeoutTimer);
	uint32_t xLastWakeTime = osKernelGetTickCount();
	
	while (true)
	{
		Throttle_CalcAvValue(pThrottleSensor);
		hTref = Throttle_CalcIqref(pThrottleSensor);
		hTref = Brake_CalcIqref(pBrakeSensor, hTref);
		TD_DistributeTorque(pTorqueDistributor, hTref);
		wSpeedM1 = VC_getMotorSpeedMeas(pVControl, M1);
		wSpeedM2 = VC_getMotorSpeedMeas(pVControl, M2);
		
		if ( md_isErrorOccured() )
		{
			VCSTM_FaultProcessing( VControl.pSTM, VC_MC_COMM_ERROR, 0 );
		}
		
		if( CAN_queueIsFull (&VControl))
		{
			VCSTM_FaultProcessing( VControl.pSTM, VC_CAN_QUEUE_FULL, 0 );
		}
		
		StateVC = VCSTM_GetState( VControl.pSTM );
		switch ( StateVC )
		{
			case V_IDLE:
					VCSTM_NextState( VControl.pSTM, V_STANDBY );
					break;
			
			case V_STANDBY:
					if ( abs( TD_GetTorqueMainMotor(pTorqueDistributor) ) > MINIMUM_STARTING_TORQUE )
					{
						VCSTM_NextState( VControl.pSTM, V_STANDBY_START );
					}
					break;
			
			case V_STANDBY_START:
					bStartupSuccess = false;
					flagTimeout = false;
					wReturn2StandbyCounter = 0;
					#ifdef USE_MOTOR1
					md_setTorqueRamp(M1, TD_GetTorqueM1(pTorqueDistributor), 0);
					md_startMotor(M1);
					#endif
					#ifdef USE_MOTOR2
					md_setTorqueRamp(M2, TD_GetTorqueM2(pTorqueDistributor), 0);
					md_startMotor(M2);
					#endif
					osTimerStart(STMTimeoutTimer, 200);
					VCSTM_NextState( VControl.pSTM, V_START_FORWARD );
					break;
			
			case V_START_FORWARD:
					bStartupSuccess = true;
					#ifdef USE_MOTOR1
					switch ( pMDrive1->MDStateMachine.bMState )
					{
						case M_RUN:
							bStartupSuccess &= true;
							break;
						case M_FAULT_NOW: 
						case M_FAULT_OVER:
							bStartupSuccess = false;
							osTimerStop(STMTimeoutTimer);
							VCSTM_FaultProcessing( VControl.pSTM, VC_M1_FAULTS, 0 );
							break;
						default:
							bStartupSuccess = false;
							break;
					}
					#endif
					#ifdef USE_MOTOR2
					switch ( pMDrive2->MDStateMachine.bMState )
					{
						case M_RUN:
							bStartupSuccess &= true;
							break;
						case M_FAULT_NOW:
						case M_FAULT_OVER:
							bStartupSuccess = false;
							osTimerStop(STMTimeoutTimer);
							VCSTM_FaultProcessing( VControl.pSTM, VC_M2_FAULTS, 0 );
							break;
						default:
							bStartupSuccess = false;
							break;
					}
					#endif
					if ( flagTimeout )
					{
						bStartupSuccess = false;
						flagTimeout = false;
						if(osTimerIsRunning(STMTimeoutTimer))
							 osTimerStop(STMTimeoutTimer);
						VCSTM_FaultProcessing( VControl.pSTM, VC_STARTUP_TIMEOUT, 0 );
					}
					if ( bStartupSuccess )
					{
						osTimerStop(STMTimeoutTimer);
						VCSTM_NextState( VControl.pSTM, V_RUN_FORWARD );
					}
					break;
			
			case V_RUN_FORWARD:
					#ifdef USE_MOTOR1
					switch ( pMDrive1->MDStateMachine.bMState )
					{
						case M_RUN:
							md_setTorqueRamp(M1, TD_GetTorqueM1(pTorqueDistributor), 0);
							break;
						case M_FAULT_NOW:
						case M_FAULT_OVER:
							VCSTM_FaultProcessing( VControl.pSTM, VC_M1_FAULTS, 0 );
							break;
						default:
							VCSTM_FaultProcessing( VControl.pSTM, VC_M1_UNEXPECTED_BEHAVIOR, 0 );
							break;
					}
					#endif
					#ifdef USE_MOTOR2
					switch ( pMDrive2->MDStateMachine.bMState )
					{
						case M_RUN:
							md_setTorqueRamp(M2, TD_GetTorqueM2(pTorqueDistributor), 0);
							break;
						case M_FAULT_NOW:
						case M_FAULT_OVER:
							VCSTM_FaultProcessing( VControl.pSTM, VC_M2_FAULTS, 0 );
							break;
						default:
							VCSTM_FaultProcessing( VControl.pSTM, VC_M2_UNEXPECTED_BEHAVIOR, 0 );
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
						
						VCSTM_NextState( VControl.pSTM, V_STANDBY );
					}
					break;
					
			case V_FAULT_NOW:
					#ifdef USE_MOTOR1
					md_stopMotor(M1);
					#endif
					#ifdef USE_MOTOR2
					md_stopMotor(M2);
					#endif
					hFaultCode = VControl.pSTM->hVFaultNow;
					VCSTM_FaultProcessing( VControl.pSTM, 0, hFaultCode );
					break;
			case V_FAULT_OVER:
					#ifdef USE_MOTOR1
					md_stopMotor(M1);
					#endif
					#ifdef USE_MOTOR2
					md_stopMotor(M2);
					#endif
					break;
			
			default:
				break;
		}
		
		#ifdef USE_MOTOR1
		getMonitoringReg_Fast(M1);
		#endif
		#ifdef USE_MOTOR2
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
	
	uint32_t xLastWakeTime = osKernelGetTickCount();
																				
	while (true)
	{
		#ifdef USE_MOTOR1
		getMonitoringReg_Slow(M1);
		sendDiagnosticCANmessages(M1);
		#endif
		#ifdef USE_MOTOR2
		getMonitoringReg_Slow(M2);
		sendDiagnosticCANmessages(M2);
		#endif
/**************** For Testing purposes ****************/
//		CAN_SendDummyMsg();
/******************************************************/
		xLastWakeTime += TASK_VCSLOWLOOP_SAMPLE_TIME_TICK;
		osDelayUntil(xLastWakeTime);
	}
}

static void sendDiagnosticCANmessages(uint8_t motorSelection)
{
	if(motorSelection == M1)
	{
		CAN_SendStatus(&VControl, M_NONE); // For Vehicle state
		CAN_SendThrottleBrake(&VControl);
		CAN_SendVbus(&VControl);
	}
	
	CAN_SendSpeed(&VControl, motorSelection);
	CAN_SendStatus(&VControl, motorSelection);
	CAN_SendCurrent(&VControl, motorSelection);
	CAN_SendTemperature(&VControl, motorSelection);
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

