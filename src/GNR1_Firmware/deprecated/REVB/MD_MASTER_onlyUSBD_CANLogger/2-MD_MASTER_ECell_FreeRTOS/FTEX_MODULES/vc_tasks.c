/**
  ******************************************************************************
  * @file    vc_tasks.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module gathers tasks of vehicule controller
  *
	******************************************************************************
	*/

#include "vc_tasks.h"

TaskHandle_t TSK_FastLoopMD_handle;
TaskHandle_t TSK_SlowLoopMD_handle;

static void getMonitoringReg_Fast(uint8_t motorSelection);
static void getMonitoringReg_Slow(uint8_t motorSelection);
static void STMTimeoutCallback();
//static void sendDiagnosticCANmessages(void);

static TimerHandle_t STMTimeoutTimer;
static bool flagTimeout = false;

#define MINIMUM_STARTING_TORQUE 2000
#define MAX_STARTUP_TIME 10000

//static void VC_clock_hfck_rdy( nrf_drv_clock_evt_type_t event )
//{
////	switch(event)
////	{
////		case NRF_DRV_CLOCK_EVT_HFCLK_STARTED:
////			break;
////		case NRF_DRV_CLOCK_EVT_LFCLK_STARTED:
////			break;
////		case NRF_DRV_CLOCK_EVT_CAL_DONE:
////			break;
////		case NRF_DRV_CLOCK_EVT_CAL_ABORTED:
////			break;
////		default:
////			break;
////	}
//}

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
	APP_ERROR_CHECK( nrf_drv_gpiote_init() );
	VC_init_clock();

	VControl.pMDComm->pMD[M1] = MotorDrive1;
	VControl.pMDComm->pMD[M2] = MotorDrive2;
	VControl.isLocked = false;
	Brake_Init(VControl.pBrake);
	VCSTM_Init(VControl.pSTM);
	md_comm_init(VControl.pMDComm);  
	Throttle_Init(VControl.pThrottle);
	TD_Init(VControl.pTorqueDistributor);
	RCM_Init(VControl.pRegularConvertionManager);
  SPI_Init(VControl.pCANbusManager->pMCP->pSPI);
	CAN_Init(VControl.pCANbusManager);
}

void TSK_FastLoopMD (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	vTaskDelay(250);
	VC_State_t StateVC;
	int16_t hTref;
	uint16_t hFaultCode;
	bool bStartupSuccess;
	
	Throttle_Handle_t * pThrottleSensor = VControl.pThrottle;
	Brake_Handle_t * pBrakeSensor = VControl.pBrake;
	TD_Handle_t * pTorqueDistributor = VControl.pTorqueDistributor;
	MD_Handle_t * pMDrive1 = &(VControl.pMDComm->pMD[M1]);
	#ifdef USE_TWO_MOTORS
	MD_Handle_t * pMDrive2 = &(VControl.pMDComm->pMD[M2]);
	#endif
	STMTimeoutTimer = xTimerCreate("STMTimeout", MAX_STARTUP_TIME, pdFALSE, 0, STMTimeoutCallback);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	while (true)
	{
		Throttle_CalcAvValue(pThrottleSensor);
		hTref = Throttle_CalcIqref(pThrottleSensor);
		hTref = Brake_CalcIqref(pBrakeSensor, hTref);
		TD_DistributeTorque(pTorqueDistributor, hTref);
		
		if ( md_isErrorOccured() )
		{
			VCSTM_FaultProcessing( VControl.pSTM, VC_MC_COMM_ERROR, 0 );
		}
		
		StateVC = VCSTM_GetState( VControl.pSTM );
		switch ( StateVC )
		{
			case V_IDLE:
					VCSTM_NextState( VControl.pSTM, V_STANDBY );
					break;
			
			case V_STANDBY:
					if ( abs( TD_GetTorqueM1(pTorqueDistributor) ) > MINIMUM_STARTING_TORQUE )
					{
						VCSTM_NextState( VControl.pSTM, V_STANDBY_START );
					}
					break;
			
			case V_STANDBY_START:
					bStartupSuccess = false;
					flagTimeout = false;
					md_setTorqueRamp(M1, TD_GetTorqueM1(pTorqueDistributor), 0);
					md_startMotor(M1);
					#ifdef USE_TWO_MOTORS
					md_setTorqueRamp(M2, TD_GetTorqueM2(pTorqueDistributor), 0);
					md_startMotor(M2);
					#endif
					xTimerStart(STMTimeoutTimer, 0);
					VCSTM_NextState( VControl.pSTM, V_START_FORWARD );
					break;
			
			case V_START_FORWARD:
					switch ( pMDrive1->MDStateMachine.bMState )
					{
						case M_RUN:
							bStartupSuccess = true;
							break;
						case M_FAULT_NOW: 
						case M_FAULT_OVER:
							bStartupSuccess = false;
							xTimerStop(STMTimeoutTimer, 0);
							VCSTM_FaultProcessing( VControl.pSTM, VC_M1_FAULTS, 0 );
							break;
						default:
							bStartupSuccess = false;
							break;
					}
					#ifdef USE_TWO_MOTORS
					switch ( pMDrive2->MDStateMachine.bMState )
					{
						case M_RUN:
							//bStartupSuccess = true;
							bStartupSuccess &= true;
							break;
						case M_FAULT_NOW:
						case M_FAULT_OVER:
							bStartupSuccess = false;
							xTimerStop(STMTimeoutTimer, 0);
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
						VCSTM_FaultProcessing( VControl.pSTM, VC_STARTUP_TIMEOUT, 0 );
					}
					if ( bStartupSuccess )
					{
						xTimerStop(STMTimeoutTimer, 0);
						VCSTM_NextState( VControl.pSTM, V_RUN_FORWARD );
					}
					break;
			
			case V_RUN_FORWARD:
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
					#ifdef USE_TWO_MOTORS
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
					break;
					
			case V_FAULT_NOW:
					md_stopMotor(M1);
					#ifdef USE_TWO_MOTORS
					md_stopMotor(M2);
					#endif
					hFaultCode = VControl.pSTM->hVFaultNow;
					VCSTM_FaultProcessing( VControl.pSTM, 0, hFaultCode );
					break;
			case V_FAULT_OVER:
					md_stopMotor(M1);
					#ifdef USE_TWO_MOTORS
					md_stopMotor(M2);
					#endif
					break;
			
			default:
				break;
		}
		
		getMonitoringReg_Fast(M1);
		#ifdef USE_TWO_MOTORS
		getMonitoringReg_Fast(M2);
		#endif
		vTaskDelayUntil( &xLastWakeTime, TASK_VCFASTLOOP_SAMPLE_TIME_TICK );
	}
}

void TSK_SlowLoopMD (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	vTaskDelay(250);

	//*******************************************//
	TickType_t xLastWakeTime = xTaskGetTickCount();
																				
	while (true)
	{
		int32_t speed_ref, speed_meas, vbus, vc_statusfaults, mc_statusfaults,
						temperature, throttle_brake;
		int16_t iq_ref, id_ref, iq_meas, id_meas;
		
		if(!VControl.pCANbusManager->pMCP->ongoing_transfer)
		{
			VControl.pCANbusManager->pMCP->ongoing_transfer = true;
			MCP_CAN_id_t currentCANId = VControl.pCANbusManager->current_ID_msg;
			switch (currentCANId)
			{
				case CAN_STATUS_VC_ID:
					vc_statusfaults = VControl.pSTM->bVState     << 16 |
																			VControl.pSTM->hVFaultNow  << 8  |
																			VControl.pSTM->hVFaultOccurred;
					CAN_SendStatus(currentCANId, vc_statusfaults);
					break;
				case CAN_THROTTLE_ID:
					throttle_brake = VControl.pBrake->isPressed << 16 | VControl.pThrottle->hAvThrottle;
					CAN_SendThrottleBrake(currentCANId, throttle_brake);
					break;
				case CAN_VBUS_ID:
					vbus = VControl.pMDComm->pMD[M1].MDMeas.bus_voltage_mes;
					CAN_SendVbus(currentCANId, vbus);
					break;
				case CAN_STATUS_M1_ID:
					mc_statusfaults = VControl.pMDComm->pMD[M1].MDStateMachine.bMState              |
																	 VControl.pMDComm->pMD[M1].MDStateMachine.hMFaultNow      << 8 |
																	 VControl.pMDComm->pMD[M1].MDStateMachine.hMFaultOccurred << 16;
					CAN_SendStatus(currentCANId, mc_statusfaults);
					break;
				case CAN_CURRENT_M1_ID:
					iq_ref  = VControl.pMDComm->pMD[M1].MDRTParam.iq_ref;
					id_ref  = VControl.pMDComm->pMD[M1].MDRTParam.id_ref;
					iq_meas = VControl.pMDComm->pMD[M1].MDMeas.iq_mes;
					id_meas = VControl.pMDComm->pMD[M1].MDMeas.id_mes;
					CAN_SendCurrent(currentCANId, iq_ref, id_ref, iq_meas, id_meas);
					break;
				case CAN_SPEED_M1_ID:
					speed_ref  = VControl.pMDComm->pMD[M1].MDRTParam.speed_ref;
					speed_meas = VControl.pMDComm->pMD[M1].MDMeas.speed_mes;
					CAN_SendSpeed(currentCANId, speed_ref, speed_meas);
					break;
				case CAN_TEMPERATURE_M1_ID:
					temperature = VControl.pMDComm->pMD[M1].MDMeas.temp_u;
					CAN_SendTemperature(currentCANId, temperature);
					break;
				#ifdef USE_TWO_MOTORS
				case CAN_STATUS_M2_ID:
					mc_statusfaults = VControl.pMDComm->pMD[M2].MDStateMachine.bMState           |
												 VControl.pMDComm->pMD[M2].MDStateMachine.hMFaultNow      << 8 |
												 VControl.pMDComm->pMD[M2].MDStateMachine.hMFaultOccurred << 16;
					CAN_SendStatus(currentCANId, mc_statusfaults);
					break;
				case CAN_CURRENT_M2_ID:
					iq_ref  = VControl.pMDComm->pMD[M2].MDRTParam.iq_ref;
					id_ref  = VControl.pMDComm->pMD[M2].MDRTParam.id_ref;
					iq_meas = VControl.pMDComm->pMD[M2].MDMeas.iq_mes;
					id_meas = VControl.pMDComm->pMD[M2].MDMeas.id_mes;
					CAN_SendCurrent(currentCANId, iq_ref, id_ref, iq_meas, id_meas);
					break;
				case CAN_SPEED_M2_ID:
					speed_ref  = VControl.pMDComm->pMD[M2].MDRTParam.speed_ref;
					speed_meas = VControl.pMDComm->pMD[M2].MDMeas.speed_mes;
					CAN_SendSpeed(currentCANId, speed_ref, speed_meas);
					break;
				case CAN_TEMPERATURE_M2_ID:
					temperature = VControl.pMDComm->pMD[M2].MDMeas.temp_u;
					CAN_SendTemperature(currentCANId, temperature);
					break;
				#endif
				default:
					break;
			}
		}
		getMonitoringReg_Slow(M1);
		#ifdef USE_TWO_MOTORS
		getMonitoringReg_Slow(M2);
		#endif
		if(!nrf_queue_is_empty(VControl.pCANbusManager->pMCP->MCP_queue))
		{
			CAN_Message_t message;
			if(NRF_SUCCESS == MCP25625_popCANmsg(&message))
				CAN_ProcessRXMsg(message);
		}
		//sendDiagnosticCANmessages();
		vTaskDelayUntil( &xLastWakeTime, TASK_VCSLOWLOOP_SAMPLE_TIME_TICK );
	}
}

//static void sendDiagnosticCANmessages(void)
//{
//	int32_t mc_statusfaults, vbus, speed_ref, speed_meas, temperature, throttle_brake, vc_statusfaults;
//	int16_t iq_ref, id_ref, iq_meas, id_meas;
//	
//	mc_statusfaults = VControl.pMDComm->pMD[M1].MDStateMachine.bMState              |
//												   VControl.pMDComm->pMD[M1].MDStateMachine.hMFaultNow      << 8 |
//												   VControl.pMDComm->pMD[M1].MDStateMachine.hMFaultOccurred << 16;
//	vbus = VControl.pMDComm->pMD[M1].MDMeas.bus_voltage_mes;
//	iq_ref  = VControl.pMDComm->pMD[M1].MDRTParam.iq_ref;
//	id_ref  = VControl.pMDComm->pMD[M1].MDRTParam.id_ref;
//	iq_meas = VControl.pMDComm->pMD[M1].MDMeas.iq_mes;
//	id_meas = VControl.pMDComm->pMD[M1].MDMeas.id_mes;
//	speed_ref  = VControl.pMDComm->pMD[M1].MDRTParam.speed_ref;
//	speed_meas = VControl.pMDComm->pMD[M1].MDMeas.speed_mes;
//	temperature = VControl.pMDComm->pMD[M1].MDMeas.temp_u;
//	
//	throttle_brake = VControl.pBrake->isPressed << 16 | VControl.pThrottle->hAvThrottle;
//	vc_statusfaults = VControl.pSTM->bVState     << 16 |
//														  VControl.pSTM->hVFaultNow  << 8  |
//														  VControl.pSTM->hVFaultOccurred;
//		
//	CAN_SendThrottleBrake(M1, throttle_brake);
//	CAN_SendStatus(M_NONE, vc_statusfaults);
//	
//	CAN_SendStatus(M1, mc_statusfaults);
//	CAN_SendVbus(M1, vbus);
//  CAN_SendCurrent(M1, iq_ref, id_ref, iq_meas, id_meas);
//  CAN_SendSpeed(M1, speed_ref, speed_meas);
//  CAN_SendTemperature(M1, temperature);
//	
//	#ifdef USE_TWO_MOTORS
//	mc_statusfaults = VControl.pMDComm->pMD[M2].MDStateMachine.bMState              |
//												   VControl.pMDComm->pMD[M2].MDStateMachine.hMFaultNow      << 8 |
//												   VControl.pMDComm->pMD[M2].MDStateMachine.hMFaultOccurred << 16;
//	
//	iq_ref  = VControl.pMDComm->pMD[M2].MDRTParam.iq_ref;
//	id_ref  = VControl.pMDComm->pMD[M2].MDRTParam.id_ref;
//	iq_meas = VControl.pMDComm->pMD[M2].MDMeas.iq_mes;
//	id_meas = VControl.pMDComm->pMD[M2].MDMeas.id_mes;
//	speed_ref  = VControl.pMDComm->pMD[M2].MDRTParam.speed_ref;
//	speed_meas = VControl.pMDComm->pMD[M2].MDMeas.speed_mes;
//	temperature = VControl.pMDComm->pMD[M2].MDMeas.temp_u;
//		
//	CAN_SendStatus(M2, mc_statusfaults);
//  CAN_SendCurrent(M2, iq_ref, id_ref, iq_meas, id_meas);
//  CAN_SendSpeed(M2, speed_ref, speed_meas);
//  CAN_SendTemperature(M2, temperature);
//	#endif
//}

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

static void STMTimeoutCallback()
{
	flagTimeout = true;
	xTimerStop(STMTimeoutTimer, 0);
}	


