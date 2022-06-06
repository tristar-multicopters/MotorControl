/**
  * @file    vc_tasks.c
  * @brief   This module gather all tasks of vehicule control application
  *
	*/

#include "vc_tasks.h"
#include "mc_tasks.h"
#include "vc_config.h"

#include "gnr_main.h"


/************* DEBUG ****************/

#if SWD_CONTROL_ENABLE
struct {
	qd_t hIqdref;
	int32_t hSpeedRef;
	bool bStartM1;
	bool bStartM2;
} sDebugVariables;
#endif

/************* DEFINES ****************/

#define TASK_VCFASTLOOP_SAMPLE_TIME_TICK 	20 		/* VC_FastLoop execute every 20 ticks */
#define TASK_VCSLOWLOOP_SAMPLE_TIME_TICK 	200		/* VC_FastLoop execute every 200 ticks */
#define TASK_VCSTM_SAMPLE_TIME_TICK				20		/* VC_FastLoop execute every 20 ticks */
#define RETURN_TO_STANDBY_LOOPTICKS 0		// Max number of ticks to stay in run while stop conditions are met
#define START_MOTORS_LOOPTICKS 			2		// Max number of ticks to stay in run while stop conditions are met
#define START_LOOPTICKS							100 // Max number of ticks to stay in start state
#define STOP_LOOPTICKS							100 // Max number of ticks to stay in stop state


/************* TASKS ****************/


/**
  * @brief  It initializes the vehicle control application. Needs to be called before using
	*					vehicle control related modules.
  * @retval None
  */
void VC_BootUp(void)
{	
	VCI_Handle_t * pVCI = &VCInterfaceHandle;
	
	/* Initialize vehicle controller components */
	VCSTM_Init(pVCI->pStateMachine);
	PWRT_Init(pVCI->pPowertrain, &MCInterface[M1]);
}

/**
  * @brief  Task to periodically update powertrain components.
  * @retval None
  */
__NO_RETURN void THR_VC_MediumFreq (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	VCI_Handle_t * pVCI = &VCInterfaceHandle;
	
	uint32_t xLastWakeTime = osKernelGetTickCount();
	while (true)
	{
//		PWRT_CalcTorqueSpeed(pVCI->pPowertrain);
		
		xLastWakeTime += TASK_VCFASTLOOP_SAMPLE_TIME_TICK;
		osDelayUntil(xLastWakeTime);
	}
}


// Flowchart describing how this state machine works avaialble on the FTEX drive
// https://drive.google.com/file/d/1PoqcsODmJ9KeqMJPuf6NUA2KDo6AWH9R/view?usp=sharing
__NO_RETURN void THR_VC_StateMachine (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	VCI_Handle_t * pVCI = &VCInterfaceHandle;
	
//	#if SWD_CONTROL_ENABLE
//	sDebugVariables.bStartM1 = false;
//	sDebugVariables.bStartM2 = false;
//	sDebugVariables.hIqdref.q = 0;
//	sDebugVariables.hIqdref.d = 0;
//	sDebugVariables.hSpeedRef = 0;
//	#else
//	uint32_t wCounter;
//	uint16_t hVehicleFault;
//	VC_State_t StateVC;
//	#endif
	
	uint32_t xLastWakeTime = osKernelGetTickCount();
	while (true)
	{	
//		#if SWD_CONTROL_ENABLE
//		MDI_SetCurrentReferences(pVCI->pPowertrain->pMDI, M2, sDebugVariables.hIqdref);
//		
//		sDebugVariables.bStartM1 ? MDI_StartMotor(pVCI->pPowertrain->pMDI, M1) : MDI_StopMotor(pVCI->pPowertrain->pMDI, M1);
//		sDebugVariables.bStartM2 ? MDI_StartMotor(pVCI->pPowertrain->pMDI, M2) : MDI_StopMotor(pVCI->pPowertrain->pMDI, M2);
//		#else
//		StateVC = VCSTM_GetState( pVCI->pStateMachine );
//		switch ( StateVC )
//		{
//			case V_IDLE:
//					osDelay(100);
//					wCounter = 0;
//					VCSTM_NextState( pVCI->pStateMachine, V_STANDBY );
//					break;
//			
//			case V_STANDBY:
//					hVehicleFault = PWRT_StandbyStateCheck(pVCI->pPowertrain);
//					VCSTM_FaultProcessing( pVCI->pStateMachine, hVehicleFault, 0 );
//					if ( PWRT_CheckStartConditions(pVCI->pPowertrain) )
//					{
//						wCounter++;
//					}
//					else
//					{
//						wCounter = 0;
//					}
//					if (wCounter > START_MOTORS_LOOPTICKS)
//					{
//						wCounter = 0;
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
//					PWRT_UpdateMotorRamps(pVCI->pPowertrain);
//					PWRT_StartMotors(pVCI->pPowertrain);
//					hVehicleFault = PWRT_StartStateCheck(pVCI->pPowertrain);
//					VCSTM_FaultProcessing( pVCI->pStateMachine, hVehicleFault, 0 );
//					if ( PWRT_IsPowertrainActive(pVCI->pPowertrain) )
//					{
//						wCounter = 0;
//						VCSTM_NextState( pVCI->pStateMachine, V_RUN );
//					}
//					wCounter++;
//					if ( wCounter > START_LOOPTICKS )
//					{
//						wCounter = 0;
//						VCSTM_FaultProcessing( pVCI->pStateMachine, VC_START_TIMEOUT, 0 );
//					}
//					break;
//					
//			case V_RUN:
//					hVehicleFault = PWRT_RunStateCheck(pVCI->pPowertrain);
//					VCSTM_FaultProcessing( pVCI->pStateMachine, hVehicleFault, 0 );
//					PWRT_UpdateMotorRamps(pVCI->pPowertrain); 			
//					if ( PWRT_CheckStopConditions(pVCI->pPowertrain) )
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
//					PWRT_StopMotors(pVCI->pPowertrain);
//					hVehicleFault = PWRT_StopStateCheck(pVCI->pPowertrain);
//					VCSTM_FaultProcessing( pVCI->pStateMachine, hVehicleFault, 0 );
//					if ( PWRT_IsPowertrainStopped(pVCI->pPowertrain) )
//					{
//						wCounter = 0;
//						VCSTM_NextState( pVCI->pStateMachine, V_IDLE );
//					}
//					wCounter++;
//					if ( wCounter > STOP_LOOPTICKS )
//					{
//						wCounter = 0;
//						VCSTM_FaultProcessing( pVCI->pStateMachine, VC_STOP_TIMEOUT, 0 );
//					}
//					break;
//					
//			case V_FAULT_NOW:
//					PWRT_StopMotors(pVCI->pPowertrain);
//					if ( PWRT_IsPowertrainStopped(pVCI->pPowertrain) )
//					{
//						if ( !PWRT_MotorFaultManagement(pVCI->pPowertrain) )
//						{
//							VCSTM_FaultProcessing( pVCI->pStateMachine, 0, VC_M1_FAULTS ); // Remove VC_M1_FAULTS flag
//							VCSTM_FaultProcessing( pVCI->pStateMachine, 0, VC_M2_FAULTS ); // Remove VC_M2_FAULTS flag
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
//		#endif
	
		xLastWakeTime += TASK_VCSTM_SAMPLE_TIME_TICK;
		osDelayUntil(xLastWakeTime);
	}
}
