/**
  ******************************************************************************
  * @file    drivetrain_management.c
  * @author  Sami Bouzid, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   This module handles management of the vehicle drivetrain
  *
	******************************************************************************
*/

#include "drivetrain_management.h"

#define OVERCURRENT_COUNTER 		0
#define STARTUP_COUNTER 				1
#define SPEEDFEEDBACK_COUNTER 	2

/* Functions ---------------------------------------------------- */

/**
	* @brief  Module initialization, to be called once before using it
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_Init(DRVT_Handle_t * pHandle)
{
	MDI_Init(pHandle->pMDI);
	THRO_Init(pHandle->pThrottle);
	BRK_Init(pHandle->pBrake);
	PAS_Init(pHandle->pPAS);
	MS_Init(pHandle->pMS);
	PWREN_Init(pHandle->pPWREN);
	
	pHandle->aTorque[M1] = 0; pHandle->aTorque[M2] = 0;
	pHandle->aSpeed[M1] = 0; pHandle->aSpeed[M2] = 0;
	pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
	pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] = 0;
	pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] = 0;

}

/**
	* @brief  Compute target torque and speed to be applied to each motors. To be called periodically.
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_CalcTorqueSpeed(DRVT_Handle_t * pHandle)
{
	bool bIsBrakePressed = BRK_IsPressed(pHandle->pBrake);
	bool bIsPwrEnabled = PWREN_IsPowerEnabled(pHandle->pPWREN);
	int32_t wSpeedM1 = MDI_getSpeed(pHandle->pMDI, M1);
	int32_t wSpeedM2 = MDI_getSpeed(pHandle->pMDI, M2);
	int32_t wSpeedMainMotor = MDI_getSpeed(pHandle->pMDI, pHandle->bMainMotor);
	int16_t hTempHeatSnkM1 = MDI_getHeatsinkTemp(pHandle->pMDI, M1);
	int16_t hTempHeatSnkM2 = MDI_getHeatsinkTemp(pHandle->pMDI, M2);
	int16_t hTempHeatSnkMainMotor = MDI_getHeatsinkTemp(pHandle->pMDI, pHandle->bMainMotor);
	
	MotorSelection_t bMotorSelection = MS_CheckSelection(pHandle->pMS);
	int16_t hTorqueRef = 0; int32_t hSpeedRef = 0;
	int16_t hAux = 0;
	
	if (pHandle->pMS->bMSEnable)
	{
		switch (bMotorSelection)
		{
			case M1_SELECTED:
				pHandle->sParameters.bMode = SINGLE_MOTOR;
				pHandle->bMainMotor = M1;
				break;
			case M2_SELECTED:
				pHandle->sParameters.bMode = SINGLE_MOTOR;
				pHandle->bMainMotor = M2;
				break;
			case ALL_MOTOR_SELECTED:
				pHandle->sParameters.bMode = DUAL_MOTOR;
				pHandle->bMainMotor = pHandle->sParameters.bDefaultMainMotor;
				break;
			default:
				break;
		}
	}
	THRO_CalcAvThrottleValue(pHandle->pThrottle);
	PAS_CalcTSAvValue(pHandle->pPAS);

	pHandle->aTorque[M1] = 0; pHandle->aTorque[M2] = 0;
	pHandle->aSpeed[M1] = 0; pHandle->aSpeed[M2] = 0;

	if (pHandle->sParameters.bCtrlType == TORQUE_CTRL)
	{
		/* Compute torque to motor depending on either throttle or PAS  */
		hTorqueRef = DRVT_CalcSelectedTorque(pHandle);
		
		if ( bIsBrakePressed )
		{
			hTorqueRef = 0;
		}
		if(pHandle->sParameters.bMode == SINGLE_MOTOR)
		{
			/* Using PAS */
			if ( PAS_IsPASDetected(pHandle->pPAS) && !THRO_IsThrottleDetected(pHandle->pThrottle) )
			{
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sSpeedFoldback[pHandle->bMainMotor], hTorqueRef, abs(wSpeedMainMotor) );
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sHeatsinkTempFoldback[pHandle->bMainMotor], hAux, hTempHeatSnkMainMotor );
				pHandle->aTorque[pHandle->bMainMotor] = hAux;
			}
			/* Using throttle */
			else
			{
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sHeatsinkTempFoldback[pHandle->bMainMotor], hAux, hTempHeatSnkMainMotor );
				pHandle->aTorque[pHandle->bMainMotor] = hTorqueRef;
			}
		}
		if(pHandle->sParameters.bMode == DUAL_MOTOR)
		{
			if ( PAS_IsPASDetected(pHandle->pPAS) && !THRO_IsThrottleDetected(pHandle->pThrottle) )
			{
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sSpeedFoldback[M1], hTorqueRef, abs(wSpeedM1) );
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sHeatsinkTempFoldback[M1], hAux, hTempHeatSnkM1 );
				pHandle->aTorque[M1] = hAux;
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sSpeedFoldback[M2], hTorqueRef, abs(wSpeedM2) );
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sHeatsinkTempFoldback[M2], hAux, hTempHeatSnkM2 );
				pHandle->aTorque[M2] = pHandle->sParameters.bM2TorqueInversion ? -hAux : hAux;
			}
			else
			{
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sHeatsinkTempFoldback[M1], hTorqueRef, hTempHeatSnkM1 );
				pHandle->aTorque[M1] = hAux;
				hAux = FLDBK_ApplyTorqueLimitation( &pHandle->sHeatsinkTempFoldback[M2], hTorqueRef, hTempHeatSnkM2 );
				pHandle->aTorque[M2] = pHandle->sParameters.bM2TorqueInversion ? -hAux : hAux;
			}
		}
	}
	else if (pHandle->sParameters.bCtrlType == SPEED_CTRL)
	{
		/* SPEED CONTROL NOT IMPLEMENTED YET. JUST PLACEHOLDER CODE */
		hSpeedRef = THRO_ThrottleToSpeed(pHandle->pThrottle);
		if ( bIsBrakePressed )
		{
			hSpeedRef = 0;
		}
		
		if(pHandle->sParameters.bMode == SINGLE_MOTOR)
		{
			pHandle->aSpeed[pHandle->bMainMotor] = hSpeedRef;
		}
		if(pHandle->sParameters.bMode == DUAL_MOTOR)
		{
			pHandle->aSpeed[M1] = hSpeedRef;
			pHandle->aSpeed[M2] = hSpeedRef;
		}
	}
	else {}
}

/**
	* @brief  Send torque and/or speed ramp commands to motors
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_UpdateMotorRamps(DRVT_Handle_t * pHandle)
{
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		if (pHandle->sParameters.bCtrlType == TORQUE_CTRL)
		{
			if ( abs(pHandle->aTorque[M1]) > abs(MDI_getIq(pHandle->pMDI, M1)) )
			{
				if ( pHandle->pPAS->bPASDetected && !THRO_IsThrottleDetected(pHandle->pThrottle) )
				MDI_SetTorqueRamp(pHandle->pMDI, M1, pHandle->aTorque[M1], pHandle->sParameters.hTorquePASRampTimeUp);	
				else

				MDI_SetTorqueRamp(pHandle->pMDI, M1, pHandle->aTorque[M1], pHandle->sParameters.hTorqueRampTimeUp);
			}
			else
			{
				MDI_SetTorqueRamp(pHandle->pMDI, M1, pHandle->aTorque[M1], pHandle->sParameters.hTorqueRampTimeDown);
			}
		}
		else if (pHandle->sParameters.bCtrlType == SPEED_CTRL)
		{
			MDI_SetSpeedRamp(pHandle->pMDI, M1, pHandle->aSpeed[M1], pHandle->sParameters.hSpeedRampTimeUp);
		}
		else {}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		if (pHandle->sParameters.bCtrlType == TORQUE_CTRL)
		{
			if ( abs(pHandle->aTorque[M2]) > abs(MDI_getIq(pHandle->pMDI, M2)) )
			{
				MDI_SetTorqueRamp(pHandle->pMDI, M2, pHandle->aTorque[M2], pHandle->sParameters.hTorqueRampTimeUp);
			}
			else
			{
				MDI_SetTorqueRamp(pHandle->pMDI, M2, pHandle->aTorque[M2], pHandle->sParameters.hTorqueRampTimeDown);
			}
		}
		else if (pHandle->sParameters.bCtrlType == SPEED_CTRL)
		{
			MDI_SetSpeedRamp(pHandle->pMDI, M2, pHandle->aSpeed[M2], pHandle->sParameters.hSpeedRampTimeUp);
		}
		else {}
	}
}

/**
	* @brief  Send command to start motors
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_StartMotors(DRVT_Handle_t * pHandle)
{
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		MDI_StartMotor(pHandle->pMDI, M1);
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		MDI_StartMotor(pHandle->pMDI, M2);
	}
}

/**
	* @brief  Send command to stop motors
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_StopMotors(DRVT_Handle_t * pHandle)
{
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		MDI_StopMotor(pHandle->pMDI, M1);
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		MDI_StopMotor(pHandle->pMDI, M2);
	}
}

/**
	* @brief  Check for motor faults during standby state
	* @param  Drivetrain handle
	* @retval Vehicle fault code
	*/
uint16_t DRVT_StandbyStateCheck(DRVT_Handle_t * pHandle)
{
	uint16_t hVehicleFault = 0;
	
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M1) )
		{
			case M_IDLE:
			case M_STOP:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M1_FAULTS;
				break;
			default:
				hVehicleFault |= VC_M1_UNEXPECTED_BEHAVIOR;
				break;
		}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M2) )
		{
			case M_IDLE:
			case M_STOP:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M2_FAULTS;
				break;
			default:
				hVehicleFault |= VC_M2_UNEXPECTED_BEHAVIOR;
				break;
		}
	}
	
	return hVehicleFault;
}

/**
	* @brief  Check for motor faults during start state
	* @param  Drivetrain handle
	* @retval Vehicle fault code
	*/
uint16_t DRVT_StartStateCheck(DRVT_Handle_t * pHandle)
{
	uint16_t hVehicleFault = 0;
	
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M1) )
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M1_FAULTS;
				break;
			default:
				break;
		}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M2) )
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M2_FAULTS;
				break;
			default:
				break;
		}
	}
	
	return hVehicleFault;
}

/**
	* @brief  Check for motor faults during run state
	* @param  Drivetrain handle
	* @retval Vehicle fault code
	*/
uint16_t DRVT_RunStateCheck(DRVT_Handle_t * pHandle)
{
	uint16_t hVehicleFault = 0;
	
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M1) )
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M1_FAULTS;
				break;
			default:
				hVehicleFault |= VC_M1_UNEXPECTED_BEHAVIOR;
				break;
		}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M2) )
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M2_FAULTS;
				break;
			default:
				hVehicleFault |= VC_M2_UNEXPECTED_BEHAVIOR;
				break;
		}
	}
	
	return hVehicleFault;
}

/**
	* @brief  Check for motor faults during stop state
	* @param  Drivetrain handle
	* @retval Vehicle fault code
	*/
uint16_t DRVT_StopStateCheck(DRVT_Handle_t * pHandle)
{
	uint16_t hVehicleFault = 0;
	
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M1) )
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M1_FAULTS;
				break;
			default:
				break;
		}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M2) )
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M2_FAULTS;
				break;
			default:
				break;
		}
	}
	
	return hVehicleFault;
}

/**
	* @brief  Check if drivetrain is active (e.g. motors are in running state)
	* @param  Drivetrain handle
	* @retval Returns true if drivetrain is active
	*/
bool DRVT_IsDrivetrainActive(DRVT_Handle_t * pHandle)
{
	bool bActive = true;
	
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		if ( MDI_getState(pHandle->pMDI, M1) != M_RUN )
		{
			bActive = false;
		}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		if ( MDI_getState(pHandle->pMDI, M2) != M_RUN )
		{
			bActive = false;
		}
	}
	return bActive;
}

/**
	* @brief  Check if drivetrain is stopped (e.g. motors are in idle state)
	* @param  Drivetrain handle
	* @retval Returns true if drivetrain is stopped
	*/
bool DRVT_IsDrivetrainStopped(DRVT_Handle_t * pHandle)
{
	bool bStopped = true;
	
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M1) )
		{
			case M_IDLE:
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				break;
			default:
				bStopped = false;
				break;
		}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		switch ( MDI_getState(pHandle->pMDI, M2) )
		{
			case M_IDLE:
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				break;
			default:
				bStopped = false;
				break;
		}
	}
	return bStopped;
}

/**
	* @brief  Check if conditions to stop drivetrain are met
	* @param  Drivetrain handle
	* @retval Returns true if drivetrain conditions to stop are met, false otherwise
	*/
bool DRVT_CheckStopConditions(DRVT_Handle_t * pHandle)
{
	bool bCheckStop1 = false;
	bool bCheckStop2 = false;
	bool bCheckStop3 = false;
	bool bCheckStop4 = false;
	bool bCheckStop5 = false;
	bool bCheckStop6 = false;
	int32_t wSpeedM1 = MDI_getSpeed(pHandle->pMDI, M1);
	int32_t wSpeedM2 = MDI_getSpeed(pHandle->pMDI, M2);
	uint16_t hThrottleValue = THRO_GetAvThrottleValue(pHandle->pThrottle);
	
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		if ( abs(wSpeedM1) <= pHandle->sParameters.hStoppingSpeed )
		{
			bCheckStop1 = true;
		}
	}
	else
	{
		bCheckStop1 = true;
	}
	
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		if ( abs(wSpeedM2) <= pHandle->sParameters.hStoppingSpeed )
		{
			bCheckStop2 = true;
		}
	}
	else
	{
		bCheckStop2 = true;
	}
	
	if ( !PWREN_IsPowerEnabled(pHandle->pPWREN) )
	{
		bCheckStop3 = true;
	}	
	
	if ( BRK_IsPressed(pHandle->pBrake) )
	{
		bCheckStop4 = true;
	}
	
	if (!pHandle->pPAS->bPASDetected)
	{
		bCheckStop5 = true;
	}     

	if (hThrottleValue < pHandle->sParameters.hStoppingThrottle)
	{
		bCheckStop6 = true;
	}    

	return (bCheckStop1 & bCheckStop2 & bCheckStop5 & bCheckStop6) | bCheckStop3 | bCheckStop4;
}

/**
	* @brief  Check if conditions to start drivetrain are met
	* @param  Drivetrain handle
	* @retval Returns true if drivetrain can be started
	*/
bool DRVT_CheckStartConditions(DRVT_Handle_t * pHandle)
{
	bool bCheckStart = false;
	uint16_t hThrottleValue = THRO_GetAvThrottleValue(pHandle->pThrottle);
	
	if ( (hThrottleValue > pHandle->sParameters.hStartingThrottle || (pHandle->pPAS->bPASDetected)) && PWREN_IsPowerEnabled(pHandle->pPWREN) && !BRK_IsPressed(pHandle->pBrake) )
	{
		bCheckStart = true;
	}
	return bCheckStart;
}

/**
	* @brief  Manage motor faults. Check if faults are still present and send motor fault acknowledge when faults are gone.
	* @param  Drivetrain handle
	* @retval Returns true if a motor fault is still active, false if no more fault is present.
	* 			  OV: Works
	*					OT: Works
  *					OC: Works
	*					SF: Works 
	*					UV: Needs to be checked on STM side
	*					SU: No tested yet
	*/
bool DRVT_MotorFaultManagement(DRVT_Handle_t * pHandle)
{
	uint16_t hM1FaultNowCode = MDI_getCurrentFaults(pHandle->pMDI, M1);
	uint16_t hM1FaultOccurredCode = MDI_getOccurredFaults(pHandle->pMDI, M1);
	uint16_t hM2FaultNowCode = MDI_getCurrentFaults(pHandle->pMDI, M2);
	uint16_t hM2FaultOccurredCode = MDI_getOccurredFaults(pHandle->pMDI, M2);
	
	bool bFaultNow = hM1FaultNowCode | hM2FaultNowCode;
	
	//If there's no current motor errors
	if (!bFaultNow)
	{
		if ( DRVT_IsMotor1Used(pHandle) )
		{// If there's an over current (OC) that has occurred but has already been cleared on the STM side
			if ( hM1FaultOccurredCode & MC_BREAK_IN ) 
			{
				if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the OC fault
					hM1FaultOccurredCode &= ~MC_BREAK_IN;
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1]++;
				}
			}
			
			if ( hM1FaultOccurredCode & MC_SPEED_FDBK )
			{// If there's a speed feedback (SF) that has occurred but has already been cleared on the STM side
				if(pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the SF fault
					hM1FaultOccurredCode &= ~MC_SPEED_FDBK;
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1]++;
				}
			}
			
			if ( hM1FaultOccurredCode & MC_START_UP )
			{
				/* In case of motor startup failure... */
				if(pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the SF fault
					hM1FaultOccurredCode &= ~MC_START_UP;
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1]++;
				}
			}
			
			if ( hM1FaultOccurredCode & MC_OVER_TEMP )
			{
				// In case of overtemperature, clear the OT fault
				hM1FaultOccurredCode &= ~MC_OVER_TEMP;
			}
			
			if ( hM1FaultOccurredCode & MC_OVER_VOLT )
			{
				// In case of DCbus overvoltage, clear the OV fault
				hM1FaultOccurredCode &= ~MC_OVER_VOLT;
			}
			
			if ( hM1FaultOccurredCode & MC_UNDER_VOLT )
			{
				// In case of DCbus undervoltage, clear the UV fault
				hM1FaultOccurredCode &= ~MC_UNDER_VOLT;
			}
		}
		
		if ( DRVT_IsMotor2Used(pHandle) )
		{
			if ( hM2FaultOccurredCode & MC_BREAK_IN ) 
			{
				if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the OC fault
					hM2FaultOccurredCode &= ~MC_BREAK_IN;
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2]++;
				}
			}
			
			if ( hM2FaultOccurredCode & MC_SPEED_FDBK )
			{// If there's a speed feedback (SF) that has occurred but has already been cleared on the STM side
				if(pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the SF fault
					hM2FaultOccurredCode &= ~MC_SPEED_FDBK;
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2]++;
				}
			}
			
			if ( hM2FaultOccurredCode & MC_START_UP )
			{// If there's a start-up (SU) that has occurred but has already been cleared on the STM side
				if(pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the SF fault
					hM2FaultOccurredCode &= ~MC_START_UP;
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2]++;
				}
			}
			
			if ( hM2FaultOccurredCode & MC_OVER_TEMP )
			{
				/* In case of overtemperature... */
				hM2FaultOccurredCode &= ~MC_OVER_TEMP;
			}
			
			if ( hM2FaultOccurredCode & MC_OVER_VOLT )
			{
				/* In case of DCbus overvoltage... */
				hM2FaultOccurredCode &= ~MC_OVER_VOLT;
			}
			
			if ( hM2FaultOccurredCode & MC_UNDER_VOLT )
			{
				/* In case of DCbus undervoltage... */
				hM2FaultOccurredCode &= ~MC_UNDER_VOLT;
			}
		}
	} // End of if (!bFaultNow)
	
	// Verify if all fault occured have been cleared
	bool bFaultOccured = (hM1FaultOccurredCode | hM2FaultOccurredCode);

	if (!bFaultOccured)
	{
				//todo: handle result from MDI_FaultAcknowledged below
		MDI_FaultAcknowledged(pHandle->pMDI, M1);
		MDI_FaultAcknowledged(pHandle->pMDI, M2);
	}	
	return bFaultOccured; 
}

/**
	* @brief  Set PAS level
	* @param  Drivetrain handle
	* @param  PAS level
	* @retval None
	*/
void DRVT_SetPASLevel(DRVT_Handle_t * pHandle, PAS_sLevel Level)
{
	 pHandle->pPAS->pLevel = Level;
}

/**
	* @brief  Get PAS level
	* @param  Drivetrain handle
	* @param  PAS level
	* @retval None
	*/
PAS_sLevel DRVT_GetPASLevel (DRVT_Handle_t * pHandle)
{
	return pHandle->pPAS->pLevel;
}
/**
	* @brief  Get main motor reference torque
	* @param  Drivetrain handle
	* @retval Returns main motor reference torque
	*/
int16_t DRVT_GetTorqueRefMainMotor(DRVT_Handle_t * pHandle)
{
	return pHandle->aTorque[pHandle->bMainMotor];
}

/**
	* @brief  Get motor 1 reference torque
	* @param  Drivetrain handle
	* @retval Returns motor 1 reference torque
	*/
int16_t DRVT_GetTorqueRefM1(DRVT_Handle_t * pHandle)
{
	return pHandle->aTorque[M1];
}

/**
	* @brief  Get motor 2 reference torque
	* @param  Drivetrain handle
	* @retval Returns motor 2 reference torque
	*/
int16_t DRVT_GetTorqueRefM2(DRVT_Handle_t * pHandle)
{
	return pHandle->aTorque[M2];
}

/**
	* @brief  Get main motor speed reference
	* @param  Drivetrain handle
	* @retval Returns main motor speed reference
	*/
int32_t DRVT_GetSpeedRefMainMotor(DRVT_Handle_t * pHandle)
{
	return pHandle->aSpeed[pHandle->bMainMotor];
}

/**
	* @brief  Get motor 1 speed reference
	* @param  Drivetrain handle
	* @retval Returns motor 1 speed reference
	*/
int32_t DRVT_GetSpeedRefM1(DRVT_Handle_t * pHandle)
{
	return pHandle->aSpeed[M1];
}

/**
	* @brief  Get motor 2 speed reference
	* @param  Drivetrain handle
	* @retval Returns motor 2 speed reference
	*/
int32_t DRVT_GetSpeedRefM2(DRVT_Handle_t * pHandle)
{
	return pHandle->aSpeed[M2];
}

/**
	* @brief  Check if motor 1 is used
	* @param  Drivetrain handle
	* @retval Returns true if motor 1 is used
	*/
bool DRVT_IsMotor1Used(DRVT_Handle_t * pHandle)
{
	return pHandle->sParameters.bUseMotorM1;
}

/**
	* @brief  Check if motor 2 is used
	* @param  Drivetrain handle
	* @retval Returns true if motor 2 is used
	*/
bool DRVT_IsMotor2Used(DRVT_Handle_t * pHandle)
{
	return pHandle->sParameters.bUseMotorM2;
}

/**
	* @brief  Set Pedal Assist torque based on screen informations
	* @param  Drivetrain handle
	* @retval pRefTorque in int16
	*/
int16_t DRVT_GetPASTorque(DRVT_Handle_t * pHandle)
{
	int16_t hRefTorque;
	PAS_sLevel Got_Level;
	Got_Level = DRVT_GetPASLevel(pHandle);
	
	hRefTorque = (pHandle->sParameters.hPASMaxTorque / pHandle->pPAS->bMaxLevel) * Got_Level;
	
	return hRefTorque;
}

/**
	* @brief  Set Pedal Assist speed based on screen informations
	* @param  Drivetrain handle
	* @retval pRefTorque in int16
	*/
void DRVT_PASSetMaxSpeed(DRVT_Handle_t * pHandle)
{
	PAS_sLevel Got_Level;
	Got_Level = DRVT_GetPASLevel(pHandle);
	
	FLDBK_SetStartValue (&pHandle->sSpeedFoldback[M1], (pHandle->sParameters.hPASMaxSpeed / pHandle->pPAS->bMaxLevel) * Got_Level);
	FLDBK_SetEndValue (&pHandle->sSpeedFoldback[M1], (pHandle->sParameters.hPASMaxSpeed / pHandle->pPAS->bMaxLevel) * Got_Level);

	FLDBK_SetStartValue (&pHandle->sSpeedFoldback[M2], (pHandle->sParameters.hPASMaxSpeed / pHandle->pPAS->bMaxLevel) * Got_Level);
	FLDBK_SetEndValue (&pHandle->sSpeedFoldback[M2], (pHandle->sParameters.hPASMaxSpeed / pHandle->pPAS->bMaxLevel) * Got_Level);
}

/**
	* @brief  Set Pedal Assist torque based on the Torque Sensor
	* @param  Drivetrain handle
	* @retval pRefTorqueS in int16
	*/
int16_t DRVT_GetTorqueFromTS(DRVT_Handle_t * pHandle)
{
	int16_t hRefTorqueS, hReadTS;
	PAS_sLevel Got_Level;
	hReadTS = TS_ToMotorTorque(pHandle->pPAS->pTorque);
	Got_Level = DRVT_GetPASLevel(pHandle);
	
	hRefTorqueS = (hReadTS / pHandle->pPAS->bMaxLevel) * Got_Level;
	
	if (hRefTorqueS < pHandle->pPAS->bMaxTorque)
	{
		hRefTorqueS = pHandle->pPAS->bMaxTorque;
	}
	return hRefTorqueS;
}
/**
	* @brief  Select Control assistance based on Throttle or PAS
	* @param  Drivetrain handle
	* @retval pHandle->pTorqueSelect in int16                                                                                    
	*/
int16_t DRVT_CalcSelectedTorque(DRVT_Handle_t * pHandle)
{	
	/* PAS and Throttle management */
	if ( PAS_IsPASDetected(pHandle->pPAS) && !THRO_IsThrottleDetected(pHandle->pThrottle) )
	{
		/* Torque sensor enabled */
		if (pHandle->sParameters.bTorqueSensorUse)
		{
				pHandle->hTorqueSelect = DRVT_GetTorqueFromTS(pHandle);
				DRVT_PASSetMaxSpeed(pHandle);
		}
		else
		{
				pHandle->hTorqueSelect= DRVT_GetPASTorque(pHandle);
				DRVT_PASSetMaxSpeed(pHandle);
		}
	}
	else
	{		
		/* Throttle value convert to torque */
		pHandle->hTorqueSelect = THRO_ThrottleToTorque(pHandle->pThrottle);
	}
	return pHandle->hTorqueSelect;
}

