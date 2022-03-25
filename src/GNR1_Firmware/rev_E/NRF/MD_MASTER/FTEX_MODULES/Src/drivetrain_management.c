/**
  ******************************************************************************
  * @file    drivetrain_management.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles management of the vehicle drivetrain
  *
	******************************************************************************
*/

#include "drivetrain_management.h"


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
	
	MotorSelection_t bMotorSelection = MS_CheckSelection(pHandle->pMS);
	int16_t hTorqueRef = 0; int32_t hSpeedRef = 0;
	
	switch (bMotorSelection)
	{
		case M1_SELECTED:
			pHandle->bMode = SINGLE_MOTOR;
			pHandle->bMainMotor = M1;
			break;
		case M2_SELECTED:
			pHandle->bMode = SINGLE_MOTOR;
			pHandle->bMainMotor = M2;
			break;
		case ALL_MOTOR_SELECTED:
			pHandle->bMode = DUAL_MOTOR;
			pHandle->bMainMotor = pHandle->bDefaultMainMotor;
			break;
		default:
			break;
	}
	
	THRO_CalcAvThrottleValue(pHandle->pThrottle);
	
	pHandle->aTorque[M1] = 0; pHandle->aTorque[M2] = 0;
	pHandle->aSpeed[M1] = 0; pHandle->aSpeed[M2] = 0;
	
	if (pHandle->bCtrlType == TORQUE_CTRL)
	{
		hTorqueRef = THRO_ThrottleToTorque(pHandle->pThrottle);
		if ( bIsBrakePressed )
		{
			hTorqueRef = 0;
		}
		
		if(pHandle->bMode == SINGLE_MOTOR)
		{
			pHandle->aTorque[pHandle->bMainMotor] = hTorqueRef;
		}
		if(pHandle->bMode == DUAL_MOTOR)
		{
			pHandle->aTorque[M1] = hTorqueRef;
			pHandle->aTorque[M2] = hTorqueRef;
		}
	}
	else if (pHandle->bCtrlType == SPEED_CTRL)
	{
		hSpeedRef = THRO_ThrottleToSpeed(pHandle->pThrottle);
		if ( bIsBrakePressed )
		{
			hSpeedRef = 0;
		}
		
		if(pHandle->bMode == SINGLE_MOTOR)
		{
			pHandle->aSpeed[pHandle->bMainMotor] = hSpeedRef;
		}
		if(pHandle->bMode == DUAL_MOTOR)
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
		if (pHandle->bCtrlType == TORQUE_CTRL)
		{
			if ( abs(pHandle->aTorque[M1]) > abs(MDI_getIq(pHandle->pMDI, M1)) )
			{
				MDI_SetTorqueRamp(pHandle->pMDI, M1, pHandle->aTorque[M1], pHandle->hTorqueRampTimeU);
			}
			else
			{
				MDI_SetTorqueRamp(pHandle->pMDI, M1, pHandle->aTorque[M1], pHandle->hTorqueRampTimeD);
			}
		}
		else if (pHandle->bCtrlType == SPEED_CTRL)
		{
			MDI_SetSpeedRamp(pHandle->pMDI, M1, pHandle->aSpeed[M1], pHandle->hSpeedRampTimeU);
		}
		else {}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		if (pHandle->bCtrlType == TORQUE_CTRL)
		{
			if ( abs(pHandle->aTorque[M2]) > abs(MDI_getIq(pHandle->pMDI, M2)) )
			{
				MDI_SetTorqueRamp(pHandle->pMDI, M2, pHandle->aTorque[M2], pHandle->hTorqueRampTimeU);
			}
			else
			{
				MDI_SetTorqueRamp(pHandle->pMDI, M2, pHandle->aTorque[M2], pHandle->hTorqueRampTimeD);
			}
		}
		else if (pHandle->bCtrlType == SPEED_CTRL)
		{
			MDI_SetSpeedRamp(pHandle->pMDI, M2, pHandle->aSpeed[M2], pHandle->hSpeedRampTimeU);
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
	int32_t wSpeedM1 = MDI_getSpeed(pHandle->pMDI, M1);
	int32_t wSpeedM2 = MDI_getSpeed(pHandle->pMDI, M2);
	uint16_t hThrottleValue = THRO_GetAvThrottleValue(pHandle->pThrottle);
	
	if ( DRVT_IsMotor1Used(pHandle) )
	{
		if ( hThrottleValue < pHandle->hStoppingThrottle && abs(wSpeedM1) <= pHandle->hStoppingSpeed )
		{
			bCheckStop1 = true;
		}
	}
	if ( DRVT_IsMotor2Used(pHandle) )
	{
		if ( hThrottleValue < pHandle->hStoppingThrottle && abs(wSpeedM2) <= pHandle->hStoppingSpeed )
		{
			bCheckStop2 = true;
		}
	}
	if ( !PWREN_IsPowerEnabled(pHandle->pPWREN) )
	{
		bCheckStop3 = true;
	}	
	if ( BRK_IsPressed(pHandle->pBrake) )
	{
		bCheckStop4 = true;
	}
	
	return (bCheckStop1 & bCheckStop2) | bCheckStop3 | bCheckStop4;
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
	
	if ( hThrottleValue > pHandle->hStartingThrottle && PWREN_IsPowerEnabled(pHandle->pPWREN) && !BRK_IsPressed(pHandle->pBrake) )
	{
		bCheckStart = true;
	}
	return bCheckStart;
}

/**
	* @brief  Manage motor faults. Check if faults are still present and send motor fault acknowledge when faults are gone.
	* @param  Drivetrain handle
	* @retval Returns true if a motor fault is still active, false if no more fault is present.
	*/
bool DRVT_MotorFaultManagement(DRVT_Handle_t * pHandle)
{
	uint16_t hM1FaultNowCode = MDI_getCurrentFaults(pHandle->pMDI, M1);
	uint16_t hM1FaultOccurredCode = MDI_getOccurredFaults(pHandle->pMDI, M1);
	uint16_t hM2FaultNowCode = MDI_getCurrentFaults(pHandle->pMDI, M2);
	uint16_t hM2FaultOccurredCode = MDI_getOccurredFaults(pHandle->pMDI, M2);
	
	bool bFaultNow = hM1FaultNowCode & hM2FaultNowCode;
			
	if (!bFaultNow)
	{
		if ( DRVT_IsMotor1Used(pHandle) )
		{
			if ( hM1FaultOccurredCode & MC_BREAK_IN )
			{
				/* In case of motor overcurrent... */
			}
			if ( hM1FaultOccurredCode & MC_SPEED_FDBK )
			{
				/* In case of speed sensor failure... */
			}
			if ( hM1FaultOccurredCode & MC_START_UP )
			{
				/* In case of motor startup failure... */
			}
			if ( hM1FaultOccurredCode & MC_OVER_TEMP )
			{
				/* In case of overtemperature... */
				hM1FaultOccurredCode &= ~MC_OVER_TEMP;
			}
			if ( hM1FaultOccurredCode & MC_OVER_VOLT )
			{
				/* In case of DCbus overvoltage... */
				hM1FaultOccurredCode &= ~MC_OVER_VOLT;
			}
			if ( hM1FaultOccurredCode & MC_UNDER_VOLT )
			{
				/* In case of DCbus undervoltage... */
				hM1FaultOccurredCode &= ~MC_UNDER_VOLT;
			}
		}
		
		if ( DRVT_IsMotor2Used(pHandle) )
		{
			if ( hM2FaultOccurredCode & MC_BREAK_IN )
			{
				/* In case of motor overcurrent... */
			}
			if ( hM2FaultOccurredCode & MC_SPEED_FDBK )
			{
				/* In case of speed sensor failure... */
			}
			if ( hM2FaultOccurredCode & MC_START_UP )
			{
				/* In case of motor startup failure... */
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
	}
	
	bool bFaultOccured = (hM1FaultOccurredCode & hM2FaultOccurredCode);
	
	if (!bFaultOccured)
	{
        //todo: handle result from MDI_FaultAcknowledged below
		MDI_FaultAcknowledged(pHandle->pMDI, M1);
		MDI_FaultAcknowledged(pHandle->pMDI, M2);
	}
	
	//return bFaultOccured;
	return true; //debug
}

/**
	* @brief  Set PAS level
	* @param  Drivetrain handle
	* @param  PAS level
	* @retval None
	*/
void DRVT_SetPASLevel(DRVT_Handle_t * pHandle, uint8_t level)
{
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
	return pHandle->bUseMotorM1;
}

/**
	* @brief  Check if motor 2 is used
	* @param  Drivetrain handle
	* @retval Returns true if motor 2 is used
	*/
bool DRVT_IsMotor2Used(DRVT_Handle_t * pHandle)
{
	return pHandle->bUseMotorM2;
}


