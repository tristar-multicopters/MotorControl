/**
  ******************************************************************************
  * @file    md_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control multiple motor drives.
	*					 M1 is the local drive, whereas M2, M3, M4, ... can be controlled externally using this interface.
	******************************************************************************
*/

#include "md_interface.h"

void MDI_Init(MDI_Handle_t * pHandle)
{
	/* MDI assumes that MCI is already initialized */
}

void MDI_ExecSpeedRamp( MDI_Handle_t * pHandle, uint8_t bMotor, int16_t hFinalSpeed, uint16_t hDurationms )
{
	switch (bMotor)
	{
		case M1:
			MCI_ExecSpeedRamp(pHandle->pMCI, hFinalSpeed, hDurationms);
			break;
		case M2:
			break;
		default:
			break;
	}
}

void MDI_ExecTorqueRamp( MDI_Handle_t * pHandle, uint8_t bMotor, int16_t hFinalTorque, uint16_t hDurationms )
{
	switch (bMotor)
	{
		case M1:
			MCI_ExecTorqueRamp(pHandle->pMCI, hFinalTorque, hDurationms);
			break;
		case M2:
			break;
		default:
			break;
	}
}

void MDI_SetCurrentReferences( MDI_Handle_t * pHandle, uint8_t bMotor, qd_t Iqdref )
{
	switch (bMotor)
	{
		case M1:
			MCI_SetCurrentReferences(pHandle->pMCI, Iqdref);
			break;
		case M2:
			break;
		default:
			break;
	}
}

bool MDI_StartMotor( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	bool bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_StartMotor(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

bool MDI_StopMotor( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	bool bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_StopMotor(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

bool MDI_FaultAcknowledged( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	bool bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_FaultAcknowledged(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

State_t  MDI_GetSTMState( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	State_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetSTMState(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

uint16_t MDI_GetOccurredFaults( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	uint16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetOccurredFaults(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

uint16_t MDI_GetCurrentFaults( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	uint16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetCurrentFaults(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

STC_Modality_t MDI_GetControlMode( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	STC_Modality_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetControlMode(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

int16_t MDI_GetImposedMotorDirection( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	int16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetImposedMotorDirection(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

int16_t MDI_GetLastRampFinalSpeed( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	int16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetLastRampFinalSpeed(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

bool MDI_RampCompleted( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	bool bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_RampCompleted(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

void MDI_StopRamp( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	switch (bMotor)
	{
		case M1:
			MCI_StopRamp(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
}

bool MDI_GetSpdSensorReliability( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	bool bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetSpdSensorReliability(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

int16_t MDI_GetAvrgMecSpeedUnit( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	int16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetAvrgMecSpeedUnit(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

int16_t MDI_GetMecSpeedRefUnit( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	int16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetMecSpeedRefUnit(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

ab_t MDI_GetIab( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	ab_t bReturnValue = {0};
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetIab(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

alphabeta_t MDI_GetIalphabeta( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	alphabeta_t bReturnValue = {0};
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetIalphabeta(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

qd_t MDI_GetIqd( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	qd_t bReturnValue = {0};
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetIqd(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

qd_t MDI_GetIqdHF( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	qd_t bReturnValue = {0};
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetIqdHF(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

qd_t MDI_GetIqdref( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	qd_t bReturnValue = {0};
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetIqdref(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

qd_t MDI_GetVqd( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	qd_t bReturnValue = {0};
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetVqd(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

alphabeta_t MDI_GetValphabeta( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	alphabeta_t bReturnValue = {0};
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetValphabeta(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

int16_t MDI_GetElAngledpp( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	int16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetElAngledpp(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

int16_t MDI_GetTeref( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	int16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetTeref(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

int16_t MDI_GetPhaseCurrentAmplitude( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	int16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetPhaseCurrentAmplitude(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

int16_t MDI_GetPhaseVoltageAmplitude( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	int16_t bReturnValue = 0;
	
	switch (bMotor)
	{
		case M1:
			bReturnValue = MCI_GetPhaseVoltageAmplitude(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
	
	return bReturnValue;
}

void MDI_Clear_Iqdref( MDI_Handle_t * pHandle, uint8_t bMotor )
{
	switch (bMotor)
	{
		case M1:
			MCI_GetCurrentFaults(pHandle->pMCI);
			break;
		case M2:
			break;
		default:
			break;
	}
}

