/**
  ******************************************************************************
  * @file    vc_interface.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module offers an interface to interact with vehicle properties
	******************************************************************************
	*/
	
#include "vc_interface.h"

//TODO: Complete register table
int32_t VCI_ReadRegister(VCI_Handle_t* pHandle, uint16_t RegID)
{
	int32_t value = 0;
	
	switch (RegID)
	{
		case REG_DRVTRAIN_TYPE:
			value = pHandle->pDrivetrain->bDrivetrainType;
			break;
		case REG_DUALMOTOR:
			value = pHandle->pDrivetrain->bMode;
			break;
		case REG_MAINMOTOR:
			value = pHandle->pDrivetrain->bMainMotor;
			break;
		default:
			break;
	}
	return value;
}

//TODO: Complete register table
void VCI_SetRegister(VCI_Handle_t* pHandle, uint16_t RegID, int32_t value)
{
	switch (RegID)
	{
		case REG_DRVTRAIN_TYPE:
			pHandle->pDrivetrain->bDrivetrainType = value;
			break;
		case REG_DUALMOTOR:
			pHandle->pDrivetrain->bMode = value;
			break;
		case REG_MAINMOTOR:
			pHandle->pDrivetrain->bMainMotor = value;
			break;
		default:
			break;
	}
}

