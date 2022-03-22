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
void VCI_SetRegister(VCI_Handle_t* pHandle, uint16_t RegID, uint8_t value)
{
	switch (RegID)
	{
		case REG_DRVTRAIN_TYPE:
		{
			// cast to enum otherwise the compiler complains
			DRVT_Type_h castedValue = (DRVT_Type_h) value; 
			
			pHandle->pDrivetrain->bDrivetrainType = castedValue;
			break;
		}
		case REG_DUALMOTOR:
		{
			// cast to enum otherwise the compiler complains
			Motor_Mode_t castedValue = (Motor_Mode_t) value; 
			
			pHandle->pDrivetrain->bMode = castedValue;
			break;
		}
		case REG_MAINMOTOR:
			pHandle->pDrivetrain->bMainMotor = value;
			break;
		default:
			break;
	}
}

