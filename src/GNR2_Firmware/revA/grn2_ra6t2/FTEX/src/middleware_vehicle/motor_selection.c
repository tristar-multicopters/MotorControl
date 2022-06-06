/**
  * @file    motor_selection.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles motor selection inputs
  *
*/

#include "motor_selection.h"


/**
 * @brief Initializes torque distribution module
 *
 *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
 */
void MS_Init( MS_Handle_t * pHandle )
{

}

MotorSelection_t MS_CheckSelection(MS_Handle_t* pHandle)
{
	
	if (pHandle->bMSEnable)
	{
		bool bM1Select = true; //TODO: check gpio input, for now just mock
		bool bM2Select = true; //TODO: check gpio input, for now just mock
		bool bBothSelect = bM1Select && bM2Select;
		
		if (bBothSelect)
		{
			pHandle->bMotorSelection = ALL_MOTOR_SELECTED;
		}
		else if (bM2Select)
		{
			pHandle->bMotorSelection = M2_SELECTED;
		}
		else
		{
			pHandle->bMotorSelection = M1_SELECTED;
		}
		
	}
	else
	{
		pHandle->bMotorSelection = M1_SELECTED;
	}
	return pHandle->bMotorSelection;
}

