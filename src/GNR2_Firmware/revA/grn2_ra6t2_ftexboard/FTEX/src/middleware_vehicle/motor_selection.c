/**
  * @file    motor_selection.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles motor selection inputs
  *
*/

#include "motor_selection.h"
#include "ASSERT_FTEX.h"

/**
 * @brief Initializes torque distribution module
 *
 *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
 */
void MS_Init(MS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    struct GPIOConfig PinConfig;
   
    PinConfig.PinDirection = INPUT;
    PinConfig.PinPull      = UP; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(pHandle->wM1SelectPinNumber, PinConfig);
    uCAL_GPIO_ReInit(pHandle->wM2SelectPinNumber, PinConfig);
}

/**
 * @brief Function used to read motor selection signals, using
 *        pins 
 *
 *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
 */
MotorSelection_t MS_CheckSelection(MS_Handle_t* pHandle)
{
	ASSERT(pHandle != NULL);
	if (pHandle->bMSEnable)
	{
		bool bM1Select = uCAL_GPIO_Read(pHandle->wM1SelectPinNumber);
		bool bM2Select = uCAL_GPIO_Read(pHandle->wM2SelectPinNumber);
		bool bBothSelect = bM1Select || bM2Select;
		
        //false(signal 0 on both pins) means switch is at middle
        //position and all motors need to be used.
		if (bBothSelect == false)
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

