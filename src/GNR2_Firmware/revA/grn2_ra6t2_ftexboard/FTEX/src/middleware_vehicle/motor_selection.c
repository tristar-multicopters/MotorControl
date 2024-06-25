/**
  * @file    motor_selection.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles motor selection inputs
  *
*/

#include "motor_selection.h"
#include "ASSERT_FTEX.h"
#include "uCAL_GPIO.h"

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
    PinConfig.PinPull      = NONE; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(pHandle->wM1SelectPinNumber, PinConfig);
    uCAL_GPIO_ReInit(pHandle->wM2SelectPinNumber, PinConfig);
}

MotorSelection_t MS_CheckSelection(MS_Handle_t* pHandle)
{
    ASSERT(pHandle != NULL);
    if (pHandle->bMSEnable)
    {
        bool bM1Select = uCAL_GPIO_Read(pHandle->wM1SelectPinNumber);
        bool bM2Select = uCAL_GPIO_Read(pHandle->wM2SelectPinNumber);
        bool bBothSelect = false;
        
        //Pins for the motor selector of M1 and M2 read 0 when both are selected
        if (bM1Select == bM2Select)
        {
            bBothSelect = true;
        }
        
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

