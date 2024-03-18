/**
  * @file    motor_selection.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles motor selection inputs
  *
    */
    
    /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_SELECTION_H
#define __MOTOR_SELECTION_H

#include "stdbool.h"
#include "stdint.h"

typedef enum
{
    M1_SELECTED,
    M2_SELECTED,
    ALL_MOTOR_SELECTED,
    
} MotorSelection_t;

typedef struct
{          
    uint32_t wM1SelectPinNumber;
    uint32_t wM2SelectPinNumber;
    bool bIsInvertedLogic;
    bool bMSEnable;
    
    MotorSelection_t bMotorSelection;
    
} MS_Handle_t;


void MS_Init(MS_Handle_t * pHandle);

MotorSelection_t MS_CheckSelection(MS_Handle_t * pHandle);


#endif /*__MOTOR_SELECTION_H*/

