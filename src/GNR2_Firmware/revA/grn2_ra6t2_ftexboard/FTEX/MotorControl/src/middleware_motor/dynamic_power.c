/**
    ******************************************************************************
    * @file     dynamic_power.c
    * @author   Behnam Shakibafar, FTEX inc
    * @brief    This file provides firmware functions that implement the features
    *           of dynamic power during time    *
    ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "dynamic_power.h"

/* 
    It reset the state variable to zero.
*/

void DynamicPower_Init(DynamicPowerHandle_t * pHandle, uint16_t hDynamicMaxPower, uint16_t hEstimatedEfficiency)
{
    ASSERT(pHandle != NULL);
    pHandle->hDynamicMaxPower = hDynamicMaxPower * hEstimatedEfficiency / 100;
    pHandle->hOverMaxPowerTimer = 0;
    pHandle->hBelowMaxPowerTimer = 0;
}

