/**
    ******************************************************************************
    * @file     dynamic_power.c
    * @author   Behnam Shakibafar, FTEX inc
    * @brief    This file provides firmware functions that implement the features
    *           of dynamic power during time    *
    ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stuck_protection.h"

/* 
    It reset the state variable to zero.
*/
void StuckProtection_Init(StuckProtection_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->counter = 0;
}

/*
    Check if motor is stuck or not
*/
uint32_t Check_MotorStuckReverse(StuckProtection_t * pHandle, int16_t hFinalTorqueRef, uint16_t hBusVoltage, int16_t AvrgMecSpeed)
{    
    ASSERT(pHandle != NULL); 
    
    uint32_t wRetval = MC_NO_FAULT;

    if ((AvrgMecSpeed == 0) && (hFinalTorqueRef > pHandle->min_torque) && hBusVoltage > 0)
    {
        // strt a timer to count time motor got stuck
        if (pHandle->counter < pHandle->timeout_general)
        {
            pHandle->counter++;
            wRetval = MC_NO_FAULT;
        }
        //if stuck time is more that threshold, rasie error and cut the power
        else 
        {
            pHandle->counter = 0;
            wRetval = MC_MSRP;
        }
        #if MOTOR_SELECTION == MOTOR_BAFANG_G062_750W
        // this part checks if the battery SoC is low, rasies sooner to prevent unknown motor issue that causes controller burn
        // for now, only spotted on QiuetKat, more tests are needed for other bikes
        if ((hBusVoltage < pHandle->low_battery_voltage) && (pHandle->counter > pHandle->timeout_low_battery))
        {
            pHandle->counter = 0;
            wRetval = MC_MSRP;
        }
        #endif
    }
    else
    {
        pHandle->counter = 0;
        wRetval = MC_NO_FAULT;
    }
    return wRetval;
}

/*
  * Clear  the motor stcuk reverse timer
*/
void Clear_MotorStuckReverse(StuckProtection_t * pHandle)
{
    pHandle->counter = 0;
}