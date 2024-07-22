/**
  ******************************************************************************
  * @file    stuck_protection.h
  * @author  Behnam Shakibafar, FTEX inc
  * @brief   This file contains all definitions and functions prototypes for the
  *          stuck protection component of the Motor Control application.
  *
  ******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STUCKPROTECTION_H
#define __STUCKPROTECTION_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "ASSERT_FTEX.h"
#include "stdlib.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint16_t  timeout_general;          /* new maximum power based on foldback function */
    uint16_t  min_torque;               /* timer to count time elapsed with power more that MAX */
    uint16_t  low_battery_voltage;      /* timer to count time elapsed with power less that MAX */
    uint16_t  timeout_low_battery;      /* timeout for hOverMaxPowerTimeout */
    uint16_t  counter;                  /* counter variable */
} StuckProtection_t;


/* Exported functions ------------------------------------------------------- */

/**
    * @brief  It reset the state variable to zero.
    * @param  pHandle related Handle of struct RampMngr_Handle_t
    * @retval none.
  */
void StuckProtection_Init(StuckProtection_t * pHandle);

/**
    * @brief  Check if the motor stcuk or not, return the result
    * @param  pHandle: handler of the stcuk protection parameters
    *         hFinalTorqueRef: final t
    *         hBusVoltage: the DC Bus voltage = Battery Voltage
    *         AvrgMecSpeed: average speed of the motor
    * @retval uint16_t the fault status
  */
uint32_t Check_MotorStuckReverse(StuckProtection_t * pHandle, int16_t hFinalTorqueRef, uint16_t hBusVoltage, int16_t AvrgMecSpeed);
/**
    * @brief  Clear  the motor stcuk timer
    * @param  
    * @retval 
*/
void Clear_MotorStuckReverse(StuckProtection_t * pHandle);

#endif /* __STUCKPROTECTION_H */



