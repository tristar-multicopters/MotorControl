/**
  * @file    mc_tasks.h
  * @brief   This file implementes tasks definition.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCTASKS_H
#define __MCTASKS_H

/* Includes ------------------------------------------------------------------*/
#include "mc_tuning.h"
#include "mc_interface.h"
#include "mc_config.h"


extern MotorControlInterfaceHandle_t MCInterface[NBR_OF_MOTORS];


/**
  * @brief  Initializes the Motor subsystem core according to user defined parameters.
	* @param	None
	* @retval None
  */
void MC_Bootup(void);


/**
  * @brief  Runs motor control tasks depending on motor state, such as speed control, current reference calculation, etc.
	* @param	None
	* @retval None
  */
void MC_RunMotorControlTasks(void);


/**
  * @brief  Executes the Medium Frequency Task functions for each drive instance.
	* @param	None
	* @retval None
  */
void MC_Scheduler(void);


/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances
	* @param	None
	* @retval None
  */
void MC_SafetyTask(void);


/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
	* @param	None
	* @retval None
  */
uint8_t MC_HighFrequencyTask(void);


/**
  * @brief  Reserves FOC execution on ADC ISR half a PWM period in advance
	* @param	None
	* @retval None
  */
void MC_DualDriveFIFOUpdate(uint8_t Motor);


/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault.
	* @param	None
	* @retval None
  */
void MC_HardwareFaultTask(void);



#endif /* __MCTASKS_H */

