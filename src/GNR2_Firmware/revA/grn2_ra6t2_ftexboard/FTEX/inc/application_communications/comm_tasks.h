/**
  * @file    comm_tasks.h
  * @brief   This module gathers tasks of vehicule controller
  *
  */

#ifndef __COMM_TASKS_H
#define __COMM_TASKS_H


#include "stdbool.h"
#include "gnr_parameters.h"
#include "comm_defines.h"
#include "firmware_update.h"
#include "vc_fault_management.h"
#include "gnr_information.h"

/**************************************/

extern bool bCANOpenTaskBootUpCompleted;


/**
  * @brief  It initializes the vehicle control application. Needs to be called before using
  *	        vehicle control related modules.
  * @retval None
  */
void Comm_BootUp(void);

/**
  * @brief  Function to configure and initialize the CANOPEN node.
  * @retval None
  */
void CANOpenTask (void);

/**
  * @brief Function to decide if some IOT functions must be used or
  * not to some bike models.
  * @retval none
*/
bool Comm_CheckIotUsage(void);

/**
  * @brief Initialises the object dictionairy with the user config values
  * @retval none
  */
void Comm_InitODWithUserConfig(CO_NODE *pNode);

/**
  * @brief  Task to send vehicle information on CANbus for data logging.
  * @retval None
  */
void CANLoggerTask (void * pvParameter);

#endif /* __COMM_TASKS_H */
