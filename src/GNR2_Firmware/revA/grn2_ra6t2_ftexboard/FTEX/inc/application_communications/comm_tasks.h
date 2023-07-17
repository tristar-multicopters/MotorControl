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

/************* DEFINES ****************/


/**************************************/


extern bool bCANOpenTaskBootUpCompleted;

/**
  * @brief  Function to configure and initialize the CANOPEN node.
  * @retval None
  */
void CANOpenTask (void);

/**
  * @brief  Task to send vehicle information on CANbus for data logging.
  * @retval None
  */
void CANLoggerTask (void * pvParameter);


void Comm_BootUp(void);

/**
  * @briefFunction to decide if some IOT functions must be used or
  * not to some bike models.
  * @retval none
*/
bool Comm_CheckIotUsage(void);

#endif /* __COMM_TASKS_H */
