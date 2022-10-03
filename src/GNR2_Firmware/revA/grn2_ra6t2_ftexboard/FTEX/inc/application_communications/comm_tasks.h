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


extern bool bCANOpenTaskBootUpCompleted;

/**
  * @brief  Task to managing the CANopen interface
  *         This task updates periodically the GNR2 object dictionary and executes when the
  *         the hardware timer is timeout.
  * @retval None
  */
void CANOpenTask (void * pvParameter);

/**
  * @brief  Task to send vehicle information on CANbus for data logging.
  * @retval None
  */
void CANLoggerTask (void * pvParameter);


void Comm_BootUp(void);


#endif /* __COMM_TASKS_H */
