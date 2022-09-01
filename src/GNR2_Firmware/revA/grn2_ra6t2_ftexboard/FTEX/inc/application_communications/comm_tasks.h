/**
  * @file    comm_tasks.h
  * @brief   This module gathers tasks of vehicule controller
  *
  */

#ifndef __COMM_TASKS_H
#define __COMM_TASKS_H

#include "lcd_apt_comm.h"

#include "vc_config.h"
#include "gnr_parameters.h"


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
