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

#ifdef ENABLE_CAN_LOGGER
#include "can_logger.h"
#endif

#define TASK_CAN_SAMPLE_TIME_TICK  50     /*  Loop for executing the task every 375ms */

/**
  * @brief  Task to managing the CAN interface
  *         This task updates periodically the GNR2 Object Dictionary and executes when the
  *         the hardware timer 1 is timeout.
  * @retval None
  */
void CANManagerTask (void * pvParameter);

/**
  * @brief  Task to handle the received messages (SDO requests)
  *         through the CAN bus
  * @retval None
  */
void CANProcessMsgs (void * pvParameter);
    

void Comm_BootUp(void);


#endif /* __COMM_TASKS_H */

