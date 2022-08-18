/**
* @file   CAN_task.h
* @author Jorge Polo
* @brief  Module for defining and managing the CAN messages task
*
* This module will define the task that will be used for managing 
* the messages CAN messages sent by Evionics power and received from
* an external device (Evionics Comms or LCD screen for exemple)
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_TASK_H
#define __CAN_TASK_H

#include "vc_config.h"
#include "gnr_main.h"
#include "gnr_parameters.h"

#ifdef ENABLE_CAN_LOGGER
#include "can_logger.h"
#endif

#define TASK_CAN_SAMPLE_TIME_TICK  50     /*  Loop for executing the task every 375ms */

/**
  * @brief  Task to handle the received messages and to send messages
  *         through the CAN bus
  * @retval None
  */
void processCANmsgTask (void * pvParameter);

#endif