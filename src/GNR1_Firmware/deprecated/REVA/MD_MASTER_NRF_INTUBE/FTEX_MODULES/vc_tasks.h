/**
  ******************************************************************************
  * @file    vc_tasks.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module gathers tasks of vehicule controller
  *
	******************************************************************************
	*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VC_TASKS_H
#define __VC_TASKS_H

#include "vc_config.h"


extern TaskHandle_t TSK_commManager_handle;
extern TaskHandle_t TSK_Throttle_handle;

	
	/**@brief Task function to read throttle value and send torque reference to motor drive*/
void TSK_Throttle (void * pvParameter);

	/**@brief Task function to manage communications between systems*/
void TSK_commManager (void * pvParameter);

#endif /* __MD_TASKS_H */

