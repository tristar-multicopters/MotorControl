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

#include "stdlib.h"
#include "nrf_drv_clock.h"

#include "vc_config.h"
#include "canbus_management.h"
#include "storage_management.h"

#define TASK_VCFASTLOOP_SAMPLE_TIME_TICK 	20
#define TASK_VCSLOWLOOP_SAMPLE_TIME_TICK 	200
#define TASK_VCSTM_SAMPLE_TIME_TICK				20


void VC_BootUp(void);

	/**@brief Task function to execute motor drive management at higher rate*/
__NO_RETURN void TSK_FastLoopMD (void * pvParameter);

	/**@brief Task function to execute motor drive management at slow rate*/
__NO_RETURN void TSK_SlowLoopMD (void * pvParameter);

	/**@brief Task function to manage fault conditions*/
__NO_RETURN void TSK_VehicleStateMachine (void * pvParameter);

#endif /* __MD_TASKS_H */

