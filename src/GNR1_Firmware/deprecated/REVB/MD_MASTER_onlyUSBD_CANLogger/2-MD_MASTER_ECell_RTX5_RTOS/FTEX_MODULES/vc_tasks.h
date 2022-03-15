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
#include "stdlib.h"
#include "nrf_drv_clock.h"

#define TASK_VCFASTLOOP_SAMPLE_TIME_TICK 20
#define TASK_VCSLOWLOOP_SAMPLE_TIME_TICK 250

void VC_BootUp(void);

	/**@brief Task function to execute motor drive management at higher rate*/
__NO_RETURN void TSK_FastLoopMD (void * pvParameter);

	/**@brief Task function to execute motor drive management at slow rate*/
__NO_RETURN void TSK_SlowLoopMD (void * pvParameter);

#endif /* __MD_TASKS_H */

