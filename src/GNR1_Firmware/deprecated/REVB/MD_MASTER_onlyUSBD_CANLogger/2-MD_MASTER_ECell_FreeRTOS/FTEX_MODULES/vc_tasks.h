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

#define TASK_VCFASTLOOP_SAMPLE_TIME_TICK 50
#define TASK_VCSLOWLOOP_SAMPLE_TIME_TICK 125


void VC_BootUp(void);

	/**@brief Task function to execute motor drive management at higher rate*/
void TSK_FastLoopMD (void * pvParameter);

	/**@brief Task function to execute motor drive management at slow rate*/
void TSK_SlowLoopMD (void * pvParameter);

#endif /* __MD_TASKS_H */

