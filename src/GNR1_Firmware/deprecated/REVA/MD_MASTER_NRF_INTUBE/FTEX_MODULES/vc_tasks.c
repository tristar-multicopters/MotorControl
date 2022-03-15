/**
  ******************************************************************************
  * @file    vc_tasks.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module gathers tasks of vehicule controller
  *
	******************************************************************************
	*/

#include "vc_tasks.h"


void TSK_Throttle (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	int16_t torque_ref = 0;
	
	Throttle_Handle_t * pThrottleSensor = &ThrottleSensor;
	
	Throttle_Init(pThrottleSensor);

	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	while (true)
	{	
		Throttle_CalcAvValue(pThrottleSensor);
		torque_ref = Throttle_CalcIqref(pThrottleSensor);
		
		vTaskDelayUntil( &xLastWakeTime, TASK_THROTTLE_SAMPLE_TIME_TICK );
	}
}

void TSK_commManager (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	vTaskDelay(500);
	
//	md_setMDReg(MC_PROTOCOL_REG_CONTROL_MODE, STC_TORQUE_MODE);		
//	md_setMDReg(MC_PROTOCOL_REG_FLUX_REF, 0);
	
	vTaskDelay(100);

	//md_sendCmdToMD(MC_PROTOCOL_CMD_START_MOTOR);
																				
	while (true)
	{
		vTaskDelay(100);
	}
}



