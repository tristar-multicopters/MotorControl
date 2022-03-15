/**
  ******************************************************************************
  * @file    display_management.h
	* @author  Sami Bouzid, FTEX
  * @brief   This module manages communication between vehicle and external screen.
  *
	******************************************************************************
	*/
	
#ifndef __DISPLAY_MANAGEMENT_H
#define __DISPLAY_MANAGEMENT_H

#include "mc_defines.h"
#include "vc_interface.h"


/************************************ FUNCTIONS *************************************/

/* Task function for managing display */
void TSK_DisplayManagement(void * pvParameter);

/* Send all relevent vehicle information to screen */
void DISP_SendInfoToScreen(VC_Handle_t * pVCHandle);



#endif
