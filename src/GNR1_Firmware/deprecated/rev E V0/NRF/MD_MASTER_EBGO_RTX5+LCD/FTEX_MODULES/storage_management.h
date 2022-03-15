/**
  ******************************************************************************
  * @file    storage_management.h
	* @author  Sami Bouzid, FTEX
  * @brief   This module manages data storage in flash or external EEPROM, and software updates.
  *
	******************************************************************************
	*/
	
#ifndef __STORAGE_MANAGEMENT_H
#define __STORAGE_MANAGEMENT_H

#include "mc_defines.h"
#include "vc_interface.h"

typedef enum
{
  VC_PARAM_ID_WHEELDIA,
  VC_PARAM_ID_MAXSPEED,
  VC_PARAM_ID_GEARRATIO,
	VC_PARAM_ID_THROTTLE_TYPE,
	VC_PARAM_ID_THROTTLE_M,
	VC_PARAM_ID_THROTTLE_F,
	VC_PARAM_ID_THROTTLE_OFFSET,
	VC_PARAM_ID_DUALMOTOR,
	VC_PARAM_ID_MAINMOTOR,
} VC_Params_ID_t;

typedef enum
{
  MD_PARAM_ID_TORQUE_KP,
  MD_PARAM_ID_TORQUE_KI,
  MD_PARAM_ID_FLUX_KP,
	MD_PARAM_ID_FLUX_KI,
	MD_PARAM_ID_MAXTEMP,
	MD_PARAM_ID_MAXPHASECURRENT,
	MD_PARAM_ID_HALL_PHASESHIFT,
	MD_PARAM_ID_POLEPAIRS,
	MD_PARAM_ID_FF_C1,
	MD_PARAM_ID_FW_KP,
	MD_PARAM_ID_BATT_MAXVOLT,
	MD_PARAM_ID_BATT_MINVOLT,
} MD_Params_ID_t;

/************************************ FUNCTIONS *************************************/

/* Task function for managing storage */
void TSK_StorageManagement(void * pvParameter);

/* Function to initialize the storage module*/
void STRG_Init(void);

/* Function to save a vehicle parameter to static memory */
void STRG_SaveVehicleParam(VC_Params_ID_t param, int32_t value);

/* Function to read a vehicle parameter from static memory */
int32_t STRG_ReadVehicleParam(VC_Params_ID_t param);

/* Function to save a motor parameter to static memory */
void STRG_SaveMotorParam(uint8_t motorSelection, MD_Params_ID_t param, int32_t value);

/* Function to read a motor parameter from static memory */
int32_t STRG_ReadMotorParam(uint8_t motorSelection, MD_Params_ID_t param);

void STRG_UpdateVehicleFromMemory(VC_Handle_t * pHandle);

void STRG_UpdateMemoryFromVehicle(VC_Handle_t * pHandle);

void STRG_ResetAllVehicleParams(void);




#endif
