/**
  ******************************************************************************
  * @file    storage_management.h
	* @author  Sami Bouzid, FTEX
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module manages data storage in flash or external EEPROM, and software updates.
  *
	******************************************************************************
	*/
	
#ifndef __STORAGE_MANAGEMENT_H
#define __STORAGE_MANAGEMENT_H

#include <string.h>
#include "mc_defines.h"
#include "vc_interface.h"

#include "nrf_fstorage.h"
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"

#define STRG_FLAG      0x20
#define START_ADDR  	 0x00070000UL
#define MOTOR_PAR_ADDR 0x00070100UL
#define LAST_ADDR   	 0x0007FFFFUL
#define ADDR_ESPACE 	 0x40

/*
typedef enum
{
  MD_PARAM_ID_TORQUE_KP  			= (START_ADDR + ADDR_ESPACE*10),
  MD_PARAM_ID_TORQUE_KI 			= (START_ADDR + ADDR_ESPACE*12),
  MD_PARAM_ID_FLUX_KP   			= (START_ADDR + ADDR_ESPACE*14),
	MD_PARAM_ID_FLUX_KI   			= (START_ADDR + ADDR_ESPACE*16),
	MD_PARAM_ID_MAXTEMP   			= (START_ADDR + ADDR_ESPACE*18),
	MD_PARAM_ID_MAXPHASECURRENT = (START_ADDR + ADDR_ESPACE*20),
	MD_PARAM_ID_HALL_PHASESHIFT = (START_ADDR + ADDR_ESPACE*22),
	MD_PARAM_ID_POLEPAIRS 			= (START_ADDR + ADDR_ESPACE*23),
	MD_PARAM_ID_FF_C1 					= (START_ADDR + ADDR_ESPACE*24),
	MD_PARAM_ID_FW_KP 					= (START_ADDR + ADDR_ESPACE*26),
	MD_PARAM_ID_BATT_MAXVOLT 		= (START_ADDR + ADDR_ESPACE*28),
	MD_PARAM_ID_BATT_MINVOLT 		= (START_ADDR + ADDR_ESPACE*30),
} MD_Params_ID_t;
*/
/************************************ FUNCTIONS *************************************/

/* Event handler for managing the events given by the flash instantiation **/
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

/* Function to save all vehicle parameters in the static memory */
static void STRG_SaveAllVehicleParams( VParams_for_STRG_t vParams );

/* Function to read all vehicle parameters from static memory */
static bool STRG_ReadAllVehicleParams( VParams_for_STRG_t * vParams );

/*Function for update vehicle parameters from memory*/
static void STRG_UpdateVehicleFromMemory( VC_Handle_t * pHandle );

/*Function for saving vehicle parameters in memory*/
static void STRG_UpdateMemoryFromVehicle( VParams_for_STRG_t vParams) ;

/*Function for reset flash memory*/
static void STRG_ResetAllVehicleParams(void);

/**************** PUBLIC FUNCTIONS *******************************************/
/* Function to initialize the storage module*/
void STRG_Init( VC_Handle_t * pHandle  );

/* Task function for managing storage */
void TSK_StorageManagement(void * pvParameter);

/* Function for setting the motor parameter from the host*/
void STRG_setMotorParam(MD_Params_ID_t param, int32_t value, uint8_t motorSelect);

/* Function for setting vehicle parameters */
void STRG_setVehicleParam(VC_Params_ID_t, int32_t value);

/* Function for setting throttle parameters */
void STRG_setTHParam(TH_params_ID_t param, int32_t value);

/* Function for setting vehicle parameters */
void STRG_setDRTparams(int32_t value, uint8_t motorSelect, bool isThreshold);

/* Function for indicate if the storage manager is overwritting the memory*/
bool STRG_isOverWritting( void );

/* Function for rise the flag */
void STRG_setOverWrite( int8_t isOverWritting);

/****************************************************************************/

/* Function to save a vehicle parameter to static memory */
void STRG_SaveVehicleParam(VC_Params_ID_t param, int32_t value);

/* Function to read a vehicle parameter from static memory */
int32_t STRG_ReadVehicleParam(VC_Params_ID_t param);

/* Function to save a motor parameter to static memory */
void STRG_SaveMotorParam(uint8_t motorSelection, MD_Params_ID_t param, int32_t value);

/* Function to read a motor parameter from static memory */
int32_t STRG_ReadMotorParam(uint8_t motorSelection, MD_Params_ID_t param);

#endif
