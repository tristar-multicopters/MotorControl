/**
  ******************************************************************************
  * @file    storage_management.h
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module manages data storage in flash or external EEPROM, and software updates.
  *
	******************************************************************************
	*/
	
#ifndef __STORAGE_MANAGEMENT_H
#define __STORAGE_MANAGEMENT_H

#include <string.h>
#include <rtx_os.h>

#include "nrf_fstorage.h"
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"

/* DEFINES */
#define START_PAGE_ADDR  	 		0x00050000UL // Start address
#define LAST_PAGE_ADDR   	 		0x0005FFFFUL // End address
#define NB_OF_PAGES						2						 // number of pages
#define PAGE_SIZE 4096  // Max of elements in a flash memory page (1024 int32_t elements)
#define STRG_FLAG 0x20	// Flag for notifying the STRG_task to write in the memory

// Errors
#define STRG_NO_ERROR 	 0x00
#define STRG_READ_ERROR  0x01
#define STRG_WRITE_ERROR 0x02
#define STRG_ERASE_ERROR 0x04

typedef struct
{
	int32_t params_table[PAGE_SIZE/2]; // Contains two memory pages (0 to 1023 and 1024 to 2047)
	uint8_t error_code;
	bool isBusy;
} STRG_handle_t;
/************************************ FUNCTIONS *************************************/

/* Event handler for managing the events given by the flash instantiation **/
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

/* Function to save all vehicle parameters in the static memory */
static void STRG_SaveAllVehicleParams( void );

/* Function to populate the local table registers with storage params*/
static void STRG_ReadAllVehicleParams( void );

/*Function for reset flash memory*/
static void STRG_ResetAllVehicleParams( void );

/*Function for saving vehicle parameters in memory*/
static void STRG_UpdateMemoryPage( void ) ;

/**************** PUBLIC FUNCTIONS *******************************************/
/* Function to initialize the storage module*/
void STRG_Init( void );

/* Task function for managing storage */
__NO_RETURN void TSK_StorageManagement(void * pvParameter);

/* Function for get a parameter from the table of storage registers*/
int32_t STRG_getParam(uint16_t id);

/* Function for load a storage parameter from EVNC (or an external device) in the local buffer of parameters*/
void STRG_setParam(uint16_t id, int32_t value);

/* Function for knowing if the flash memory is busy been written */
bool STRG_isBusy( void );

#endif
