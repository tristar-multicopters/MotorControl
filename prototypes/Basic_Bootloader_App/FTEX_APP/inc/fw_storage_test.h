/**
  * @file    fw_storage_test.h
  * @brief   This file is the device firmware storage test header
	*/

#ifndef __FW_STORAGE_TEST_H
#define __FW_STORAGE_TEST_H

/* Includes ------------------------------------------------------------------*/
#include "fw_storage.h"
#include "Basic_Bootloader_App.h"
/* Defines --------------------------------------------------------------------*/


/* Function Prototypes ------------------------------------------------------------*/

/*
 * @brief Used for testing purpose
 *		This function does a full file creation, write and readout process with dummy data.
 * @param nothing
 * @eturn nothing
**/
void fw_storage_test( void );

/*
 * @brief Used for testing purpose
 *		This function does a large file read operation. This allows checking memory content in debug.
 * @param nothing
 * @eturn nothing
**/
void fw_storage_test_read( void );

#endif /* __FW_STORAGE_TEST_H */