/**
  ******************************************************************************
  * @file    serial_process.h
  * @brief   This file contain the header for serial process task
  ******************************************************************************
*/
#ifndef __SERIAL_PROCESS_H
#define __SERIAL_PROCESS_H

// =============================== Includes ================================ //
#include "serial_communication.h"
#include "external_flash_storage.h"
#include "internal_flash_storage.h"
#include "serial_response.h"

// ================================ Struct =============================== //
typedef struct datas {
	uint8_t startByte;
	uint8_t device;
	uint8_t type;
	uint8_t code;
	uint8_t msgLength;
	uint8_t message[20];
	uint8_t confByte; 
	uint8_t dataLength;
}data;

// ================================ Defines =============================== //

#define CONTROLLER_DEVICE   0x43 /* Using C character for EPC controller Device */
#define IOT_DEVICE          0x49 /* Using I character for EPI IOT Device */

/* Error Codes */
#define LENGTH_ERR			0x01 /* Frame Length error */
#define CONF_BYTE_ERR		0x02 /* Frame Confirm Byte error */
#define DEVICE_ERR			0x03 /* Frame Device error */
#define CODE_ERR			0x04 /* Frame Code Byte error */
#define FAILED_ERR			0x05 /* Test fail error */
#define NO_TEST_ERR			0x06 /* No Test performed error */
#define MSG_LENGTH_ERR		0x07 /* Frame Message Length error */
#define FRAME_ERR			0x08 /* Full Frame error */

/* Command code type */
#define CONTROLLER_GET   0x80   /* Controller get commande adress use */
#define CONTROLLER_SET   0xA0   /* Controller set commande adress use */
#define CONTROLLER_TEST  0xC0   /* Controller test commande adress use */

/* CONTROLLER_GET command code */
#define GET_SERIAL      0x81    // Get serial number from internal flash memory

/* CONTROLLER_SET command code */
#define SET_SERIAL      0xA1    // Set serial number using external ICT by saving  it to the internal flash memory


/* CONTROLLER_TEST command code */
#define TEST_FLASH      0xC1    // Test Serial flash integrety and format 

/* ---------------------------------------------------------------------- */

#define START       's' /* Frame Start */
#define COMMAND     'C' /* frame type as commande to be launched */
#define SET         'S' /* frame type as set to be launched  for setting a parmaeter */
#define GET         'G' /* frame type as get to be launched  for getting a parmaeter */
#define END         'E' /* Frame End */
#define CHECKFRAME  '!' /* Frame Check */

#define FLASHFORMAT '0' // sC0E!


#define STARTBYTE   (uint8_t)0 /* Frame first Byte */
#define PROCESSTYPE (uint8_t)1 /* Frame procees type byte*/

// ==================== Public function prototypes ======================== //

/**
  * @brief  Execute commands codes for test procedure
  * @param  None
  * @return None
  */
void SerialProcess_ExecuteCommand ();
  
/**
  * @brief  Serial Process that be runnig on main 
  * @param  None
  * @return None
  */
void SerialProcess_Main();

#endif /* __SERIAL_PROCESS_H */