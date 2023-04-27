/**
  ******************************************************************************
  * @file    Main.h
  * @brief   This file contain the header for main firmware application
  ******************************************************************************
*/
#ifndef __SERIAL_PROCESS_H
#define __SERIAL_PROCESS_H

// =============================== Includes ================================ //
#include "serial_communication.h"
#include "external_flash_storage.h"

// ================================ Defines =============================== //

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


bool Serial_Frame_Check (uint8_t Length, uint8_t Tab[Length] );

void Serial_Process_Launch (void);

void Serial_Frame_Process(void);
    

#endif /* __SERIAL_PROCESS_H */