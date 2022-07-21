/**
  ******************************************************************************
  * @file    lcd_apt_comm.h
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes APT LCD communication protocol.
  *
  ******************************************************************************
*/
	
#ifndef __LCD_APT_COMM_H
#define __LCD_APT_COMM_H

#include "vc_interface.h"
#include "uCAL_UART.h"
#include "gnr_main.h"

/**************************** DEFINITIONS AND STRUCTS ****************************/
// Commands
#define APT_START      0x55  // Fixed value that indicates the start of a frame.
#define APT_END        0x0D  // Fixed value that indicates the end of a frame.

// Display Read Cmd
// represents the location of where certain informations are in the frame. 
typedef enum
{
    PASS     = 1,
    SPEED    = 2,
    CURRENTL = 3,
    WHEELD   = 4, 
    SENSOR   = 5,
    CHECK    = 6 
}APT_Receive_t;
 

#define APT_MAX_BUFF_SIZE 13  // Max size for frame buffer.

typedef struct
{                         
    uint8_t Size;                         // Size of the Payload of the frame in bytes.
    uint8_t Buffer[APT_MAX_BUFF_SIZE];    // buffer containing the received data.
    uint8_t ByteCnt;                      // Byte counter used for byte by byte transmission/reception
}APT_frame_t;

typedef struct
{	
	UART_Handle_t *pUART_handle;    // Pointer to uart									
	VCI_Handle_t  *pVController;    // Pointer to vehicle
    APT_frame_t rx_frame; 		   	// Frame for data reception
	APT_frame_t tx_frame; 		   	// Frame for send response
    uint8_t RxByte;                 // Used for byte by byte reception
}APT_Handle_t;

/********************************* FUNCTIONS *******************************/

/**@brief Function for initializing the LCD module with the APT protocol
 *        It links its own custom interrupt routines with the UART callbacks using
 *        a void pointer. 
 *
 * @param pHandle: handle for APT module instance, pVCIHandle: Handle for vehicle controller,
 *            pUARTHandle: handle to the uart port being used 
 * @return nothing
 */
void LCD_APT_init(APT_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle);

/**@brief Function for building a frame specific to the APT protocol
 *        It is interrupt driven until a frame is completed where it  
 *        unblocks a comm task to process the frame.
 *        it is based on a byte by byte reception 
 *  
 * @param pVoidHandle: a void pointer that contaisn the handle of the 
 *        APT module instance
 *        
 * @return nothing
 */
void LCD_APT_RX_IRQ_Handler(void *ppVoidHandle);

/**@brief Function for sending a frame specific to the APT protocol
 *        once a frame has been received and processed this function
 *        sends the response made byt the frame process function.  
 *        It is based on a byte by byte transmission. 
 *  
 * @param pVoidHandle: a void pointer that contaisn the handle of the 
 *        APT module instance
 *        
 * @return nothing
 */
void LCD_APT_TX_IRQ_Handler(void *ppVoidHandle);

/**@brief Function for decoding a received frame (previously built in the RxCallback function)
 *        according to the APT screen protocol.
 *        This is executed in a comm task that gets unblocked when a complete frame is received.
 *
 * @param[in] pVoidHandle: a void pointer that contaisn the handle of the 
 *            APT module instance
 * @return nothing
 */
void LCD_APT_frame_Process(APT_Handle_t *pHandle);

#endif
