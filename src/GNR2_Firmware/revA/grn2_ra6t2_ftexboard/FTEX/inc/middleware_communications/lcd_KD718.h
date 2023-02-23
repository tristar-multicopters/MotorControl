/**
  ******************************************************************************
  * @file    lcd_KD718.h
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes KD718 communication protocol.
  *
  ******************************************************************************
*/
	
#ifndef __LCD_KD718_H
#define __LCD_KD718_H

#include "vc_interface.h"
#include "powertrain_management.h"
#include "can_vehicle_interface.h"
#include "vc_constants.h"
#include "uCAL_UART.h"
#include "gnr_main.h"

/**************************** DEFINITIONS AND STRUCTS ****************************/
#define KD718_RX_BYTE_BUFFER_SIZE 8


// Commands
#define READ_COMMAND  0x11
#define WRITE_COMMAND 0x16

// Display Write Sub Cmd
typedef enum
{
   W_PAS       = 0x0B,
   W_LIGHTS    = 0x1A,   
}
KD718_WriteType_t;

// Display Read Sub Cmd
typedef enum
{
    R_PROTOCOL        = 0x90,
    R_STATUS          = 0x08,
    R_PWN             = 0x06,
    R_MOTOR_COMU      = 0x07, 
    R_VOLTAGE         = 0x01,
    R_CURRENT         = 0x0A,
    R_MODE            = 0x0B,
    R_THROTTLE_TARGET = 0x0E,
    R_BRAKE_STATE     = 0x0F,
    R_ACC_PULSE_NB    = 0x10,
    R_SPD_HALL_CYCLE  = 0x1D,
    R_SPEED           = 0x20,    
}KD718_ReadType_t;
 

#define KD718_MAX_BUFF_SIZE 8  // Max size for frame buffer.

typedef struct
{                         
    uint8_t Size;                          // Size of the Payload of the frame in bytes.
    uint8_t Buffer[KD718_MAX_BUFF_SIZE];   // buffer containing the received data.
    uint8_t ByteCnt;                       // Byte counter used for byte by byte transmission/reception
}KD718_frame_t;

typedef struct
{	
	UART_Handle_t *pUART_handle;           // Pointer to uart									
	VCI_Handle_t  *pVController;           // Pointer to vehicle
    KD718_frame_t rx_frame; 		   	   // Frame for data reception
	KD718_frame_t tx_frame; 		   	   // Frame for send response
    
    uint8_t RxBuffer[KD718_RX_BYTE_BUFFER_SIZE]; // Hold bytes to be process by the KD718 task function
    uint8_t RxCount;                       // Counts how many bytes we are holding
    uint8_t RxByte;                        // Used for byte by byte reception
    uint8_t WheelDiameter;                 // Used to hold the wheel diamater given by the screen
         
}KD718_Handle_t;

/********************************* FUNCTIONS *******************************/

/**@brief Function for initializing the LCD module with the KD718 protocol
 *        It links its own custom interrupt routines with the UART callbacks using
 *        a void pointer. 
 *
 * @param pHandle: handle for KD718 module instance, pVCIHandle: Handle for vehicle controller,
 *            pUARTHandle: handle to the uart port being used 
 * @return nothing
 */
void LCD_KD718_init(KD718_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle);

/**@brief Function for building a frame specific to the KD718 protocol
 *        It is interrupt driven until a frame is completed where it  
 *        unblocks a comm task to process the frame.
 *        it is based on a byte by byte reception 
 *  
 * @param pVoidHandle: a void pointer that contaisn the handle of the 
 *        KD718 module instance
 *        
 * @return nothing
 */
void LCD_KD718_RX_IRQ_Handler(void *ppVoidHandle);

/**@brief Function for sending a frame specific to the KD718 protocol
 *        once a frame has been received and processed this function
 *        sends the response made byt the frame process function.  
 *        It is based on a byte by byte transmission. 
 *  
 * @param pVoidHandle: a void pointer that contains the handle of the 
 *        KD718 module instance
 *        
 * @return nothing
 */
void LCD_KD718_TX_IRQ_Handler(void *ppVoidHandle);

/**@brief Function for handling the regular task to manage the communication with
 *        an KD718 screen        
 *
 * @param[in] pVoidHandle: a void pointer that contains the handle of the 
 *            KD718 module instance
 * @return nothing
 */
void LCD_KD718_Task(KD718_Handle_t *pHandle);

/**@brief Function for decoding a received frame (previously built in the RxCallback function)
 *        according to the KD718 screen protocol.
 *        This is executed in a comm task that gets unblocked when a complete frame is received.
 *
 * @param[in] pVoidHandle: a void pointer that contains the handle of the 
 *            KD718 module instance
 * @return nothing
 */
void LCD_KD718_ProcessFrame(KD718_Handle_t *pHandle);

/**@brief Function used to apply a filter on the speed we send to the screen
 *        
 * @param[in] instantaneous speed in RPM
 *            
 * @return filtered speed in RPM
 */
uint16_t LCD_KD718_ApplySpeedFilter(uint16_t aInstantSpeedInRPM);

/**@brief Function used to apply a filter on the power we send to the screen        
 *
 * @param[in] instantaneous power in Amps
 *            
 * @return filtered power in Amps
 */
uint16_t LCD_KD718_ApplyPowerFilter(uint16_t aInstantPowerInAmps);

/**@brief Function used to translate the PAS level received from the KD718  
 *        screen standard to the FTEX standard
 *
 * @param[in] pVoidHandle: a void pointer that contaisn the handle of the 
 *            KD718 module instance
 * @return nothing
 */
uint8_t LCD_KD718_ConvertPASLevelToKD718(PasLevel_t aPAS_Level);

/**@brief Function used to translate the FTEX standard PAS level to the KD718  
 *        screen standard.(is not the same as when we receive a PAS level from the KD718 screen)
 *
 * @param[in] pVoidHandle: a void pointer that contaisn the handle of the 
 *            KD718 module instance
 * @return nothing
 */
PasLevel_t LCD_KD718_ConvertPASLevelFromKD718(uint8_t aPAS_Level, uint8_t aNumberOfLevels);

#endif
