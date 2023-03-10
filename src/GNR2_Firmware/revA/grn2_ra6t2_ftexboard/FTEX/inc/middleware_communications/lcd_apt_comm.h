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
#include "powertrain_management.h"
#include "can_vehicle_interface.h"
#include "vc_errors_management.h"
#include "vc_constants.h"
#include "uCAL_UART.h"
#include "gnr_main.h"

/**************************** DEFINITIONS AND STRUCTS ****************************/
// Commands
#define APT_START      0x55  // Fixed value that indicates the start of a frame.
#define APT_END        0x0D  // Fixed value that indicates the end of a frame.

#define PAS_UNCHANGED  0x0A  // Value to idicate there are no change sin the PAS level

#define NUMBER_OF_CRC_BYTES 10  // Value used to tell how many bytes need to be included inside the CRC
#define BYTE_PER_PAIR        2  // Value that tells how many bytes per pair iside the CRC calculation  

#define RX_BYTE_BUFFER_SIZE 16

#define APT_WHEEL_DIAM_INCHES_MAX   34 // Max value received from the APT screen that represents a diamater in inches
#define APT_WHEEL_CIRCUMFERENCE_MIN 50 // Min value received from APT screen that represents a circumference in centimetres


// Display Read Cmd
// represents the location of where certain informations are in the frame. 
typedef enum
{
    PAS      = 1,
    SPEED    = 2,
    CURRENTL = 3,
    WHEELD   = 4, 
    SENSOR   = 5,
    CHECK    = 6 
}APT_Receive_t;
 
typedef enum
{
    // APT defined errors
    APT_NO_ERROR           = 0x00, // No error
    APT_COMM_ERROR         = 0x01, // Check the cable connection
    APT_CONTROL_PROTEC     = 0x02, // Check three-phase power line
    APT_THREE_PHASE_ERROR  = 0x03, // Check three-phase power line connection
    APT_BAT_LOW            = 0x04, // Battery low
    APT_BRAKE_ERROR        = 0x05, // Check the brake connection
    APT_TURN_ERROR         = 0x06, // Check turn to connect
    APT_HALL_ERROR         = 0x07, // Check the hall connection
    // Custom FTEX errors
    // From 0x08 to 0x9F EXCLUDING 0x30

}APT_ErrorCodes_t;
    
#define APT_MAX_BUFF_SIZE 13  // Max size for frame buffer.

typedef struct
{                         
    uint8_t Size;                         // Size of the Payload of the frame in bytes.
    uint8_t Buffer[APT_MAX_BUFF_SIZE];    // buffer containing the received data.
    uint8_t ByteCnt;                      // Byte counter used for byte by byte transmission/reception
}APT_frame_t;

typedef struct
{	
	UART_Handle_t *pUART_handle;           // Pointer to uart									
	VCI_Handle_t  *pVController;           // Pointer to vehicle
    APT_frame_t rx_frame; 		   	       // Frame for data reception
    APT_frame_t tx_frame; 		   	       // Frame for send response    
    
    uint8_t RxBuffer[RX_BYTE_BUFFER_SIZE]; // Hold bytes to be process by the APT task function
    uint8_t RxCount;                       // Counts how many bytes we are holding
    uint8_t RxByte;                        // Used for byte by byte reception
    uint8_t OldPAS;                        // Used to keep track of the current PAs levle on the screen 
    uint8_t WheelDiameter;                 // Used to hold the wheel diamater given by the screen
    
    bool CanChangePasFlag;                 // Used to tell the screen that PAS has bene changed from the Can interface
    bool APTChangePasFlag;                 // Used to tell the Can interfacne that the screen changed the PAS    
    
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
 * @param pVoidHandle: a void pointer that contains the handle of the 
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
 * @param pVoidHandle: a void pointer that contains the handle of the 
 *        APT module instance
 *        
 * @return nothing
 */
void LCD_APT_TX_IRQ_Handler(void *ppVoidHandle);

/**@brief Function for handling the regular task to manage the communication with
 *        an APT screen        
 *
 * @param[in] pHandle: handle for APT module instance
 *            
 * @return nothing
 */
void LCD_APT_Task(APT_Handle_t *pHandle);

/**@brief Function for decoding a received frame (previously built in the RxCallback function)
 *        according to the APT screen protocol.
 *        This is executed in a comm task that gets unblocked when a complete frame is received.
 *
 * @param[in] pHandle: handle for APT module instance 
 *            
 * @return nothing
 */
void LCD_APT_ProcessFrame(APT_Handle_t *pHandle);

/**@brief Function used to apply a filter on the speed we send to the screen
 *        
 * @param[in] instantaneous speed in RPM
 *            
 * @return filtered speed in RPM
 */
uint16_t LCD_APT_ApplySpeedFilter(uint16_t aInstantSpeedInRPM);

/**@brief Function used to apply a filter on the power we send to the screen        
 *
 * @param[in] instantaneous power in Amps
 *            
 * @return filtered power in Amps
 */
uint16_t LCD_APT_ApplyPowerFilter(uint16_t aInstantPowerInAmps);

/**@brief Function used to translate the PAS level received from the APT  
 *        screen standard to the FTEX standard
 *
 * @param[in] FTEX Standard PAS level 
 *            
 * @return APT standard PAS level
 */
uint8_t LCD_APT_ConvertPASLevelToAPT(PasLevel_t aPAS_Level);

/**@brief Function used to translate the FTEX standard PAS level to the APT  
 *        screen standard.(is not the same as when we receive a PAS level from the APT screen)
 *
 * @param[in] APT standard PAS level and Number of PAS levels
 *            
 * @return FTEX Standard PAS level
 */
PasLevel_t LCD_APT_ConvertPASLevelFromAPT(uint8_t aPAS_Level, uint8_t aNumberOfLevels);

/**@brief Function used to translate the FTEX standard PAS level to the APT  
 *        screen standard.(is not the same as when we receive a PAS level from the APT screen)
 *
 * @param[in] Value from APT screen if under 35 then value represents a diamater in inches.
 *            If the value is equal or greater then 50 the value represents the wheel     
 *            circumference in centimeters. if the value is outisde of the valid ranges then 
 *            the function will reutnr 28 inch wheel  
 *
 * @return the wheel diamater in inches
 */
uint8_t LCD_APT_CalculateWheelDiameter(uint16_t aValue);

/**@brief Function used to convert from standard FTEX error codes to APT error codes 
 *    
 * @param[in] an error to convert
 *            
 * @return the converted error
 */
uint8_t LCD_APT_ErrorConversionFTEXToAPT(uint8_t aError);


#endif
