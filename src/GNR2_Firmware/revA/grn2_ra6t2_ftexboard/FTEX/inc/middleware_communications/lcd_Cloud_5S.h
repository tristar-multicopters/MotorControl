/**
  ******************************************************************************
  * @file    lcd_Cloud_5S.h
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes Cloud 5S LCD communication protocol.
  *
  ******************************************************************************
*/
    
#ifndef __LCD_CLOUD_5S_H
#define __LCD_CLOUD_5S_H

#include "vc_interface.h"
#include "powertrain_management.h"
#include "can_vehicle_interface.h"
#include "vc_errors_management.h"
#include "vc_constants.h"
#include "uCAL_UART.h"
#include "gnr_main.h"

/**************************** DEFINITIONS AND STRUCTS ****************************/
// Commands
#define CLOUD_SYSTEM     0x53      // Systeme config frame, only called at the start of the communication
#define CLOUD_SYSTEM_FRAME_LENGTH  17

#define CLOUD_RUNTIME    0x52      // Run time frame, contains the data that we need to exchange with the screen while the bike is operating
#define CLOUD_RUNTIME_FRAME_LENGTH 11


#define CLOUD_START      0x3A      // Start code
#define CLOUD_SLAVE_ID   0x1A

#define CLOUD_END_CODE_1  0x0D     // Frame always had these two end code
#define CLOUD_END_CODE_2  0x0A

#define CLOUD_ASSIST_TYPE 0x80     // Masked used to obtain the assistance type (torque/cadence)

#define CLOUD_TORQ_ASSIST_TYPE  0x80  // Torque PAS assist type
 
#define CLOUD_CADE_ASSIST_TYPE  0x00  // Cadence PAS assist type

#define CLOUD_POST_CRC_BYTE_NB 4 //Specify how many bytes at the end of the frame need to be excluded from CRC calculation

#define CLOUD_RX_BUFFER_SIZE 32

#define CLOUD_DEFAULT_SPEED_PERIOD 4000 // when the wheel isn't spinning this is the value sent to the display    

#define CLOUD_X_THROTTLE_MAX     190    // Maximum value for clouddrive external throttle
#define CLOUD_X_THROTTLE_OFFSET   76    // Offset value for clouddrive external throttle
// Display Read Cmd
// represents the location of where certain informations are in the frame. 
 
typedef enum
{
    // Cloud 5S errors codes defined errors
    CLOUD_5S_NO_ERROR       = 0x00, // No error
    CLOUD_5S_PAS_BOOT_ERROR = 0x01, // PAS boot anamoly
    CLOUD_5S_CNTL_ERROR     = 0x02, // Controller anamolies
    CLOUD_5S_PHASE_ERROR    = 0x03, // Phase loss
    CLOUD_5S_UVP            = 0x04, // Under voltage
    CLOUD_5S_BRAKE_ERROR    = 0x05, // Brake abnormal 
    CLOUD_5S_HALL_ERROR     = 0x06, // motor hall abnormal
    CLOUD_5S_THROTTLE_ERROR = 0x07, // throttle abnormal
    CLOUD_5S_UT_ERROR       = 0x08, // controller undertemperature error
    CLOUD_5S_OT_ERROR       = 0x09, // controller overtemperature error
    CLOUD_5S_IOT_COMM_ERROR = 0x0A, // iot communication error
    CLOUD_5S_MOT_ERROR      = 0x0B, // controller overtemperature error
    CLOUD_5S_OV_ERROR       = 0x0C, // DC bus goes over the safe voltage operating range of the controller
    CLOUD_5S_LOW_BAT        = 0x0D, // low battery error
    CLOUD_5S_RESERVED       = 0x0F, // Cloud drive's internal code

}Cloud_5S_ErrorCodes_t;
    
typedef struct
{                         
    uint8_t Size;                           // Size of the Payload of the frame in bytes.
    uint8_t Buffer[CLOUD_RX_BUFFER_SIZE];   // buffer containing the received data.
    uint8_t ByteCnt;                        // Byte counter used for byte by byte transmission/reception
}Cloud_5S_frame_t;

typedef struct
{    
    UART_Handle_t *pUART_handle;            // Pointer to uart                                    
    VCI_Handle_t  *pVController;            // Pointer to vehicle
    Cloud_5S_frame_t rx_frame;                    // Frame for data reception
    Cloud_5S_frame_t tx_frame;                    // Frame for send response    
    
    uint8_t RxBuffer[CLOUD_RX_BUFFER_SIZE]; // Hold bytes to be process by the APT task function
    uint8_t RxCount;                        // Counts how many bytes we are holding
    uint8_t RxByte;                         // Used for byte by byte reception
    uint8_t OldPAS;                         // Used to keep track of the current Pas levle on the screen 
    uint8_t WheelDiameter;                  // Used to hold the wheel diamater given by the screen
     
    bool cloud5SChangePasFlag;              // Used to tell the Can interfacne that the screen changed the PAS    
    bool isScreenSlave;                     // Listening to the screen for pas changes, unless the app changes it
                                            // At that point the screen will become slave to the controller until it's updated.
    
}Cloud_5S_Handle_t;

/********************************* FUNCTIONS *******************************/

/**@brief Function for initializing the LCD module with the Cloud 5S protocol
 *        It links its own custom interrupt routines with the UART callbacks using
 *        a void pointer. 
 *
 * @param pHandle: handle for Cloud 5S module instance, pVCIHandle: Handle for vehicle controller,
 *            pUARTHandle: handle to the uart port being used 
 * @return nothing
 */
void LCD_Cloud_5S_init(Cloud_5S_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle);

/**@brief Function for building a frame specific to the Cloud 5S protocol
 *        It is interrupt driven until a frame is completed where it  
 *        unblocks a comm task to process the frame.
 *        it is based on a byte by byte reception 
 *  
 * @param pVoidHandle: a void pointer that contains the handle of the 
 *        Cloud 5S module instance
 *        
 * @return nothing
 */
void LCD_Cloud_5S_RX_IRQ_Handler(void *pVoidHandle);

/**@brief Function for sending a frame specific to the Cloud 5S protocol
 *        once a frame has been received and processed this function
 *        sends the response made byt the frame process function.  
 *        It is based on a byte by byte transmission. 
 *  
 * @param pVoidHandle: a void pointer that contains the handle of the 
 *        Cloud 5S module instance
 *        
 * @return nothing
 */
void LCD_Cloud_5S_TX_IRQ_Handler(void *pVoidHandle);

/**@brief Function for handling the regular task to manage the communication with
 *        an Cloud 5S screen        
 *
 * @param[in] pHandle: handle for APT module instance
 *            
 * @return nothing
 */
void LCD_Cloud_5S_Task(Cloud_5S_Handle_t *pHandle);

/**@brief Function for decoding a received frame (previously built in the RxCallback function)
 *        according to the Cloud 5S screen protocol.
 *        This is executed in a comm task that gets unblocked when a complete frame is received.
 *
 * @param[in] pHandle: handle for Cloud 5S module instance 
 *            
 * @return nothing
 */
void LCD_Cloud_5S_ProcessFrame(Cloud_5S_Handle_t *pHandle);

/**@brief Function used to apply a filter on the power we send to the screen        
 *
 * @param[in] instantaneous power in Amps
 *            
 * @return filtered power in Amps
 */
uint16_t LCD_Cloud_5S_ApplyPowerFilter(uint16_t aInstantPowerInAmps);

/**@brief Function used to translate the PAS level received from the Cloud 5S  
 *        screen standard to the FTEX standard
 *
 * @param[in] FTEX Standard PAS level 
 *            
 * @return Cloud 5S standard PAS level
 */
uint8_t LCD_Cloud_5S_ConvertPASLevelToCloud_5S(PasLevel_t aPAS_Level);

/**@brief Function used to translate the FTEX standard PAS level to the Cloud 5S  
 *        screen standard.(is not the same as when we receive a PAS level from the APT screen)
 *
 * @param[in] Cloud 5S standard PAS level and Number of PAS levels
 *            
 * @return FTEX Standard PAS level
 */
PasLevel_t LCD_Cloud_5S_ConvertPASLevelFromCloud_5S(uint8_t aPAS_Level, uint8_t aNumberOfLevels);

/**@brief Function used to translate the FTEX standard PAS level to the Cloud 5S  
 *        screen standard.(is not the same as when we receive a PAS level from the Cloud 5S screen)
 *
 * @param[in] Value from Cloud 5S screen if under 35 then value represents a diamater in inches.
 *            If the value is equal or greater then 50 the value represents the wheel     
 *            circumference in centimeters. if the value is outisde of the valid ranges then 
 *            the function will reutnr 28 inch wheel  
 *
 * @return the wheel diamater in inches
 */
uint8_t LCD_Cloud_5S_CalculateWheelDiameter(uint16_t aValue);

/**@brief Function used to convert from standard FTEX error codes to Cloud 5S error codes 
 *    
 * @param[in] an error to convert
 *            
 * @return the converted error
 */
uint8_t LCD_Cloud_5S_ErrorConversionFTEXToCloud_5S(uint8_t aError);

/**@brief Function used to compute the checksum values to verify frames
 *    
 * @param[in] pHandle: handle for Cloud 5S module instance, two pointers to update the checksum results 
 *            
 * @return void
 */
void LCD_Cloud_5S_ComputeChecksum(Cloud_5S_frame_t aFrame, uint8_t *pCheckLow, uint8_t *pCheckHigh);

#endif
