/**
  * @file    vc_errors_management.h
  * @brief   This module manages errors in the vehicle control layer
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VC_ERRORS_MANAGEMENT_H
#define __VC_ERRORS_MANAGEMENT_H

#include "gnr_main.h"


#define ERROR_BUFFER_SIZE       20
#define DEFAULT_CYCLE_LENGTH    6 
#define DEFAULT_HOLD_FRAMES     40
#define HOLD_UNTIL_CLEARED       0

typedef enum
{
    NO_ERROR          = 0x00,
    THROTTLE_STUCK    = 0x01, // On boot controller checks to make sure throttle is not above minimumu threshold
    UV_PROTECTION     = 0x02, // Under voltage protection
    OV_PROTECTION     = 0x03, // Over voltage protection
    UT_PROTECTION     = 0x04, // Undertemperature protection
    MOTOR_OT_PROTECT  = 0x05, // Motor over temperature protection
    MOTOR_FOLDBACK_TEMP = 0x06, // Foldback for motor temp has been initiated
    MOTOR_NTC_DISC_FREEZE    = 0x07,    // Motor temperature sensor is disconnected or cold warning
    CONTROLLER_OT_PROTECT  = 0x08, // Controller over temperature protection
    MOTOR_HALL_ERROR  = 0x09, // We can't read motor hall sensor
    MOTOR_PHASE_ERROR = 0x0A, // Motor phase wiring is faulty
    IOT_COMM_ERROR    = 0x0B, // Controller loses communication with IOT module
    DUAL_COMM_ERROR   = 0x0C, // Master lost comm with slave or slave lost comm with master
    OVER_CURRENT      = 0x0D, // Over Current protection - HW & SW
    BATT_LOW          = 0x0E, // Battery is low
    PAS_BOOT_ERROR    = 0x0F, // Peddle Assist error
    CONTROLLER_ERROR  = 0x10, // Controller in unrecoverable state
    BRAKE_ERROR       = 0x11, // Brake cutoff sensor abnormal
    UNMAPPED_ERROR    = 0xFF, // DO NOT FLAG used to fill in for errors we don't flag    
}
ErrorCodes_t; 

typedef struct
{
    ErrorCodes_t errorCodes[ERROR_BUFFER_SIZE];
    uint16_t errorHoldFrames[ERROR_BUFFER_SIZE]; // 0 means indefinitely.
    uint16_t errorCount;
    uint16_t cycleLength;

}VC_Errors_Handle_t;



/**@brief Function used to raise a specific error on the screen, cannot raise the same error twice. 
 *        
 *
 * @param[in] pHandle: handle for APT module instance, an APT error code 
 * @param errorHoldFrames: amount of frames to hold the error, 0 means indefinitely
 *                 
 * @return nothing
 */
void VC_Errors_RaiseError(ErrorCodes_t aError, uint16_t errorHoldFrames);

/**@brief Function used to clear a specific error on the screen.
 *        
 *
 * @param[in] pHandle: handle for APT module instance, an APT error code 
 *                 
 *
 * @return nothing
 */
void VC_Errors_ClearError(ErrorCodes_t aError);

/**@brief Function used to clear all of the errors present in the buffer
 *        
 *
 * @param[in] pHandle: handle for APT module instance
 *                
 *
 * @return nothing
 */
void VC_Errors_ClearAllErrors(void);

/**@brief Function used to cycle the next error to show on the screen 
 *        The speed at which the error cylce is define by a constant in the function
 *
 * @param[in] pHandle: handle for APT module instance
 *                 
 *
 * @return the error code that should be sent to the screen 
 */
ErrorCodes_t VC_Errors_CycleError(void);

/**@brief Function used to specify how many cycles of the function CycleError
 *        need to be done before its output will change         
 *
 * @param[in] a number of cycles to wait
 *                 
 *
 * @return nothing
 */
void VC_Errors_SetCycleLength(uint8_t aCycle);

#endif /* __VC_ERRORS_MANAGEMENT_H */

