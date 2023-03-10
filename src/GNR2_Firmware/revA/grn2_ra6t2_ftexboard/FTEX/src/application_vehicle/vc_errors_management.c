/**
  * @file    vc_errors_management.c
  * @brief   This module declares global structures used by vehicle control application
  *
  */

#include "vc_errors_management.h"

VC_Errors_Handle_t Error_Handle;

/**
 *  Function used to raise a specific error on the screen, cannot raise the same error twice. 
 */
void VC_Errors_RaiseError(ErrorCodes_t aError)
{
    bool ErrorAlreadyRaised = false;
    
    for (uint16_t u = 0; u < ERROR_BUFFER_SIZE; u ++) // Check if the error is already raised
    {
       if (Error_Handle.ErrorCodes[u] == aError)
       {
           ErrorAlreadyRaised = true; 
       }
    }    
    
    if (ErrorAlreadyRaised == false) // If the error is already raised there is not point in raising it again
    {        
        if (Error_Handle.NumberErrors < ERROR_BUFFER_SIZE) // If the error buffer isn't full 
        {
            Error_Handle.NumberErrors ++; // increment the counter
        }
        else // If its full erase the oldlest error and add the new one
        {
            for (uint16_t i = 0; i < ERROR_BUFFER_SIZE - 1 ; i ++) // Shift the other ones to make room at the end of the buffer
            {
                Error_Handle.ErrorCodes[i] = Error_Handle.ErrorCodes[i + 1];        
            }                        
        }
    
        Error_Handle.ErrorCodes[Error_Handle.NumberErrors-1] = aError; //Add the new error
    }
}

/**
 *  Function used to clear a specific error on the screen.
 */
void VC_Errors_ClearError(ErrorCodes_t aError)
{
    bool ErrorFound = false;
    bool ErrorCleared = false;
    
    for (uint16_t i = 0; i < Error_Handle.NumberErrors; i++) // Cycle through the buffer once we find the error 
    {                                                        // shift the others to ensure there is no empty spot 
                                                             // in the middle of the buffer.
    
        if (Error_Handle.ErrorCodes[i] == aError) // Is this error raised in the buffer ?
        {
            ErrorFound = true;  
        }            
        
        if (ErrorFound) // If the error is present in the buffer
        {
            if (i < Error_Handle.NumberErrors - 1) // Making sure we are not on the last error present in the buffer
            {
                Error_Handle.ErrorCodes[i] = Error_Handle.ErrorCodes[i+1]; // If not shift the other errors to ensure
            }                                                      // we don't have an empty spot in the buffer
            else // If we are on the last error just clear it
            {
                Error_Handle.ErrorCodes[i] = NO_ERROR;
                ErrorCleared = true;                
            }                 
        }      
    }
    
    if (ErrorCleared) // If an error was cleared reflect the change in the error counter
    {
        Error_Handle.NumberErrors --;
    }
}

/**
 *  Function used to clear all of the errors present in the buffer
 */
void VC_Errors_ClearAllErrors(void)
{
    for (uint16_t i = 0; i < Error_Handle.NumberErrors; i++)
    {
       Error_Handle.ErrorCodes[i] = NO_ERROR; // Clear all of the raised errors
    }
    
    Error_Handle.NumberErrors = 0; // Reset the error counter    
} 


/** Function used to cycle the next error to show on the screen 
 *  The speed at which the error cylce is define by a constant in the function
 */
ErrorCodes_t VC_Errors_CycleError(void)
{
    uint16_t CycleLenght; // Number of cycles that we show an error before switching
                          // If this is 0 then only the first error will be shown
    static uint8_t CycleTracking;
    if(Error_Handle.Cycle_Lenght >= 0)
    {
        CycleLenght = Error_Handle.Cycle_Lenght;
    }
    else
    {
        CycleLenght = DEFAULT_CYCLE_LENGHT;
    }
    
    
    // Keeps track how many cycles has the current error been shown for
        
    static  uint16_t ShownErrorIndex;
    static  ErrorCodes_t ErrorShown; 
    
    
    if (CycleTracking >= CycleLenght) // Check if we need to cycle to show the next error
    {
        CycleTracking = 0;
        
        if (Error_Handle.NumberErrors <= 0) // If the error(s) got cleared since the last cycle
        {
            ErrorShown = NO_ERROR;     // Show no error  
            ShownErrorIndex = 0;
        }       
        else 
        {    
            if (ShownErrorIndex >= Error_Handle.NumberErrors - 1) // reset the cycle if we have shown all of them
            {
                ShownErrorIndex = 0;        
            }   
            else if(CycleLenght > 0)  // If not incremente the index to shw the next error in line
            {
                ShownErrorIndex ++;                        
            }
        
            ErrorShown = Error_Handle.ErrorCodes[ShownErrorIndex];
        }
    }
    else
    {
        CycleTracking ++;     
    }        
   
    return ErrorShown;
}

void VC_Errors_SetCycleLenght(uint8_t aCycle)
{    
    Error_Handle.Cycle_Lenght = aCycle;
}

