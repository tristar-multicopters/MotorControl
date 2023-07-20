/**
  * @file    brake.c
  * @brief   This module handles brake sensor management
  *
  */

#include "brake.h"
#include "ASSERT_FTEX.h"

// ============================= Variables ================================ //
static uint16_t hSafeCounter = 0;
static bool brakeStuck = false;

/* Functions ---------------------------------------------------- */

/**
 *  Initializes brake sensor module
 */
void BRK_Init(BRK_Handle_t * pHandle, Delay_Handle_t * pBrakeDelay)
{	
    ASSERT(pHandle != NULL); 
    struct GPIOConfig PinConfig;
   
    PinConfig.PinDirection = INPUT;
    PinConfig.PinPull      = UP; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(pHandle->wPinNumber, PinConfig);

    pHandle->pBrakeStuckDelay = pBrakeDelay;
      
    ASSERT(pHandle->pBrakeStuckDelay->DelayInitialized); // Delay should be initialized in the task to specify at which 
                                                            // frequence the update delay function will eb called
    Delay_SetTime(pHandle->pBrakeStuckDelay, 5, SEC); // Set a 5 seconds delay to detect a stuck throttle
    Delay_Reset(pHandle->pBrakeStuckDelay);           // Make sure the counter is reset 
}

/**
 *  Returns state of the brake
 */
bool BRK_IsPressed(BRK_Handle_t * pHandle)
{
	ASSERT(pHandle != NULL); 

    bool bAux = uCAL_GPIO_Read(pHandle->wPinNumber);
    
    pHandle->bIsPressed = bAux ^ pHandle-> bIsInvertedLogic;
	
	return pHandle->bIsPressed;
}

/**
 *  MUST ONLY BE CALLED IN ONE PLACE
 *  Returns state of the brake and checks for stuck brake
 */
bool BRK_IsPressedSafety(BRK_Handle_t * pHandle)
{
	ASSERT(pHandle != NULL); 

    bool bAux = uCAL_GPIO_Read(pHandle->wPinNumber);
    pHandle->bIsPressed = bAux ^ pHandle-> bIsInvertedLogic;

    if(!pHandle->bSafeStart){
        if(!pHandle->bIsPressed){
            hSafeCounter++;
            /* Launch Safe Start after a delay counter */
            if (hSafeCounter >= SAFE_BRAKE_COUNT_100MS) 
            {   
                pHandle->bSafeStart = true;
                /* Clear this error in case it was falsly flagged as stuck (user brakes held at max on boot) */
                VC_Errors_ClearError(BRAKE_ERROR); 
                Delay_Reset(pHandle->pBrakeStuckDelay);
            }
        }
        /* Brake Sensor is detected */
        else     
        {
            hSafeCounter = 0;
            if (!brakeStuck)
            {
                /* Increase the counter for the error delay and check if the delay has been reached */
                if (Delay_Update(pHandle->pBrakeStuckDelay) == true) 
                {
                    VC_Errors_RaiseError(BRAKE_ERROR);
                    brakeStuck = true;
                }
            }
        }            
    }
	
	return pHandle->bIsPressed;
}
