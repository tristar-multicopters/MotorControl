/**
* @file   uCAL_GPIO.h
* @brief  uController Abstraction Layer for General Purpose Inputs and Outputs
*
* This module is the interface that is used by the entire firmware
* to interact with the GPIOs. It is the bridge between the Renesas BSP reference
* for pins (ports and pins)and a FTEX standard pin numbering system 
* (first pin is pin 0 second pin is pin 1, etc). 
* If a change would be made to the hardware only 
* this file needs to be changed so that the GPIO are still usable.
*
*/

#ifndef UCAL_GPIO
#define UCAL_GPIO

#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"

// ======================== Pin configuration enums ======================== // 

typedef enum  // Used to select the pin direction
{
    Input,
    Output	
} Direction;

typedef enum  // Used to select if the pull up is enable
{
    None,
    Up
   //Down // Still part of the FTEX standard but not supported by renesas	
} PullType;

typedef enum  // Used to select the output
{
    PushPull,
    OpenDrain	
} OutputType;

// ================= Structure used to configure a pin ===================== //

struct GPIOConfig  //Contaisn the different parameters to configure a pin
{
    Direction  PinDirection;
    PullType   PinPull;
    OutputType PinOutput;
};

// ==================== Public function prototypes ========================= //

/**
  @brief Function used to initialise a GPIO using the renesas API
	
  @param Receives GPIO number and config struct
  @return void
*/
void uCAL_GPIO_Init(uint32_t aGPIO, struct GPIOConfig aPinConfig);

/**
  @brief Function used to read the state of a GPIO using the renesas API
	      Depending on if the pin is in output or input mode a different part of 
         the pin register is being read 
  
  @param Receives GPIO number
  @return void
*/
bool uCAL_GPIO_Read(uint32_t aGPIO);

/**
  @brief Function used to set a GPIO using the renesas API
	
  @param Receives GPIO number
  @return void
*/
void uCAL_GPIO_Set(uint32_t aGPIO);

/**
  @brief Function used to reset a GPIO using the renesas API
	
  @param Receives GPIO number
  @return void
*/
void uCAL_GPIO_Reset(uint32_t aGPIO);

/**
  @brief Function used to toggle a GPIO using the renesas API
	
  @param Receives GPIO number
  @return void
*/
void uCAL_GPIO_Toggle(uint32_t aGPIO);

#endif
