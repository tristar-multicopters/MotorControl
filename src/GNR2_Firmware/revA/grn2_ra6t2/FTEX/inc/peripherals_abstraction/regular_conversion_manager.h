/**
* @file   regular_conversion_manager.h
* @brief  This file contains all definitions and functions prototypes for the regular_conversion_manager component of the Motor Control SDK.
*
* This module is the interface that is used by the entire firmware
* to interact with the GPIOs. It is the bridge between the Renesas BSP reference
* for pins (ports and pins)and a FTEX standard pin numbering system 
* (first pin is pin 0 second pin is pin 1, etc). 
* If a change would be made to the hardware only 
* this file needs to be changed so that the GPIO are still usable.
*
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __regular_conversion_manager_h
#define __regular_conversion_manager_h

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "r_adc_b.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief RegConv_t contains all the parameters required to execute a regular conversion
  *
  * it is used by all regular_conversion_manager's client
  *
  */
typedef enum
{
    GROUP_1 = 1,       // GROUP_0 reserved for PWM based Injected conversion
    GROUP_2,
    GROUP_3,
    GROUP_4,
    GROUP_5,
    GROUP_6,
    GROUP_7,
    GROUP_8,
    GROUP_9
} ScanGroup;

typedef struct
{
    const adc_instance_t * regADC;
    uint8_t channel;
} RegConv_t;

typedef void (*RCM_exec_cb_t)(uint8_t handle, uint16_t data, void *UserData);

/* Exported functions ------------------------------------------------------- */
/**
  @brief Function used to register ADC convesion as a part of sequence which will be executed by RCM_ExecRegularConv function call.    
 
  @param Receives pointer to ADC handle which will be used in module where it is called. 
  @return Returns registry position for a registered conversion in regular conversion manager array.  
*/
uint8_t RCM_RegisterRegConv(RegConv_t * regConv);

/**
  @brief Function used to flag start of sequential ADC conversion registered by regular conversion manager. RCM_ExecRegularConv function call is expected to occur anywhere in the RTOS tasks.   
 
  @param Receives information on which sequential group needs to start converting its registries.      
  @return void  
*/
void RCM_ExecuteRegularConv(ScanGroup group);

/**
  @brief Function used to flag start of sequential ADC conversion registered by regular conversion manager. RCM_ExecRegularConv function call is expected to occur anywhere in the RTOS tasks.   
 
  @param Receives registry position for desired registered conversion in regular conversion manager array.      
  @return Returns ADC conversion value .  
*/
uint16_t RCM_ReadConv(uint8_t handle);

/**
  @brief Function used to enable regular conversion manager. Registered ADC conversions will not execute unless the manager is enabled.    
 
  @param void.      
  @return void.  
*/
void RCM_EnableConv(void);

/**
  @brief Function used to disable regular conversion manager. Registered ADC conversions will stop executing if the manager is enabled.    
 
  @param void.      
  @return void.  
*/
void RCM_DisableConv(void);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __regular_conversion_manager_h */