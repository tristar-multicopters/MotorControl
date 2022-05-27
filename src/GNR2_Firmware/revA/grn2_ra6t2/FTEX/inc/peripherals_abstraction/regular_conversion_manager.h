/**
* @file   regular_conversion_manager.h
* @brief  Regular conversion manager module for non-injected ADC conversions.
*
* This file contains all definitions and functions prototypes for the regular_conversion_manager component of the Motor Control SDK.
* Register conversion without callback
* Execute regular conv directly from Temperature and VBus sensors
* Manage conversion state machine
*
*/

#ifndef __regular_conversion_manager_h
#define __regular_conversion_manager_h

#include "stdint.h"
#include "stdbool.h"
#include "r_adc_b.h"

// =================== Regular conversion manager enums =================== //

typedef enum  // Used to select ADC scan group which needs to start scan 
{
    GROUP_1=1,  // GROUP_0 reserved for PWM based Injected conversion
    GROUP_2,
    GROUP_3,
    GROUP_4,
    GROUP_5,
    GROUP_6,
    GROUP_7,
    GROUP_8,
    GROUP_9
} ScanGroup_t;

// ====== Structure used to configure regular conversion manager ========== //

typedef struct  // Parameters to configure an ADC conversion element in array.
{
    const adc_instance_t * regADC;
    uint8_t channel;
    uint8_t group;  // Group selection to be used in future
} RegConv_t;

// ==================== Public function prototypes ========================= //

/**
  @brief Function used to register ADC convesion as a part of sequence which will be executed by RCM_ExecRegularConv function call.    
 
  @param Receives pointer to ADC handle which will be used in module where it is called. 
  @return Returns registry position for a registered conversion in regular conversion manager array.  
*/
uint8_t RCM_RegisterRegConv(RegConv_t * regConv);

/**
  @brief Function used to flag start of sequential ADC conversion registered by regular conversion manager. RCM_ExecRegularConv function call is expected to occur anywhere in the RTOS tasks. % Function for future use. %   
 
  @param Receives information on which sequential group needs to start converting its registries.      
  @return void  
*/
void RCM_ExecuteGroupRegularConv(ScanGroup_t group);

/**
  @brief Function used to flag start of sequential ADC conversion registered by regular conversion manager. All ADC conversions specified in scan group 1 will executed with this function. CM_ExecRegularConvGroup1 function call is expected to occur anywhere in the RTOS tasks.   
 
  @param void.      
  @return void.  
*/
void RCM_ExecuteRegularConv(void);

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

#endif