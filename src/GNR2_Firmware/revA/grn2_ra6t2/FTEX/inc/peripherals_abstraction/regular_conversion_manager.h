/**
  * @file    regular_conversion_manager.h
  * @author  Ronak Nemade, FTEX inc
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file contains all definitions and functions prototypes for the
  *          regular_conversion_manager component of the Motor Control application.
*/

#ifndef __regular_conversion_manager_h
#define __regular_conversion_manager_h

#include "stdint.h"
#include "stdbool.h"
#include "r_adc_b.h"


// ====== Structure used to configure regular conversion manager ========== //

typedef struct  // Parameters to configure an ADC conversion element in array.
{
		uint8_t hChannel;
} RegConv_t;

// ==================== Public function prototypes ========================= //

/**
  @brief Function used to register ADC convesion as a part of sequence which will be executed by RCM_ExecRegularConv function call.

  @param Receives pointer to ADC handle which will be used in module where it is called.
  @return Returns registry position for a registered conversion in regular conversion manager array.
*/
uint8_t RegConvMng_RegisterRegConv(RegConv_t * regConv);

/**
  @brief Function used to flag start of sequential ADC conversion registered by regular conversion manager. RCM_ExecRegularConv function call is expected to occur anywhere in the RTOS tasks. % Function for future use. %

  @param Receives information on which sequential group needs to start converting its registries.
  @return void
*/
void RegConvMng_ExecuteGroupRegularConv(const adc_group_mask_t ADCGroupMask);

/**
  @brief Function used to flag start of sequential ADC conversion registered by regular conversion manager. RCM_ExecRegularConv function call is expected to occur anywhere in the RTOS tasks.

  @param Receives registry position for desired registered conversion in regular conversion manager array.
  @return Returns ADC conversion value .
*/
uint16_t RegConvMng_ReadConv(uint8_t handle);


#endif
