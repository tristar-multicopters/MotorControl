/**
  * @file    regular_conversion_manager.c
  * @brief   This file provides firmware functions that implement the following features
  *          of the regular_conversion_manager component of the Motor Control application:
  *           Register conversion with or without callback
  *           Execute regular conv directly from Temperature and VBus sensors
  *           Execute user regular conversion scheduled by medium frequency task
  *           Manage user conversion state machine
  *           +
  *           +
  *
*/

#include "regular_conversion_manager.h"
#include "hal_data.h"
#include "ASSERT_FTEX.h"


// ========================== Private defines ============================== //

#define RCM_MAX_CONV  8
#define ADC_EOC_STATUS_FLAGS 0x02u
#define ADC_EOC_CLEAR_FLAGS 0x1FEU

// ========================== Private variables ============================ //

RegConv_t * RCM_handle_array [RCM_MAX_CONV];
uint16_t RCM_ReadValue = 0;

// ========================================================================= //

/**
  * This function registers a regular ADC conversion that can be later scheduled for execution. It
  * returns a handle that uniquely identifies the conversion. This handle is used in the other API
  * of the Regular Converion Manager to reference the registered conversion.
  *
  * A regular conversion is defined by an ADC + ADC hChannel pair. If a registration already exists
  * for the requested ADC + ADC hChannel pair, the same handle will be reused.
  *
  * The registration may fail if there is no space left for additional conversions. The
  * maximum number of regular conversion that can be registered is defined by #RCM_MAX_CONV.
  */
uint8_t RegConvMng_RegisterRegConv(RegConv_t * regConv)
{
    uint8_t handle=255;
    uint8_t i=0;
    /* Parse the array to be sure that same
    * conversion does not already exist*/
    while (i < RCM_MAX_CONV)
    {
        if ( RCM_handle_array [i] == 0 && handle > RCM_MAX_CONV)
        {
            handle = i; /* First location available, but still looping to check that this config does not already exist*/
        }
        if ( RCM_handle_array [i] != 0)
        {
            if (RCM_handle_array [i]->hChannel == regConv->hChannel)
            {
                handle = i; /* Reuse the same handle */
                i = RCM_MAX_CONV; /* we can skip the rest of the loop*/
            }
        }
        i++;
    }
    if (handle < RCM_MAX_CONV)
    {
        RCM_handle_array[handle] = regConv;  // Register a regular conversion in array.
    }
    else
    {
        /* Nothing to do handle is already set to error value : 255 */
    }
    return handle;
}

/*
 * This function is used to flag start of scan group passed as input.
 */
void RegConvMng_ExecuteGroupRegularConv(const adc_group_mask_t ADCGroupMask)
{
		ASSERT(!(ADCGroupMask & ADC_GROUP_MASK_0)); // Input scan group cannot contain group 0, since it is reserved for PWM module
		ASSERT(ADCGroupMask <= ADC_GROUP_MASK_ALL); // Input scan cannot be higher than ADC_GROUP_MASK_ALL
	
		adc_status_t ADCStatus;
		
		R_ADC_B_StatusGet(g_adc0.p_ctrl, &ADCStatus);
		
//		while (ADCStatus.state != ADC_STATE_IDLE)
//		{
//		}
		
		// Issue here to start conversion with R_ADC_B_ScanGroupStart function. Group 2 works but not group 1.
		// As quick workaround, we use direct register manipulation with ADTRGENR and ADSYSTR.
		R_ADC_B->ADTRGENR |= ADCGroupMask;
		R_ADC_B->ADSYSTR |= ADCGroupMask;
		
		//R_ADC_B_ScanGroupStart(g_adc.p_ctrl, ADCGroupMask);
}

/**
 * This function reads data available in ADC result register
 */
uint16_t RegConvMng_ReadConv(uint8_t handle)
{
	uint16_t hConvResult = 0;
	
	R_ADC_B_Read(g_adc0.p_ctrl, RCM_handle_array[handle]->hChannel, &hConvResult);
	
	return hConvResult;
}

