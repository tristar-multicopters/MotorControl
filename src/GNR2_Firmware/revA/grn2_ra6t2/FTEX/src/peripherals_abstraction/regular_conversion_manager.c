/**
*  regular_conversion_manager.c
*  This file provides firmware functions that implement the following features of the regular_conversion_manager component of the Motor Control SDK:
*  Register conversion without callback
*  Execute regular conv directly from Temperature and VBus sensors
*  Manage conversion state machine
*/ 

#include "regular_conversion_manager.h"
#include "mc_config.h"

// ========================== Private typedef ============================== //

typedef enum    // Used to define state of regular conversion manager
{
    stop,
    start,
    ongoing,
    dataavailable
} RCM_status_t;

// ========================== Private defines ============================== //

#define RCM_MAX_CONV  8 

// ========================== Private variables ============================ //

RegConv_t * RCM_handle_array [RCM_MAX_CONV];
RCM_status_t ConversionStatus = stop;
uint16_t RCM_ReadValue = 0;

// ========================================================================= //

/**
  * @brief  Registers a regular conversion.
  *
  * This function registers a regular ADC conversion that can be later scheduled for execution. It
  * returns a handle that uniquely identifies the conversion. This handle is used in the other API
  * of the Regular Converion Manager to reference the registered conversion.
  *
  * A regular conversion is defined by an ADC + ADC channel pair. If a registration already exists
  * for the requested ADC + ADC channel pair, the same handle will be reused.
  *
  * The registration may fail if there is no space left for additional conversions. The
  * maximum number of regular conversion that can be registered is defined by #RCM_MAX_CONV.
  *
  * @param  regConv Pointer to the regular conversion parameters.
  *         Contains ADC, Channel and scan group to be used.
  *
  *  @retval the handle of the registered conversion or 255 if the registration failed
  */
uint8_t RCM_RegisterRegConv(RegConv_t * regConv)
{
    uint8_t handle=255;
    uint8_t i=0;
    /* Parse the array to be sure that same
    * conversion does not already exist*/
    while (i < RCM_MAX_CONV)
    {
        if (  RCM_handle_array [i] == 0 && handle > RCM_MAX_CONV)
        {
            handle = i; /* First location available, but still looping to check that this config does not already exist*/
        }
        /* Ticket 64042 : If RCM_handle_array [i] is null access to data member will cause Memory Fault. */
        if (  RCM_handle_array [i] != 0 )
        {
            if ( RCM_handle_array [i]->channel == regConv->channel )
            {
                handle = i; /* Reuse the same handle */
                i = RCM_MAX_CONV; /* we can skip the rest of the loop*/
            }
        }
        i++;
    }    
    if (handle < RCM_MAX_CONV )
    {
        RCM_handle_array[handle] = regConv;
    }
    else
    {
        /* Nothing to do handle is already set to error value : 255 */
    }
    return handle;
}

/*
 * This function is used to flag start of scan group passed as input.
 * Depending of the state of RCM manager for previous conversion, this function can change state of RCM manager and flag availability of scanned channel data. 
 * If the ADC is already in use for currents sensing, the regular conversion can not
 * be executed instantaneously but have to be scheduled in order to be executed.
 * inside HF task.
 * If it is possible to execute the conversion instantaneously, it will be executed.
 * Otherwise, the latest stored conversion result will be returned.
 *
 * NOTE: This function is not completely defined. RegConv_t.group will be used to register scan group in which the adc conversion will be placed. 
 */
void RCM_ExecuteGroupRegularConv(ScanGroup group)
{
    if(ConversionStatus == ongoing && R_ADC_B->ADSCANENDSR >= 0x2)
    {
        R_ADC_B->ADSCANENDSCR = 0x1FEU;
        ConversionStatus = dataavailable;
    }
    else if(ConversionStatus == start | ConversionStatus == dataavailable)
    {
        R_ADC_B->ADSTR[group] |= 1UL << 0;
        ConversionStatus = ongoing;
    }   
}

/*
 * This function is used to flag start of scan group 1.
 * Depending of the state of RCM manager for previous conversion, this function can change state of RCM manager and flag availability of scanned channel data. 
 * If the ADC is already in use for currents sensing, the regular conversion can not
 * be executed instantaneously but have to be scheduled in order to be executed after currents sensing
 * inside HF task.
 * If it is possible to execute the conversion instantaneously, it will be executed.
 * Otherwise, the latest stored conversion result will be returned.
 */
void RCM_ExecuteRegularConv(void)
{
    if(ConversionStatus == ongoing && R_ADC_B->ADSCANENDSR >= 0x2)
    {
        R_ADC_B->ADSCANENDSCR = 0x1FEU;
        ConversionStatus = dataavailable;
    }
    else if(ConversionStatus == start | ConversionStatus == dataavailable)
    {
        R_ADC_B->ADSTR[1] |= 1UL << 0;  // setting last bit in element 1 of array ADSSTR flags start of conversion in scan group 1. 
        ConversionStatus = ongoing;
    }
}

/**
 * @brief Reads scanned ADC data using regular conversionmanager.
 *
 * This function reads data available in ADC result register if end of conversion flag is received ands state of manager is changed to "dataavailable"
 *
 * @param  handle used to identify which unique registered conversion data needs to be accessed.
 *
 * @return Returns ADC read data.
 */
uint16_t RCM_ReadConv(uint8_t handle)
{
    if (ConversionStatus == dataavailable)
    {
        RCM_ReadValue = (uint16_t) R_ADC_B->ADDR[RCM_handle_array[handle]->channel];
    }
    return RCM_ReadValue;
}

/**
 * @brief Enables regular conversion manager for execution.
 *
 * This function changes regular conversion manager state to start so that manager can receive execution commands.
 *
 * @param  void.
 *
 * @return void.
 */
void RCM_EnableConv(void)
{
    if(ConversionStatus == stop)
    {
        ConversionStatus = start;
    }    
}

/**
 * @brief Enables regular conversion manager for execution.
 *
 * This function changes regular conversion manager state to stop so that manager can not receive execution commands.
 *
 * @param  void.
 *
 * @return void.
 */
void RCM_DisableConv(void)
{
    if(ConversionStatus == dataavailable || ConversionStatus == ongoing )
    {
        ConversionStatus = stop;
    }
}
