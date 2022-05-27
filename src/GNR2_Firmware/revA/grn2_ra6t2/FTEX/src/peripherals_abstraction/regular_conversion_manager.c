/**
  ******************************************************************************
  * @file    regular_conversion_manager.c
  * @author  FTEX inc
  * @brief   This file provides firmware functions that implement the following features
  *          of the regular_conversion_manager component of the Motor Control SDK:
  *          Register conversion without callback
  *          Execute regular conv directly from Temperature and VBus sensors
  *          Manage conversion state machine
  *          +
  *          +
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "regular_conversion_manager.h"
#include "mc_config.h"

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief Defining state machine of regular conversion manager
  *
  * ...
  */
typedef enum 
{
    stop,
    start,
    ongoing,
    dataavailable
} RCM_status_t;

/* Private defines -----------------------------------------------------------*/
/**
  * @brief Number of regular conversion allowed By default.
  *
  * In single drive configuration, 2 of them are consumed by
  * Bus voltage and temperature reading.
  *
  * In dual drives configuration, 4 of them are consumed by
  * Bus voltage and temperature reading for each motor.
  *
  * Defined to 8 here.
  */
#define RCM_MAX_CONV  8 

/* Global variables ----------------------------------------------------------*/

RegConv_t * RCM_handle_array [RCM_MAX_CONV];
RCM_status_t ConversionStatus = stop;
uint16_t RCM_ReadValue = 0;

/* Private functions ---------------------------------------------------------*/

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
  * @note   Users who do not want a callback to be executed at the end of the conversion,
  *         should use RCM_RegisterRegConv() instead.
  *
  * @param  regConv Pointer to the regular conversion parameters.
  *         Contains ADC, Channel and sampling time to be used.
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
 * This function is used to read the result of a regular conversion.
 * Depending of the MC state machine, this function can poll on the ADC end of conversion or not.
 * If the ADC is already in use for currents sensing, the regular conversion can not
 * be executed instantaneously but have to be scheduled in order to be executed after currents sensing
 * inside HF task.
 * This function takes care of inserting the handle into the scheduler.
 * If it is possible to execute the conversion instantaneously, it will be executed, and result returned.
 * Otherwise, the latest stored conversion result will be returned.
 *
 * NOTE: This function is not part of the public API and users should not call it.
 */
void RCM_ExecuteRegularConv(ScanGroup group)
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

/**
 * @brief Schedules a regular conversion for execution.
 *
 * This function requests the execution of the user-defined regular conversion identified
 * by @p handle. All user defined conversion requests must be performed inside routines with the
 * same priority level. If a previous regular conversion request is pending this function has no
 * effect, for this reason is better to call RCM_GetUserConvState() and check if the state is
 * #RCM_USERCONV_IDLE before calling RCM_RequestUserConv().
 *
 * @param  handle used for the user conversion.
 *
 * @return true if the regular conversion could be scheduled and false otherwise.
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
 * @brief Schedules a regular conversion for execution.
 *
 * This function requests the execution of the user-defined regular conversion identified
 * by @p handle. All user defined conversion requests must be performed inside routines with the
 * same priority level. If a previous regular conversion request is pending this function has no
 * effect, for this reason is better to call RCM_GetUserConvState() and check if the state is
 * #RCM_USERCONV_IDLE before calling RCM_RequestUserConv().
 *
 * @param  handle used for the user conversion.
 *
 * @return true if the regular conversion could be scheduled and false otherwise.
 */
void RCM_EnableConv(void)
{
    if(ConversionStatus == stop)
    {
        ConversionStatus = start;
    }    
}

/**
 * @brief Schedules a regular conversion for execution.
 *
 * This function requests the execution of the user-defined regular conversion identified
 * by @p handle. All user defined conversion requests must be performed inside routines with the
 * same priority level. If a previous regular conversion request is pending this function has no
 * effect, for this reason is better to call RCM_GetUserConvState() and check if the state is
 * #RCM_USERCONV_IDLE before calling RCM_RequestUserConv().
 *
 * @param  handle used for the user conversion.
 *
 * @return true if the regular conversion could be scheduled and false otherwise.
 */
void RCM_DisableConv(void)
{
    if(ConversionStatus == dataavailable || ConversionStatus == ongoing )
    {
        ConversionStatus = stop;
    }
}
