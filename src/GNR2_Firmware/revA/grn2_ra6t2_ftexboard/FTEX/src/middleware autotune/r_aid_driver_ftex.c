/***********************************************************************************************************************
 * File Name    : r_aid_driver_ftex.c
 * Author				: Sami Bouzid
 * Description  : Implementation of interface defined in r_aid_driver_if.h for FTEX motor controller. 
									Uses FTEX motor control submodules. MOTOR CONTROL APPLICATION MUST HAVE BEEN INITIALIZED PRIOR 
									TO USING BELOW FUNCTIONS, USING MC_BootUp().
 **********************************************************************************************************************/
/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdint.h>
#include "r_aid_config.h"
#include "r_aid_driver_if.h"
#include "mc_config.h"
#include "autotune.h"

// Max and min duty cycle are not directly defined yet for our inverter. Temporally defined here to set a limit to the tuner.
#define INVERTER_MAX_DUTY 0.97f
#define INVERTER_MIN_DUTY 0.03f

float g_f4_iu; // Current value of phase U.
float g_f4_iv; // Current value of phase V.
float g_f4_iw; // Current value of phase W.

/*******************************************************************************************************************//**
 * @brief Enable PWM output
 * @note  If there are any HW prohibiting the PWM output, it should also be cleared
 **********************************************************************************************************************/
void aid_hal_pwm_enable(void)
{
    PWMCurrFdbkHandle_t * pPWMCurrFdbk = &PWMInsulCurrSensorFdbkHandleM1.Super;
    PWMCurrFdbk_SwitchOnPWM(pPWMCurrFdbk);
} /* End of function aid_hal_pwm_enable */

/*******************************************************************************************************************//**
 * @brief Disable the PWM output
 **********************************************************************************************************************/
void aid_hal_pwm_disable(void)
{
    PWMCurrFdbkHandle_t * pPWMCurrFdbk = &PWMInsulCurrSensorFdbkHandleM1.Super;
    PWMCurrFdbk_SwitchOffPWM(pPWMCurrFdbk);
} /* End of function aid_hal_pwm_disable */

/*******************************************************************************************************************//**
 * @brief  Gets the voltage of DC bus
 * @return The voltage [V] of DC bus
 **********************************************************************************************************************/
float aid_hal_get_vdc(void)
{
    float fVbus = 0;
    BusVoltageSensorHandle_t * pVoltageSensor = &RealBusVoltageSensorParamsM1.Super;
    fVbus = (float)VbusSensor_GetAvBusVoltageVolt(pVoltageSensor);
    return fVbus;
} /* End of function aid_hal_get_vdc */

/*******************************************************************************************************************//**
 * @brief      Gets the information of inverter
 * @param[out] st_inv_info  Pointer to the structure of inverter information, every member of the structure should be
 *                          set with appropriate value. See @ref st_aid_inv_info_t.
 **********************************************************************************************************************/
void aid_hal_get_inv_info(st_aid_inv_info_t* st_inv_info)
{
    PWMCurrFdbkHandle_t * pPWMCurrFdbk = &PWMInsulCurrSensorFdbkHandleM1.Super;
    ResDivVbusSensorHandle_t * pVoltageSensorResDiv = &RealBusVoltageSensorParamsM1;
    BusVoltageSensorHandle_t * pVoltageSensor = &RealBusVoltageSensorParamsM1.Super;

    float fOverVoltageTh = (float)pVoltageSensorResDiv->hOverVoltageThreshold * (float)pVoltageSensor->hConversionFactor / 65536.0f;
    float fUnderVoltageTh = (float)pVoltageSensorResDiv->hUnderVoltageThreshold * (float)pVoltageSensor->hConversionFactor / 65536.0f;
    float fOverCurrentTh = (float)pPWMCurrFdbk->hSoftwareOCPMaximumCurrent * pPWMCurrFdbk->fCurrentConversionFactor;
    float fPWMPeriodSeconds = (float)pPWMCurrFdbk->hPWMperiod * pPWMCurrFdbk->fCycle2SecondConversionFactor;
    float fPWMDeadtimeSeconds = (float)pPWMCurrFdbk->hPWMDeadtime * pPWMCurrFdbk->fCycle2SecondConversionFactor;

    st_inv_info->duty_min           = INVERTER_MIN_DUTY;
    st_inv_info->duty_max           = INVERTER_MAX_DUTY;
    st_inv_info->overcurrent_th     = fOverCurrentTh;
    st_inv_info->overvoltage_th     = fOverVoltageTh;
    st_inv_info->undervoltage_th    = fUnderVoltageTh;
    st_inv_info->pwm_cycle_s        = fPWMPeriodSeconds;
    st_inv_info->pwm_deadtime_s     = fPWMDeadtimeSeconds;
    st_inv_info->pwm_lsb            = 2.0f / (float)pPWMCurrFdbk->hPWMperiod;
    st_inv_info->current_lsb        = 65536.0f * pPWMCurrFdbk->fCurrentConversionFactor / 4096.0f;  /* Full current range / ADC max digits */
} /* End of function aid_hal_get_inv_info */

/*******************************************************************************************************************//**
 * @brief Gets the current of each phase
 *
 * @param[out] f4_ia  Pointer to the variable to store the phase A current
 * @param[out] f4_ib  Pointer to the variable to store the phase B current
 * @param[out] f4_ic  Pointer to the variable to store the phase C current
 **********************************************************************************************************************/
void aid_hal_get_current_abc(float *f4_ia, float *f4_ib, float *f4_ic)
{
    PWMCurrFdbkHandle_t * pPWMCurrFdbk = &PWMInsulCurrSensorFdbkHandleM1.Super;
    
    ab_t Iab;
    Autotune_GetFiltPhaseCurrents(pPWMCurrFdbk, &Iab);
    
    // Added inversion for compatibility. To be investigated. 
    g_f4_iu = (float)Iab.a * (-pPWMCurrFdbk->fCurrentConversionFactor);
    g_f4_iv = (float)Iab.b * (-pPWMCurrFdbk->fCurrentConversionFactor);
    g_f4_iw =  -g_f4_iu - g_f4_iv;
    *f4_ia = g_f4_iu;
    *f4_ib = g_f4_iv;
    *f4_ic = g_f4_iw;
} /* End of function aid_hal_get_current_abc */

/*******************************************************************************************************************//**
 * @brief Gets the current of phases a and b.
 *
 * @param[out] f4_ia  Pointer to the variable to store the phase A current
 * @param[out] f4_ib  Pointer to the variable to store the phase B current
 **********************************************************************************************************************/
void aid_hal_get_current_ab(float *f4_ia, float *f4_ib)
{
    PWMCurrFdbkHandle_t * pPWMCurrFdbk = &PWMInsulCurrSensorFdbkHandleM1.Super;
    
    ab_t Iab;
    Autotune_GetPhaseCurrents(pPWMCurrFdbk, &Iab);
    
    // Added inversion for compatibility. To be investigated. 
    g_f4_iu = (float)Iab.a * (-pPWMCurrFdbk->fCurrentConversionFactor);
    g_f4_iv = (float)Iab.b * (-pPWMCurrFdbk->fCurrentConversionFactor);
    g_f4_iw =  -g_f4_iu - g_f4_iv;
    *f4_ia = g_f4_iu;
    *f4_ib = g_f4_iv;
} /* End of function aid_hal_get_current_ab */

/*******************************************************************************************************************//**
 * @brief Gets the actual duty cycles of PWM output, should be calculated with count registers of PWM timer
 *
 * @param[out] f4_duty_a  The duty cycle of phase A (0~1)
 * @param[out] f4_duty_b  The duty cycle of phase B (0~1)
 **********************************************************************************************************************/
void aid_hal_pwm_get_duty_abc(float *f4_duty_a, float *f4_duty_b, float *f4_duty_c)
{
    PWMCurrFdbkHandle_t * pPWMCurrFdbk = &PWMInsulCurrSensorFdbkHandleM1.Super;

    float fDutyALtd = 1.0f - (2.0f * (float)pPWMCurrFdbk->hCntPhA / (float)pPWMCurrFdbk->hPWMperiod);
    float fDutyBLtd = 1.0f - (2.0f * (float)pPWMCurrFdbk->hCntPhB / (float)pPWMCurrFdbk->hPWMperiod);
    float fDutyCLtd = 1.0f - (2.0f * (float)pPWMCurrFdbk->hCntPhC / (float)pPWMCurrFdbk->hPWMperiod);

    // Limit to 1.0f
    if (fDutyALtd > 1.0f)
    {
        fDutyALtd = 1.0f;
    }
    if (fDutyBLtd > 1.0f)
    {
        fDutyBLtd = 1.0f;
    }
    if (fDutyCLtd > 1.0f)
    {
        fDutyCLtd = 1.0f;
    }

    // Limit to 0.0f
    if (fDutyALtd < 0.0f)
    {
        fDutyALtd = 0.0f;
    }
    if (fDutyBLtd < 0.0f)
    {
        fDutyBLtd = 0.0f;
    }
    if (fDutyCLtd < 0.0f)
    {
        fDutyCLtd = 0.0f;
    }

    *f4_duty_a = fDutyALtd;
    *f4_duty_b = fDutyBLtd;
    *f4_duty_c = fDutyCLtd;
} /* End of function aid_hal_pwm_get_duty_ab */

/*******************************************************************************************************************//**
 * @brief     Sets the duty cycles of PWM output
 *
 * @param[in] f4_duty_a  The duty cycle of phase A (0~1)
 * @param[in] f4_duty_b  The duty cycle of phase B (0~1)
 * @note      It is better to ignore any invalid input
 **********************************************************************************************************************/
void aid_hal_pwm_set_duty_abc(float f4_duty_a, float f4_duty_b, float f4_duty_c)
{
    PWMCurrFdbkHandle_t * pPWMCurrFdbk = &PWMInsulCurrSensorFdbkHandleM1.Super;

    float fDutyALtd = f4_duty_a;
    float fDutyBLtd = f4_duty_b;
    float fDutyCLtd = f4_duty_c;

    // Limit to INVERTER_MAX_DUTY
    if (fDutyALtd > INVERTER_MAX_DUTY)
    {
        fDutyALtd = INVERTER_MAX_DUTY;
    }
    if (fDutyBLtd > INVERTER_MAX_DUTY)
    {
        fDutyBLtd = INVERTER_MAX_DUTY;
    }
    if (fDutyCLtd > INVERTER_MAX_DUTY)
    {
        fDutyCLtd = INVERTER_MAX_DUTY;
    }

    // Limit to INVERTER_MIN_DUTY
    if (fDutyALtd < INVERTER_MIN_DUTY)
    {
        fDutyALtd = INVERTER_MIN_DUTY;
    }
    if (fDutyBLtd < INVERTER_MIN_DUTY)
    {
        fDutyBLtd = INVERTER_MIN_DUTY;
    }
    if (fDutyCLtd < INVERTER_MIN_DUTY)
    {
        fDutyCLtd = INVERTER_MIN_DUTY;
    }

    Autotune_SetDuties(pPWMCurrFdbk, (uint16_t)(fDutyALtd*UINT16_MAX), (uint16_t)(fDutyBLtd*UINT16_MAX), (uint16_t)(fDutyCLtd*UINT16_MAX));
} /* End of function aid_hal_pwm_set_duty_ab */
