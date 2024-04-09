/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2018 Renesas Electronics Corporation. All rights reserved.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : r_aid_driver_if.h
 * Version      : 0.1
 * Description  : Hardware abstraction interface for Tuner
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/
/*******************************************************************************************************************//**
 * @ingroup Tuner_Interface
 * @defgroup TUNER_DRIVER STM Tuner driver interface
 * @brief This is an abstractive interface for STM tuner
 * @{
 **********************************************************************************************************************/

#ifndef R_AID_DRIVER_IF_H_
    #define R_AID_DRIVER_IF_H_

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
/*******************************************************************************************************************//**
 * Data structure used to store information of inverter and configurations depends on hardware
 **********************************************************************************************************************/
typedef struct
{
    float duty_min;             /**< The minimum duty within (0 ~ 1) */
    float duty_max;             /**< The maximum duty within (@ref duty_min ~ 1) */
    float overcurrent_th;       /**< The over-current threshold */
    float overvoltage_th;       /**< The over-voltage threshold */
    float undervoltage_th;      /**< The under-voltage threshold */
    float current_lsb;          /**< The current that represented by 1 LSB of ADC, which provide the current sensing
                                 *   resolution information to improve accuracy of identification */
    float pwm_lsb;              /**< The duty cycle that is represented by 1 count of PWM timer */
    float pwm_cycle_s;          /**< The PWM period [s] */
    float pwm_deadtime_s;       /**< The dead-time [s] */
} st_aid_inv_info_t;

/***********************************************************************************************************************
 Exported global functions (to be accessed by other files)
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * @brief Enable PWM output
 * @note  If there are any HW prohibiting the PWM output, it should also be cleared
 **********************************************************************************************************************/
void aid_hal_pwm_enable(void);

/*******************************************************************************************************************//**
 * @brief Disable the PWM output
 **********************************************************************************************************************/
void aid_hal_pwm_disable(void);

/*******************************************************************************************************************//**
 * @brief Gets the actual duty cycles of PWM output, should be calculated with count resisters of PWM timer
 *
 * @param[out] f4_duty_a  The duty cycle of phase A (0~1)
 * @param[out] f4_duty_b  The duty cycle of phase B (0~1)
 **********************************************************************************************************************/
void aid_hal_pwm_get_duty_ab(float *f4_duty_a, float *f4_duty_b);

/*******************************************************************************************************************//**
 * @brief     Sets the duty cycles of PWM output
 *
 * @param[in] f4_duty_a  The duty cycle of phase A (0~1)
 * @param[in] f4_duty_b  The duty cycle of phase B (0~1)
 * @note      It is better to ignore any invalid input
 **********************************************************************************************************************/
void aid_hal_pwm_set_duty_ab(float f4_duty_a, float f4_duty_b);

/*******************************************************************************************************************//**
 * @brief  Gets the voltage of DC bus
 * @return The voltage [V] of DC bus
 **********************************************************************************************************************/
float aid_hal_get_vdc(void);

/*******************************************************************************************************************//**
 * @brief Gets the current of each phase
 *
 * @param[out] f4_ia  Pointer to the variable to store the phase A current
 * @param[out] f4_ib  Pointer to the variable to store the phase B current
 **********************************************************************************************************************/
void aid_hal_get_current_ab(float *f4_ia, float *f4_ib);

/*******************************************************************************************************************//**
 * @brief      Gets the information of inverter
 * @param[out] st_inv_info  Pointer to the structure of inverter information, every member of the structure should be
 *                          set with appropriate value. See @ref st_aid_inv_info_t.
 **********************************************************************************************************************/
void aid_hal_get_inv_info(st_aid_inv_info_t* st_inv_info);

/** The following prototype declarations is for 3 phase BLDC
 *
 *  **/
/*******************************************************************************************************************//**
 * @brief Gets the current of each phase
 *
 * @param[out] f4_ia  Pointer to the variable to store the phase A current
 * @param[out] f4_ib  Pointer to the variable to store the phase B current
 * @param[out] f4_ic  Pointer to the variable to store the phase C current
 **********************************************************************************************************************/
void aid_hal_get_current_abc(float *f4_ia, float *f4_ib, float *f4_ic);

/*******************************************************************************************************************//**
 * @brief Gets the actual duty cycles of PWM output, should be calculated with count resisters of PWM timer
 *
 * @param[out] f4_duty_a  The duty cycle of phase A (0~1)
 * @param[out] f4_duty_b  The duty cycle of phase B (0~1)
 * @param[out] f4_duty_c  The duty cycle of phase C (0~1)
 **********************************************************************************************************************/
void aid_hal_pwm_get_duty_abc(float *f4_duty_a, float *f4_duty_b, float *f4_duty_c);

/*******************************************************************************************************************//**
 * @brief     Sets the duty cycles of PWM output
 *
 * @param[in] f4_duty_a  The duty cycle of phase A (0~1)
 * @param[in] f4_duty_b  The duty cycle of phase B (0~1)
 * @param[in] f4_duty_c  The duty cycle of phase C (0~1)
 * @note      It is better to ignore any invalid input
 **********************************************************************************************************************/
void aid_hal_pwm_set_duty_abc(float f4_duty_a, float f4_duty_b, float f4_duty_c);

/** @} */
#endif /* MIDDLE_IDENTIFICATION_R_AID_DRIVER_IF_H_ */
