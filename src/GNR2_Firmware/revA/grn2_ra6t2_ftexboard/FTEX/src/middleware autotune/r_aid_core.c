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
 * File Name    : r_aid_core.c
 * Version      : 1.0
 * Description  : This module solves all the world's problems
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/
/*******************************************************************************************************************//**
 * @ingroup Tuner_Interface
 * @defgroup CORE CORE module
 * @brief CORE module handles calibrations, current/speed control and signals. Also launch events for upper layer
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <math.h>
#include <stdint.h>
//#include <machine.h>
#include "r_aid_core.h"
#include "r_aid_driver_if.h"
#include "r_aid_volterr_comp.h"
#include "r_aid_pi_control.h"
#include "r_aid_filter.h"
#include "r_aid_transform.h"

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#ifndef AID_PRV_FILE_CODE
    #define AID_PRV_FILE_CODE (107)
#endif
#define AID_CORE_PRV_OL2CL_METHOD                (1)
#define AID_CORE_PRV_INV_CURRENT_LIMIT           (5.0f)

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
typedef struct {
    uint32_t u4_warn_core_flags;
} st_aid_warn_t;

/***********************************************************************************************************************
 Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/
uint16_t  aid_u2_ctrl_level;
e_aid_rotor_angle_mode_t aid_s4_rotor_angle_mode;
uint16_t  aid_u2_error_status;
uint16_t  aid_u2_run_mode;
/* Inverter properties */
float aid_f4_current_lsb;
float aid_f4_pwm_duty_lsb;

/* Measurement parameters */
float  aid_f4_r_dc;
float  aid_f4_v_err;
float  aid_f4_r;
float  aid_f4_ld;
float  aid_f4_lq;
float  aid_f4_ke;
float  aid_f4_j;
float  aid_f4_d;

/* Input */
float aid_f4_rated_current;
float aid_f4_rated_power;
float aid_f4_pole_pairs;
float aid_f4_r_max;
float aid_f4_r_min;
float aid_f4_ld_min;
float aid_f4_lq_min;

/* Motor driver */
float aid_f4_duty_available;
float aid_f4_vmag_max;                     /* Maximum magnitude of available voltage */
float aid_f4_imag_max;
float aid_f4_ia_ad;
float aid_f4_ib_ad;
float aid_f4_ic_ad;
float aid_f4_offset_ia;
float aid_f4_offset_ib;
float aid_f4_pre_offset_ia;
float aid_f4_pre_offset_ib;
float aid_f4_offset_lpf_k;
uint16_t  aid_u2_cnt_adjust;
float aid_f4_offset_calc_time;
float aid_f4_vdc_ad;
float aid_f4_id_ad;
float aid_f4_iq_ad;
float aid_f4_angle_rad;
float aid_f4_lim_iq;
float aid_f4_overcurrent_limit_hw;
float aid_f4_overcurrent_limit;
float aid_f4_overvoltage_limit;
float aid_f4_undervoltage_limit;
float aid_f4_id_ref;
float aid_f4_iq_ref;
float aid_f4_e;
float aid_f4_e_lpf;
float aid_f4_ref_id;
float aid_f4_ref_speed_rad;
float aid_f4_limit_speed_change;            /* Limit of speed change rate [(rad/s)/ms] */
float aid_f4_speed_rad;
float aid_f4_speed_lpf_rad;
st_aid_1st_order_lpf_t aid_st_speed_lpf;
float aid_f4_speed_lpf_omega_hz;
float aid_f4_omega_current;
float aid_f4_zeta_current;
float aid_f4_kp_id;
float aid_f4_ki_id;
float aid_f4_kp_iq;
float aid_f4_ki_iq;
float aid_f4_omega_e_obs;
float aid_f4_zeta_e_obs;
float aid_f4_k_e_obs_d_1;
float aid_f4_k_e_obs_d_2;
float aid_f4_k_e_obs_q_1;
float aid_f4_k_e_obs_q_2;
float aid_f4_kp_speed;
float aid_f4_ki_speed;
float aid_f4_id_down_speed_rad;
float aid_f4_id_up_speed_rad;
float aid_f4_id_up_step;
float aid_f4_id_down_step;
float aid_f4_pre_id_ad;
float aid_f4_pre_id_est;
float aid_f4_pre_dd;
float aid_f4_id_est;
float aid_f4_dd;
float aid_f4_ref_speed_rad_ctrl;
float aid_f4_ed;
float aid_f4_pre_iq_ad;
float aid_f4_pre_iq_est;
float aid_f4_pre_dq;
float aid_f4_iq_est;
float aid_f4_iq_est_limit;
float aid_f4_dq;
float aid_f4_dq_limit;
float aid_f4_eq;
float aid_f4_phase_error;
float aid_f4_va_ref;
float aid_f4_vb_ref;
float aid_f4_vc_ref;
float aid_f4_va_ref_pwm;
float aid_f4_vb_ref_pwm;
float aid_f4_vc_ref_pwm;
float aid_f4_va_ref_comp;
float aid_f4_vb_ref_comp;
float aid_f4_vc_ref_comp;
float aid_f4_vphase_limit;
float aid_f4_moda;
float aid_f4_modb;
float aid_f4_1ms_speed_rad;
uint16_t  aid_u2_ctrl_conf;
uint8_t aid_u1_flag_down_to_ol;
uint8_t aid_u1_flag_iq_ref;
uint8_t aid_u1_flag_id_ref;
uint8_t aid_u1_flag_speed_ref;
float aid_f4_iq_down_step;
float aid_f4_ol_iq_down_step;
float aid_f4_kp_est_speed;
float aid_f4_ki_est_speed;
float aid_f4_i_est_speed;
float aid_f4_ia_ref;
float aid_f4_ib_ref;
float aid_f4_ic_ref;
float aid_f4_vcomp_array[3];
float aid_f4_vcomp_i_array[3];
float aid_f4_temp_cos;
float aid_f4_temp_sin;
st_aid_pi_ctrl_t st_aid_vd;
st_aid_pi_ctrl_t st_aid_vq;
st_aid_pi_ctrl_t st_aid_speed;
st_aid_volterr_comp_t st_aid_volt_comp_p;
float  aid_f4_phase_error_lpf;
float  aid_f4_phase_error_hpf;
float  aid_f4_bemf_a_lpf;
float  aid_f4_bemf_a_lpf_prev;
float  aid_f4_torque_current;

#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    float aid_f4_offset_ic;
    float aid_f4_pre_offset_ic;
    float aid_f4_modc;
#endif

/* Common */
uint32_t  aid_u4_sample_cnt;
uint16_t  aid_u2_1ms_cnt;
uint16_t  aid_u2_sum_cnt;
float aid_f4_f_ref;
float aid_f4_vd_ref;
float aid_f4_vq_ref;
aid_rotor_angle_t aid_st_rotor_angle;

float aid_f4_pwm_period_ms;
float aid_f4_ctrl_period_ms;
float aid_f4_spd_ctrl_period_ms;
float aid_f4_ctrl_freq_hz;
float aid_f4_spd_ctrl_freq_hz;

uint32_t dbg_u4_line;
uint32_t dbg_u4_file;
float dbg_f4_ld_bemf_observer;

float kp_id_final, ki_id_final, kp_iq_final, ki_iq_final;

/* Warning message */
st_aid_warn_t aid_st_warn;

/* Inverter info */
st_aid_inv_info_t aid_st_inv_info;


/***********************************************************************************************************************
 Private global variables and functions
 **********************************************************************************************************************/
static void core_dummy_delegate(void);

core_delegate_t gs_before_current_ctrl = core_dummy_delegate;
core_delegate_t gs_after_pwm_output    = core_dummy_delegate;

/*******************************************************************************************************************//**
 * @brief Dummy function for avoiding null function pointer
 **********************************************************************************************************************/
static void core_dummy_delegate(void)
{
    /* Do nothing */
    __asm __volatile("nop\n");
} /* End of function core_dummy_delegate */

/*******************************************************************************************************************//**
 * @brief  Generates speed command
 * @return Speed command value [rad/s]
 **********************************************************************************************************************/
static float core_generate_speed_ref(void)
{
    float f4_temp0;
    float f4_temp1;
    float f4_speed_rad_ref_buff=0;

    switch (aid_u1_flag_speed_ref)
    {
        case AID_SPEED_ZERO_CONST:
        {
            //f4_speed_rad_ref_buff = 0.0f;
            f4_speed_rad_ref_buff = aid_f4_ref_speed_rad_ctrl;
            if (AID_ID_CONST == aid_u1_flag_id_ref)
            {
                aid_u1_flag_speed_ref = AID_SPEED_CHANGE;
            }
        }
        break;

        case AID_SPEED_CHANGE:
        {
            f4_temp0 = aid_f4_ref_speed_rad - aid_f4_ref_speed_rad_ctrl;
            f4_temp1 = fminf(aid_f4_limit_speed_change * aid_f4_spd_ctrl_period_ms, fabsf(f4_temp0));
            f4_speed_rad_ref_buff = aid_f4_ref_speed_rad_ctrl + copysignf(f4_temp1, f4_temp0);
        }
        break;

        default:
        {
            AID_ASSERT_FAIL();
        }
        break;
    }

    /* return speed reference */
    return(f4_speed_rad_ref_buff);
} /* End of function core_generate_speed_ref */

/*******************************************************************************************************************//**
 * @brief Generate Iq reference
 * @return Iq reference [A]
 **********************************************************************************************************************/
static float core_generate_iq_ref(void)
{
    float f4_temp0;
    float f4_iq_ref_buff=0;

    if (AID_FLG_SET == aid_u1_flag_down_to_ol)
    {
        aid_u1_flag_iq_ref = AID_IQ_DOWN;
        aid_s4_rotor_angle_mode = AID_ROTOR_ANGLE_MODE_OPENLOOP;
        aid_f4_iq_down_step = aid_f4_ol_iq_down_step * aid_f4_iq_ref;
    }

    switch (aid_u1_flag_iq_ref)
    {
        case AID_IQ_ZERO_CONST:
        {
            aid_s4_rotor_angle_mode = AID_ROTOR_ANGLE_MODE_OPENLOOP;
            f4_iq_ref_buff = 0.0f;
            f4_temp0 = fabsf(aid_f4_1ms_speed_rad);
#if (AID_CORE_PRV_OL2CL_METHOD == 1)
            /* If the speed is sufficient to estimate phase error, refresh the maximum value of torque current */
            if(f4_temp0 > aid_f4_id_down_speed_rad * 0.75f)
            {
                /* Simplified, Iq is always zero at this state */
                float torque_current_temp = sinf(aid_f4_phase_error_lpf) * aid_f4_id_ref;

                /* use maximum value to ensure the estimated torque current value is high enough
                 *  for performing OL to CL transition */
                aid_f4_torque_current = (torque_current_temp > aid_f4_torque_current) ?
                                        (torque_current_temp) : (aid_f4_torque_current);
            }
#endif
            if (f4_temp0 >= aid_f4_id_down_speed_rad)
            {
#if (AID_CORE_PRV_OL2CL_METHOD == 1)
                aid_u1_flag_iq_ref = AID_IQ_CL_TRANS;
#elif (AID_CORE_PRV_OL2CL_METHOD == 0)
                aid_u1_flag_iq_ref = AID_IQ_SPEED_PI_OUTPUT;
                aid_s4_rotor_angle_mode = AID_ROTOR_ANGLE_MODE_CLOSEDLOOP;
                aid_f4_i_est_speed = aid_f4_ref_speed_rad_ctrl;
#endif
            }
        }
        break;

        case AID_IQ_CL_TRANS:
        {
            /* Increase Iq to eliminate the phase error before transit to the closed-loop FOC operation */
            f4_iq_ref_buff = aid_f4_iq_ref;
            f4_iq_ref_buff += (aid_f4_torque_current * (aid_f4_spd_ctrl_period_ms / g_f4_aid_id_up_time) *
                              (aid_f4_phase_error_hpf >= (-g_f4_aid_ol2cl_inciq_phaseerr_dec_th))) * g_f4_aid_ol2cl_crnt_inc_mult;

            /* Limit Iq reference */
            f4_iq_ref_buff = R_AID_LimitfAbs(f4_iq_ref_buff, aid_f4_lim_iq);
            if (f4_iq_ref_buff < 0.0f)
            {
                f4_iq_ref_buff = 0.0f;
            }

            if ((AID_ID_ZERO_CONST == aid_u1_flag_id_ref) ||              /* Id became zero, the ol2cl phase ends */
                (aid_f4_id_ref <= aid_f4_torque_current) ||               /* Leave enough Id to prevent motor stall */
                (aid_f4_phase_error_lpf < 0.0f) ||                        /* The phase error has been eliminated */
                (aid_f4_phase_error_hpf < (-g_f4_aid_ol2cl_phaseerr_max_dec))) /* The phase error decreases too fast */
            {
                /* Set the current speed to the PLL integrator to ensure smooth transition */
                aid_f4_ref_speed_rad_ctrl = aid_f4_speed_lpf_rad;
                aid_f4_i_est_speed = aid_f4_ref_speed_rad_ctrl;

                /* Set the current Iq to the speed PI integrator to ensure smooth transition */
                st_aid_speed.f4_refi = f4_iq_ref_buff;

                /* Enable normal FOC operation */
                aid_u1_flag_iq_ref = AID_IQ_SPEED_PI_OUTPUT;
                aid_s4_rotor_angle_mode = AID_ROTOR_ANGLE_MODE_CLOSEDLOOP;
            }
        }
        break;

        case AID_IQ_SPEED_PI_OUTPUT:
        {
            /*** speed PI control ***/
            f4_iq_ref_buff = aid_pi_ctrl(&st_aid_speed, aid_f4_ref_speed_rad_ctrl - aid_f4_1ms_speed_rad);

            /*** iq reference limit ***/
            f4_iq_ref_buff = R_AID_LimitfAbs(f4_iq_ref_buff, aid_f4_lim_iq);
        }
        break;

        case AID_IQ_DOWN:
        {
            f4_iq_ref_buff = aid_f4_iq_ref - aid_f4_iq_down_step;
            f4_temp0 = aid_f4_iq_down_step * f4_iq_ref_buff;                 /* check sign */
            if (f4_temp0 <= 0)
            {
                f4_iq_ref_buff = 0.0f;
                aid_u1_flag_iq_ref = AID_IQ_ZERO_CONST;
            }
        }
        break;

        default:
        {
            /* Do Nothing */
            break;
        }
    }

    /* return iq reference */
    return(f4_iq_ref_buff);
} /* End of function core_generate_iq_ref */

/*******************************************************************************************************************//**
 * @brief  Generate Id reference
 * @return Id reference [A]
 **********************************************************************************************************************/
static float core_generate_id_ref(void)
{
    float f4_temp0;
    float f4_id_ref_buff=0;

    if (AID_FLG_SET == aid_u1_flag_down_to_ol)
    {
        aid_u1_flag_id_ref = AID_ID_UP;
    }
    switch (aid_u1_flag_id_ref)
    {
        case AID_ID_UP:
        {
            f4_id_ref_buff = aid_f4_id_ref + aid_f4_id_up_step;
            if (f4_id_ref_buff >= aid_f4_ref_id)
            {
                f4_id_ref_buff = aid_f4_ref_id;
                aid_u1_flag_id_ref = AID_ID_CONST;
            }
        }
        break;

        case AID_ID_CONST:
        {
            f4_id_ref_buff = aid_f4_id_ref;
            if ((uint8_t)aid_f4_ref_id != (uint8_t)f4_id_ref_buff)
            {
                f4_id_ref_buff = aid_f4_ref_id;
            }

            f4_temp0 = fabsf(aid_f4_ref_speed_rad_ctrl);
            if (f4_temp0 >= aid_f4_id_down_speed_rad)
            {
                aid_u1_flag_id_ref = AID_ID_DOWN;
            }
        }
        break;

        case AID_ID_DOWN:
        {
            f4_id_ref_buff = aid_f4_id_ref - aid_f4_id_down_step;
            if (f4_id_ref_buff < 0.0f)
            {
                f4_id_ref_buff = 0.0f;
                aid_u1_flag_id_ref = AID_ID_ZERO_CONST;
            }
        }
        break;

        case AID_ID_ZERO_CONST:
        {
            f4_id_ref_buff = 0.0f;
        }
        break;

        default:
        {
            AID_ASSERT_FAIL();
        }
        break;
    }

    /* return id reference */
    return(f4_id_ref_buff);
} /* End of function core_generate_id_ref */

/******************************************************************************
* Function Name : core_check_over_current_error
* Description   : Over current error check
* Arguments     : Iu, Iv, Iw, over current limit value
* Return Value  : error status
******************************************************************************/
static void core_check_over_current_error(float f4_iu, float f4_iv, float f4_iw, float f4_overcurrent_limit)
{
    float f4_temp0;

    f4_temp0 = fabsf(f4_iu);
    if (f4_temp0 > f4_overcurrent_limit)
    {
        aid_core_throw_error(AID_ERROR_OVER_CURRENT_SW);
    }

    f4_temp0 = fabsf(f4_iv);
    if (f4_temp0 > f4_overcurrent_limit)
    {
        aid_core_throw_error(AID_ERROR_OVER_CURRENT_SW);
    }

    f4_temp0 = fabsf(f4_iw);
    if (f4_temp0 > f4_overcurrent_limit)
    {
        aid_core_throw_error(AID_ERROR_OVER_CURRENT_SW);
    }
} /* End of function core_check_over_current_error */

/******************************************************************************
* Function Name : core_check_over_voltage_error
* Description   : Over voltage error check
* Arguments     : Vdc value, over voltage limit value
* Return Value  : error status
******************************************************************************/
static void core_check_over_voltage_error(float f4_vdc, float f4_overvoltage_limit)
{
    if (f4_vdc > f4_overvoltage_limit)
    {
        aid_core_throw_error(AID_ERROR_OVER_VOLTAGE);              /* over voltage error */
    }
} /* End of function core_check_over_voltage_error */

/******************************************************************************
* Function Name : core_check_under_voltage_error
* Description   : Under voltage error check
* Arguments     : Vdc value, under voltage limit value
* Return Value  : error status
******************************************************************************/
static void core_check_under_voltage_error(float f4_vdc, float f4_undervoltage_limit)
{
    if (f4_vdc < f4_undervoltage_limit)
    {
        aid_core_throw_error(AID_ERROR_UNDER_VOLTAGE);             /* under voltage error */
    }
} /* End of function core_check_under_voltage_error */

/*******************************************************************************************************************//**
 * Initializes Ke (BEMF constant) identification module
 **********************************************************************************************************************/
static void core_angle_speed_est(void)
{
    float f4_temp0;
    float f4_temp1;
    static float pll_rampup;

    if ((AID_ROTOR_ANGLE_MODE_CLOSEDLOOP) == aid_s4_rotor_angle_mode)
    {
        /* Estimate angle and speed with PLL */
        pll_rampup = (pll_rampup > 1.0f) ?  1.0f : (pll_rampup + 0.01f);
        f4_temp0 = (-aid_f4_phase_error) * pll_rampup;

        /* Speed calculation, speed is estimated by PLL */
        f4_temp1 = f4_temp0 * aid_f4_kp_est_speed;
        aid_f4_i_est_speed += (f4_temp0 * aid_f4_ki_est_speed);
        aid_f4_speed_rad = f4_temp1 + aid_f4_i_est_speed;
    }
    else if ((AID_ROTOR_ANGLE_MODE_OPENLOOP) == aid_s4_rotor_angle_mode)
    {
        /* speed setting */
        aid_f4_speed_rad = aid_f4_ref_speed_rad_ctrl;
        pll_rampup = 1.00f;
    }
    else
    {
        /* Do nothing */
        __asm __volatile("nop\n");
    }
    aid_f4_speed_lpf_rad = R_AID_FirstOrderLpff(&aid_st_speed_lpf, aid_f4_speed_rad);
    aid_f4_angle_rad += (aid_f4_speed_rad * aid_f4_ctrl_period_ms * 0.001f);

    f4_temp0 = fabsf(aid_f4_angle_rad);
    if (f4_temp0 > AID_TWOPI)
    {
        f4_temp0 = f4_temp0 - AID_TWOPI;
        aid_f4_angle_rad = copysignf(f4_temp0, aid_f4_angle_rad);
    }
} /* End of function core_angle_speed_est */

/*******************************************************************************************************************//**
 * Estimates BEMF vector, calculates amplitude and angle of BEMF
 **********************************************************************************************************************/
static void core_bemf_observer(void)
{
    float f4_temp0;
    float f4_temp1;
    float f4_temp2;
    float f4_temp3;
    float f4_ld;

#if (AID_LD_USE_LOOKUP_TABLE == 1)
    f4_ld = aid_lookuptab_intpltonly(&st_ld_table, aid_f4_id_ad);
#else
    f4_ld = aid_f4_ld;
#endif
    dbg_f4_ld_bemf_observer = f4_ld;

    /*************** d-axis ***************/
    aid_f4_pre_id_est = aid_f4_id_est;
    f4_temp0 = (aid_f4_pre_id_ad - aid_f4_pre_id_est) * f4_ld * aid_f4_k_e_obs_d_1;
    f4_temp1 = aid_f4_pre_id_est * aid_f4_r;
    f4_temp2 = (f4_temp0 - f4_temp1) + aid_f4_vd_ref + aid_f4_pre_dd;
    f4_temp3 = f4_temp2 / f4_ld;
    aid_f4_id_est = aid_f4_pre_id_est + (aid_f4_ctrl_period_ms * 0.001f * f4_temp3);

    aid_f4_pre_dd = aid_f4_dd;
    f4_temp0 = aid_f4_id_ad - aid_f4_id_est;
    f4_temp1 = f4_temp0 * aid_f4_k_e_obs_d_2;
    aid_f4_dd = aid_f4_pre_dd + ((aid_f4_ctrl_period_ms * 0.001f) * f4_temp1);

    f4_temp0 = aid_f4_speed_rad * aid_f4_lq * aid_f4_iq_ad;
    aid_f4_ed = (- aid_f4_dd) + f4_temp0;


    /*************** q-axis ***************/
    aid_f4_pre_iq_est = aid_f4_iq_est;
    f4_temp0 = (aid_f4_pre_iq_ad - aid_f4_pre_iq_est) * aid_f4_lq * aid_f4_k_e_obs_q_1;
    f4_temp1 = aid_f4_pre_iq_est * aid_f4_r;
    f4_temp2 = (f4_temp0 + aid_f4_vq_ref + aid_f4_pre_dq) - f4_temp1;
    f4_temp3 = f4_temp2 / aid_f4_lq;
    aid_f4_iq_est = aid_f4_pre_iq_est + (aid_f4_ctrl_period_ms * 0.001f * f4_temp3);

    aid_f4_pre_dq = aid_f4_dq;
    f4_temp0 = aid_f4_iq_ad - aid_f4_iq_est;
    f4_temp1 = f4_temp0 * aid_f4_k_e_obs_q_2;
    aid_f4_dq = aid_f4_pre_dq + (aid_f4_ctrl_period_ms * 0.001f * f4_temp1);

    f4_temp0 = aid_f4_speed_rad * f4_ld * aid_f4_id_ad;
    aid_f4_eq = (- aid_f4_dq) - f4_temp0;

    /* save current */
    aid_f4_pre_id_ad = aid_f4_id_ad;
    aid_f4_pre_iq_ad = aid_f4_iq_ad;

    /* BEMF estimate */
    aid_f4_e = sqrtf((aid_f4_ed * aid_f4_ed) + (aid_f4_eq * aid_f4_eq));
    if (aid_f4_eq < 0)
    {
        aid_f4_e = -aid_f4_e;
    }

    /* delta angle estimate */
    aid_f4_phase_error = atanf(aid_f4_ed / aid_f4_eq);
} /* End of function core_bemf_observer */

/*******************************************************************************************************************//**
 * @brief  Measures current offset
 * @retval 0 Not completed yet
 * @retval 1 Completed
 **********************************************************************************************************************/
static uint32_t core_offset_measure(void)
{
    /* current offset detection */
    aid_f4_offset_ia = R_AID_Lpff(aid_f4_ia_ad, aid_f4_pre_offset_ia, aid_f4_offset_lpf_k);
    aid_f4_pre_offset_ia = aid_f4_offset_ia;
    aid_f4_offset_ib = R_AID_Lpff(aid_f4_ib_ad, aid_f4_pre_offset_ib, aid_f4_offset_lpf_k);
    aid_f4_pre_offset_ib = aid_f4_offset_ib;
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    aid_f4_offset_ic = R_AID_Lpff(aid_f4_ic_ad, aid_f4_pre_offset_ic, aid_f4_offset_lpf_k);
    aid_f4_pre_offset_ic = aid_f4_offset_ic;
#endif
    aid_u2_cnt_adjust++;
    return (aid_u2_cnt_adjust >= aid_f4_offset_calc_time);
} /* End of function core_offset_measure */

/*******************************************************************************************************************//**
 * @brief     Set-up voltage limiters with given phase voltage limit
 * @param[in] f4_voltage_limit  The phase voltage limit
 **********************************************************************************************************************/
static void core_set_voltage_limit(float f4_voltage_limit)
{
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    aid_f4_vmag_max = f4_voltage_limit * (AID_SQRT_3 / AID_SQRT_2);
#else
    aid_f4_vmag_max = f4_voltage_limit;
#endif

    aid_f4_vphase_limit = f4_voltage_limit;

    /* Set integral limit for current regulators */
    st_aid_vd.f4_ilimit = f4_voltage_limit;
    st_aid_vq.f4_ilimit = f4_voltage_limit;
} /* End of function core_set_voltage_limit */

/*******************************************************************************************************************//**
 * @brief Restart motor drive without change configurations and controller gains
 * @pre   aid_u2_run_mode is under AID_RUNMODE_INIT condition
 **********************************************************************************************************************/
static void core_restart_drive(void)
{
    if (aid_u2_run_mode == AID_RUNMODE_INIT)
    {
        /* Common */
        aid_u4_sample_cnt = 0;
        aid_u2_1ms_cnt    = 0;
        aid_u2_sum_cnt    = 0;

        /* Reset modes */
        aid_s4_rotor_angle_mode = AID_ROTOR_ANGLE_MODE_OPENLOOP;
        aid_core_set_ctrl_level(AID_CTRL_LEVEL_0);
        aid_u2_error_status    = AID_ERROR_NONE;
        aid_u1_flag_id_ref     = AID_ID_UP;
        aid_u1_flag_iq_ref     = AID_IQ_ZERO_CONST;
        aid_u1_flag_speed_ref  = AID_SPEED_ZERO_CONST;
        aid_u1_flag_down_to_ol = 0;
        aid_u2_ctrl_conf = 0;

        /* Motor driver */
        aid_f4_ia_ad  = 0.0f;
        aid_f4_ib_ad  = 0.0f;
        aid_f4_vd_ref = 0.0f;
        aid_f4_vq_ref = 0.0f;
        aid_f4_ia_ref = 0.0f;
        aid_f4_ib_ref = 0.0f;
        aid_f4_id_ref = 0.0f;
        aid_f4_iq_ref = 0.0f;
        aid_f4_id_ad  = 0.0f;
        aid_f4_iq_ad  = 0.0f;
        aid_f4_vdc_ad = 0.0f;

        aid_f4_imag_max = 1.0f;         /* This value will be overrided when setting rated current */

        /* Reset offset calibration */
        aid_f4_offset_ia = 0.0f;
        aid_f4_offset_ib = 0.0f;
        aid_f4_pre_offset_ia = 0.0f;
        aid_f4_pre_offset_ib = 0.0f;
        aid_u2_cnt_adjust = 0;

        /* Reset PLL */
        aid_f4_phase_error = 0.0f;
        aid_f4_speed_rad = 0.0f;
        aid_f4_angle_rad = 0.0f;
        aid_f4_i_est_speed = 0.0f;
        aid_f4_temp_cos = 1.0f;     /* Cos(0) */
        aid_f4_temp_sin = 0.0f;

        /* Reset current control */
        aid_f4_kp_id = 0.0f;
        aid_f4_ki_id = 0.0f;
        aid_f4_kp_iq = 0.0f;
        aid_f4_ki_iq = 0.0f;
        st_aid_vd.f4_refi = 0.0f;
        st_aid_vq.f4_refi = 0.0f;

        /* Reset BEMF observer */
        aid_f4_pre_id_ad = 0.0f;
        aid_f4_pre_id_est = 0.0f;
        aid_f4_pre_dd = 0.0f;
        aid_f4_id_est = 0.0f;
        aid_f4_dd = 0.0f;
        aid_f4_pre_iq_ad = 0.0f;
        aid_f4_pre_iq_est = 0.0f;
        aid_f4_pre_dq = 0.0f;
        aid_f4_iq_est = 0.0f;
        aid_f4_dq = 0.0f;
        aid_f4_ed = 0.0f;
        aid_f4_eq = 0.0f;
        aid_f4_e = 0.0f;
        aid_f4_e_lpf = 0.0f;

        /* Reset speed control, speed change rate limiter */
        aid_f4_ref_speed_rad_ctrl = 0.0f;
        aid_f4_ref_speed_rad = 0.0f;
        st_aid_speed.f4_refi = 0.0f;

        /* Reset start-up sequence parameters */
        aid_f4_id_down_speed_rad = 0.0f;
        aid_f4_id_up_speed_rad = 0.0f;
        aid_f4_id_up_step = 0.0f;
        aid_f4_id_down_step = 0.0f;
        aid_f4_ref_id = 0.0f;
        aid_f4_iq_down_step = 0.0f;

        /* Resets modulation */
        aid_f4_va_ref = 0.0f;
        aid_f4_vb_ref = 0.0f;
        aid_f4_va_ref_comp = 0.0f;
        aid_f4_vb_ref_comp = 0.0f;
        aid_f4_vphase_limit = 0.0f;
        aid_f4_moda = 0.0f;
        aid_f4_modb = 0.0f;

        /* Reset voltage error compensation */
        aid_f4_vcomp_array[0] = 0.0f;
        aid_f4_vcomp_array[1] = 0.0f;
        aid_f4_vcomp_i_array[0] = 0.0f;
        aid_f4_vcomp_i_array[1] = 0.0f;

        /* Reset speed LPF */
        aid_f4_speed_lpf_rad = 0.0f;
        aid_f4_1ms_speed_rad = 0.0f;
        aid_f4_speed_lpf_omega_hz = 125.0f;
        R_AID_FirstOrderLpffReset(&aid_st_speed_lpf);

        /* Reset flags */
        aid_st_warn.u4_warn_core_flags = 0;

        aid_hal_pwm_enable();
    }
    else
    {
        AID_ASSERT_FAIL();
    }
} /* End of function core_restart_drive */

/*******************************************************************************************************************//**
 * @brief     Performs multiple filter process on estimated signals
 *
 * @param[in] phase_error_rad  The phase error in [rad]
 * @param[in] est_bemf         The estimated magnitude of BEMF [V]
 **********************************************************************************************************************/
static void core_estimator_filter(float phase_error_rad, float est_bemf)
{
    float f4_phase_error_lpf_pre;
    float f4_lpf_num = ((AID_TWOPI * g_f4_aid_phaseerr_lpf_band_hz) * (aid_f4_ctrl_period_ms * 0.001f));
    float f4_lpf_den = 1.0f - ((AID_TWOPI * g_f4_aid_phaseerr_lpf_band_hz) * (aid_f4_ctrl_period_ms * 0.001f));

    aid_f4_bemf_a_lpf = (aid_f4_bemf_a_lpf * f4_lpf_den) + (est_bemf * f4_lpf_num);
    f4_phase_error_lpf_pre = aid_f4_phase_error_lpf;
    aid_f4_phase_error_lpf = (aid_f4_phase_error_lpf * f4_lpf_den) + (phase_error_rad * f4_lpf_num);
    aid_f4_phase_error_hpf = (aid_f4_phase_error_hpf * f4_lpf_den) +
            ((aid_f4_phase_error_lpf - f4_phase_error_lpf_pre) * (aid_f4_ctrl_freq_hz * f4_lpf_num));

    aid_f4_e_lpf = aid_f4_e_lpf * f4_lpf_den + aid_f4_e * f4_lpf_num;
} /* End of function core_estimator_filter */

/*******************************************************************************************************************//**
 * Record debug data when assert failure happened
 *
 * @param[in] line  The line where the assertion failure happened
 * @param[in] file  The file code where the assertion failure happened
 **********************************************************************************************************************/
void aid_core_assert_fail(uint32_t line, uint32_t file)
{
    aid_core_throw_error(AID_ERROR_ASSERT_FAIL);
    dbg_u4_line = line;
    dbg_u4_file = file;
} /* End of function aid_core_assert_fail */

/*******************************************************************************************************************//**
 * @brief     Throw an error with the given error code and stop PWM output
 * @param[in] u2_error_code  The error code of the error
 **********************************************************************************************************************/
void aid_core_throw_error(uint16_t u2_error_code)
{
    aid_u2_error_status = u2_error_code;
    aid_core_stop();
    aid_core_on_error(u2_error_code);
} /* End of function aid_core_throw_error */

/*******************************************************************************************************************//**
 * Configure start-up sequence
 *
 * @param[in] f4_ref_id           The Id[A] injected in open-loop mode
 * @param[in] f4_ol2cl_speed_rad  The open-loop to closed-loop switching threshold speed in [rad/s]
 * @param[in] f4_cl2ol_speed_rad  The closed-loop to open-loop switching threshold speed in [rad/s]
 **********************************************************************************************************************/
void aid_core_set_startup_params(float f4_ref_id, float f4_ol2cl_speed_rad, float f4_cl2ol_speed_rad)
{
    AID_ASSERT(f4_ref_id >= 0.0f);
    AID_ASSERT(f4_cl2ol_speed_rad >= 0.0f);
    AID_ASSERT(f4_ol2cl_speed_rad >= f4_cl2ol_speed_rad);

    aid_f4_ref_id = f4_ref_id;
    aid_f4_id_down_speed_rad = f4_ol2cl_speed_rad;
    aid_f4_id_up_speed_rad = f4_cl2ol_speed_rad;
    aid_f4_id_up_step = aid_f4_ref_id / g_f4_aid_id_up_time * aid_f4_spd_ctrl_period_ms;
    aid_f4_id_down_step = aid_f4_ref_id / g_f4_aid_id_down_time * aid_f4_spd_ctrl_period_ms;
} /* End of function aid_core_set_startup_params */

/*******************************************************************************************************************//**
 * Configure speed pi with given gains
 *
 * @param f4_kp    - The proportional gain
 * @param f4_kidt  - The integral gain
 **********************************************************************************************************************/
void aid_core_set_speed_pi(float f4_kp, float f4_kidt)
{
    AID_ASSERT(f4_kp    >= 0.0f);
    AID_ASSERT(f4_kidt  >= 0.0f);

    aid_f4_kp_speed = f4_kp;
    aid_f4_ki_speed = f4_kidt;
    st_aid_speed.f4_kp = f4_kp;
    st_aid_speed.f4_ki = f4_kidt;
} /* End of function aid_core_set_speed_pi */

/*******************************************************************************************************************//**
 * @brief     Configure rated current, this will also change the current limits and the over-current threshold
 * @param[in] f4_rated_current  The value [A] to be set as the rated current
 **********************************************************************************************************************/
void aid_core_set_rated_current(float f4_rated_current)
{
    AID_ASSERT(f4_rated_current > 0.0f);

#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    aid_f4_imag_max = f4_rated_current * AID_SQRT_3;
    aid_f4_lim_iq = aid_f4_imag_max;
    st_aid_speed.f4_ilimit   = aid_f4_lim_iq;

    /* The same phase current, just convert [Arms] to [A] by multiply sqrt(2), g_f4_aid_overcurrent_limit_mult is for
     * adding some margin, so it should be larger than 1.0 */
    aid_f4_overcurrent_limit = f4_rated_current * AID_SQRT_2 * g_f4_aid_overcurrent_limit_mult;
#else
    /* Set current limits, the rated current of steppers represents maximum peak current */
    aid_f4_imag_max          = f4_rated_current;
    aid_f4_lim_iq            = aid_f4_imag_max;
    st_aid_speed.f4_ilimit   = aid_f4_lim_iq;
    aid_f4_overcurrent_limit = aid_f4_imag_max * g_f4_aid_overcurrent_limit_mult;
#endif
    if (aid_f4_overcurrent_limit > aid_f4_overcurrent_limit_hw)
    {
        aid_f4_overcurrent_limit = aid_f4_overcurrent_limit_hw;
    }
} /* End of function aid_core_set_rated_current */

/*******************************************************************************************************************//**
 * @brief     Configures the voltage error compensation module with the given lookup table
 *
 * @param[in] f4_current_tab  The pointer to the current column array of the lookup table
 * @param[in] f4_voltage_tab  The pointer to the voltage column array of the lookup table
 * @param[in] f4_vdc_ref      The reference DC bus voltage which is the measurement condition of the given table
 **********************************************************************************************************************/
void aid_core_set_volterr_table(const float *f4_current_tab, const float *f4_voltage_tab, float f4_vdc_ref)
{
    AID_ASSERT(NULL != f4_current_tab);
    AID_ASSERT(NULL != f4_voltage_tab);
    AID_ASSERT(f4_vdc_ref > 0.0f);

    aid_volterr_comp_set_table(&st_aid_volt_comp_p, f4_current_tab, f4_voltage_tab, f4_vdc_ref);
} /* End of function aid_core_set_volterr_table */

void aid_core_clear_error(void)
{
    aid_u2_error_status = AID_ERROR_NONE;
} /* End of function aid_core_clear_error */

/*******************************************************************************************************************//**
 * Configure current pi with given gains
 *
 * @param[in] f4_omega  The natural frequency in [Hz]
 * @param[in] f4_zeta   The damping factor
 **********************************************************************************************************************/
void aid_core_config_current_pi_gains(float f4_omega, float f4_zeta)
{
    float f4_omega_sqr;
    float f4_omega_rad;
    AID_ASSERT(f4_omega > 0.0f);
    AID_ASSERT((f4_zeta > 0.0f) && (f4_zeta <= 1.0f));

    f4_omega_rad = f4_omega * AID_TWOPI;
    f4_omega_sqr    = f4_omega_rad * f4_omega_rad;
    aid_f4_omega_current = f4_omega * AID_TWOPI;
    aid_f4_zeta_current = f4_zeta;

    aid_f4_kp_id    = (2.0f * f4_omega_rad * f4_zeta * aid_f4_ld) - aid_f4_r;
    st_aid_vd.f4_kp = aid_f4_kp_id;
    aid_f4_ki_id    = f4_omega_sqr * aid_f4_ld * aid_f4_ctrl_period_ms * 0.001f; 
    st_aid_vd.f4_ki = aid_f4_ki_id;
    aid_f4_kp_iq    = (2.0f * f4_omega_rad * f4_zeta * aid_f4_lq) - aid_f4_r;
    st_aid_vq.f4_kp = aid_f4_kp_iq;
    aid_f4_ki_iq    = f4_omega_sqr * aid_f4_lq * aid_f4_ctrl_period_ms * 0.001f;
    st_aid_vq.f4_ki = aid_f4_ki_iq;
    
    kp_id_final = 4096 * aid_f4_kp_id;
    ki_id_final = 16384 * aid_f4_ki_id;
    kp_iq_final = 4096 * aid_f4_kp_iq;
    ki_iq_final = 16384 * aid_f4_ki_iq;
    
  
    
} /* End of function aid_core_config_current_pi_gains */

/*******************************************************************************************************************//**
 * Configure speed pi with given gains
 *
 * @param[in] f4_omega  The natural frequency in [Hz]
 * @param[in] f4_zeta   The damping factor
 **********************************************************************************************************************/
void aid_core_config_bemf_obsv(float f4_omega, float f4_zeta)
{
    float f4_omega_sqr;
    float f4_omega_rad;

    AID_ASSERT(f4_omega > 0.0f);
    AID_ASSERT((f4_zeta > 0.0f) && (f4_zeta <= 1.0f));

    f4_omega_rad = f4_omega * AID_TWOPI;
    f4_omega_sqr = f4_omega_rad * f4_omega_rad;

    aid_f4_omega_e_obs = f4_omega_rad;
    aid_f4_zeta_e_obs = f4_zeta;

    /* Design of BEMF observer */
    f4_omega_sqr       = f4_omega_rad * f4_omega_rad;
    aid_f4_k_e_obs_d_1 = (2.0f * f4_omega_rad * f4_zeta) - (aid_f4_r / aid_f4_ld);
    aid_f4_k_e_obs_d_2 = f4_omega_sqr * aid_f4_ld;
    aid_f4_k_e_obs_q_1 = (2.0f * f4_omega_rad * f4_zeta) - (aid_f4_r / aid_f4_lq);
    aid_f4_k_e_obs_q_2 = f4_omega_sqr * aid_f4_lq;

} /* End of function aid_core_config_bemf_obsv */
float g_f4_vd_ref_mon;
float g_f4_id_ad_mon;
/*******************************************************************************************************************//**
 * @brief Carrier interrupt handler of CORE module
 * @details This function defines the process should be executed every PWM cycle
 * @pre AD conversion of current and DC bus voltage must be done before calling this function
 **********************************************************************************************************************/
void aid_core_crnt_ctrl_handler(void)
{
    float f4_temp0;
    uint32_t ret;
    float temp_f4_abc[3];
    float temp_f4_abc_2[3];
    float temp_f4_dq[2];

    if (AID_RUNMODE_INIT == aid_u2_run_mode)
    {
        core_restart_drive();
        aid_u2_run_mode = AID_RUNMODE_BOOT;
    }

//setpsw_i();                                                /* interrupt enable */

    /* Current measurement */
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    aid_hal_get_current_abc(&aid_f4_ia_ad, &aid_f4_ib_ad, &aid_f4_ic_ad);
#else
    aid_hal_get_current_ab(&aid_f4_ia_ad, &aid_f4_ib_ad);
#endif

    /* Vdc measurement, calculate maximum magnitude of output voltage vector */
    aid_f4_vdc_ad = aid_hal_get_vdc();
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    core_set_voltage_limit(aid_f4_duty_available * aid_f4_vdc_ad * 0.5f);
#else
    core_set_voltage_limit(aid_f4_duty_available * aid_f4_vdc_ad);
#endif


    if (AID_RUNMODE_BOOT == aid_u2_run_mode)
    {
        ret = core_offset_measure();
        if (1 == ret)
        {
            aid_u2_run_mode = AID_RUNMODE_READY;
        }
    }

    /* Current offset adjustment */
    aid_f4_ia_ad = aid_f4_ia_ad - aid_f4_offset_ia;
    aid_f4_ib_ad = aid_f4_ib_ad - aid_f4_offset_ib;
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    aid_f4_ic_ad = aid_f4_ic_ad - aid_f4_offset_ic;
#endif
    /* Error check */
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    core_check_over_current_error(aid_f4_ia_ad, aid_f4_ib_ad, aid_f4_ic_ad, aid_f4_overcurrent_limit);
#else
    core_check_over_current_error(aid_f4_ia_ad, aid_f4_ib_ad, 0.0f, aid_f4_overcurrent_limit);
#endif
    core_check_over_voltage_error(aid_f4_vdc_ad, aid_f4_overvoltage_limit);
    core_check_under_voltage_error(aid_f4_vdc_ad, aid_f4_undervoltage_limit);

    if (AID_RUNMODE_READY == aid_u2_run_mode && AID_ERROR_NONE == aid_u2_error_status)
    {

#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        temp_f4_abc[0] = aid_f4_ia_ad;
        temp_f4_abc[1] = aid_f4_ib_ad;
        temp_f4_abc[2] = aid_f4_ic_ad;
        aid_transform_uvw_dq_abs(&aid_st_rotor_angle, temp_f4_abc , temp_f4_dq);
        aid_f4_id_ad = temp_f4_dq[0];
        aid_f4_iq_ad = temp_f4_dq[1];

#else
        /* Park transformation (ab -> dq) */
        aid_f4_id_ad = (aid_f4_temp_cos * aid_f4_ia_ad) + (aid_f4_temp_sin * aid_f4_ib_ad);
        aid_f4_iq_ad = ((-aid_f4_temp_sin) * aid_f4_ia_ad) + (aid_f4_temp_cos * aid_f4_ib_ad);
#endif
        /* BEMF Observer */
        if (aid_u2_ctrl_level >= AID_CTRL_LEVEL_2)
        {
            core_bemf_observer();
            core_estimator_filter(aid_f4_phase_error, aid_f4_e);
        }

        /* Angle and speed estimation/command generation (in open-loop operation)  */
        if (aid_u2_ctrl_level >= AID_CTRL_LEVEL_3)
        {
            core_angle_speed_est();
        }

#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        aid_rotor_angle_update(&aid_st_rotor_angle, aid_f4_angle_rad);
#else
    aid_f4_temp_cos = cosf(aid_f4_angle_rad);
    aid_f4_temp_sin = sinf(aid_f4_angle_rad);
#endif

        /* Motor parameter identification sequences */
        gs_before_current_ctrl();   /* =>aid_motorid_sequence() */

        if (aid_u2_ctrl_level >= AID_CTRL_LEVEL_2)
        {
            /* Current PI */
            aid_f4_vd_ref = aid_pi_ctrl(&st_aid_vd, aid_f4_id_ref - aid_f4_id_ad);
            aid_f4_vq_ref = aid_pi_ctrl(&st_aid_vq, aid_f4_iq_ref - aid_f4_iq_ad);
        }

#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        temp_f4_dq[0] = aid_f4_vd_ref;
        temp_f4_dq[1] = aid_f4_vq_ref;
        aid_transform_dq_uvw_abs(&aid_st_rotor_angle, temp_f4_dq, temp_f4_abc);
        aid_f4_va_ref = temp_f4_abc[0];
        aid_f4_vb_ref = temp_f4_abc[1];
        aid_f4_vc_ref = temp_f4_abc[2];
        g_f4_vd_ref_mon = aid_f4_vd_ref;
        g_f4_id_ad_mon = aid_f4_id_ad;
#else
        /*     Inverse park transformation (dq -> ab)      */
        aid_f4_va_ref = (aid_f4_temp_cos * aid_f4_vd_ref) - (aid_f4_temp_sin * aid_f4_vq_ref);
        aid_f4_vb_ref = (aid_f4_temp_sin * aid_f4_vd_ref) + (aid_f4_temp_cos * aid_f4_vq_ref);
#endif

        /*     voltage error compensation     */
        if (aid_u2_ctrl_level >= AID_CTRL_LEVEL_3)
        {
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
            temp_f4_dq[0] = aid_f4_id_ref;
            temp_f4_dq[1] = aid_f4_iq_ref;
            aid_transform_dq_uvw_abs(&aid_st_rotor_angle, temp_f4_dq, temp_f4_abc);
            aid_f4_ia_ref = temp_f4_abc[0];
            aid_f4_ib_ref = temp_f4_abc[1];
            aid_f4_ic_ref = temp_f4_abc[2];

            temp_f4_abc[0] = aid_f4_va_ref;
            temp_f4_abc[1] = aid_f4_vb_ref;
            temp_f4_abc[2] = aid_f4_vc_ref;
            temp_f4_abc_2[0] = aid_f4_ia_ref;
            temp_f4_abc_2[1] = aid_f4_ib_ref;
            temp_f4_abc_2[2] = aid_f4_ic_ref;
            aid_volterr_comp_main(&st_aid_volt_comp_p, temp_f4_abc, temp_f4_abc_2, aid_f4_vdc_ad);
            aid_f4_va_ref_comp = temp_f4_abc[0];
            aid_f4_vb_ref_comp = temp_f4_abc[1];
            aid_f4_vc_ref_comp = temp_f4_abc[2];
#else
            /* Calculate the ab current command to be used in voltage compensation as the inputs  */
            aid_f4_ia_ref = (aid_f4_temp_cos * aid_f4_id_ref) - (aid_f4_temp_sin * aid_f4_iq_ref);
            aid_f4_ib_ref = (aid_f4_temp_sin * aid_f4_id_ref) + (aid_f4_temp_cos * aid_f4_iq_ref);

            /* Only the first 2 phases require voltage compensation, ignore the 3rd phase */
            aid_f4_vcomp_array[0] = aid_f4_va_ref;
            aid_f4_vcomp_array[1] = aid_f4_vb_ref;
            aid_f4_vcomp_i_array[0] = aid_f4_ia_ref;
            aid_f4_vcomp_i_array[1] = aid_f4_ib_ref;
            aid_volterr_comp_main_ab(&st_aid_volt_comp_p, aid_f4_vcomp_array, aid_f4_vcomp_i_array, aid_f4_vdc_ad);

            aid_f4_va_ref_comp = aid_f4_vcomp_array[0];
            aid_f4_vb_ref_comp = aid_f4_vcomp_array[1];
#endif
        }
        else if (aid_u2_ctrl_level >= AID_CTRL_LEVEL_1)
        {
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
            temp_f4_abc[0] = aid_f4_va_ref;
            temp_f4_abc[1] = aid_f4_vb_ref;
            temp_f4_abc[2] = aid_f4_vc_ref;
            temp_f4_abc_2[0] = aid_f4_ia_ref;
            temp_f4_abc_2[1] = aid_f4_ib_ref;
            temp_f4_abc_2[2] = aid_f4_ic_ref;
            aid_volterr_comp_main(&st_aid_volt_comp_p, temp_f4_abc, temp_f4_abc_2, aid_f4_vdc_ad);
            aid_f4_va_ref_comp = temp_f4_abc[0];
            aid_f4_vb_ref_comp = temp_f4_abc[1];
            aid_f4_vc_ref_comp = temp_f4_abc[2];
#else
            /* Only the first 2 phases require voltage compensation, ignore the 3rd phase */
            aid_f4_vcomp_array[0] = aid_f4_va_ref;
            aid_f4_vcomp_array[1] = aid_f4_vb_ref;
            aid_f4_vcomp_i_array[0] = aid_f4_ia_ad;
            aid_f4_vcomp_i_array[1] = aid_f4_ib_ad;
            aid_volterr_comp_main_ab(&st_aid_volt_comp_p, aid_f4_vcomp_array, aid_f4_vcomp_i_array, aid_f4_vdc_ad);

            aid_f4_va_ref_comp = aid_f4_vcomp_array[0];
            aid_f4_vb_ref_comp = aid_f4_vcomp_array[1];
#endif
        }
        else
        {
            aid_f4_va_ref_comp = aid_f4_va_ref;
            aid_f4_vb_ref_comp = aid_f4_vb_ref;
            aid_f4_vc_ref_comp = aid_f4_vc_ref;
        }

        /*     limit (Voltage)     */
        aid_f4_va_ref_comp = R_AID_LimitfAbs(aid_f4_va_ref_comp, aid_f4_vphase_limit);
        aid_f4_vb_ref_comp = R_AID_LimitfAbs(aid_f4_vb_ref_comp, aid_f4_vphase_limit);
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        aid_f4_vc_ref_comp = R_AID_LimitfAbs(aid_f4_vc_ref_comp, aid_f4_vphase_limit);
#endif

        /*     calculate modulation     */
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        f4_temp0 = 1.0f / aid_f4_vdc_ad;
#else
        f4_temp0 = 0.5f / aid_f4_vdc_ad;                               /* modulation range (- vdc/2 -> vdc/2) to (- 1 -> 1 )  */
#endif

        aid_f4_moda = (aid_f4_va_ref_comp * f4_temp0) + 0.5f;
        aid_f4_modb = (aid_f4_vb_ref_comp * f4_temp0) + 0.5f;
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        aid_f4_modc = (aid_f4_vc_ref_comp * f4_temp0) + 0.5f;
#endif

        /*     PWM reference setting    */
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        aid_hal_pwm_set_duty_abc(aid_f4_moda, aid_f4_modb, aid_f4_modc);
#else
        aid_hal_pwm_set_duty_ab(aid_f4_moda, aid_f4_modb);
#endif
        {
            float f4_va_pwm_duty;
            float f4_vb_pwm_duty;
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
            float f4_vc_pwm_duty;
            aid_hal_pwm_get_duty_abc(&f4_va_pwm_duty, &f4_vb_pwm_duty, &f4_vc_pwm_duty);
#else
            aid_hal_pwm_get_duty_ab(&f4_va_pwm_duty, &f4_vb_pwm_duty);
#endif
            aid_f4_va_ref_pwm = (f4_va_pwm_duty - 0.5f) * 2.0f * aid_f4_vdc_ad;
            aid_f4_vb_ref_pwm = (f4_vb_pwm_duty - 0.5f) * 2.0f * aid_f4_vdc_ad;
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
            aid_f4_vc_ref_pwm = (f4_vc_pwm_duty - 0.5f) * 2.0f * aid_f4_vdc_ad;
#endif
        }

        gs_after_pwm_output();   /* =>aid_motorid_sequence_post() */
    }
} /* End of function aid_core_crnt_ctrl_handler */

/*******************************************************************************************************************//**
 * Long period interrupt handler, for speed control and start-up sequence control
 **********************************************************************************************************************/
void aid_core_spd_ctrl_handler(void)
{
    float f4_temp0;

    aid_f4_1ms_speed_rad = aid_f4_speed_lpf_rad;                  /* speed */

    if (aid_u2_ctrl_level >= AID_CTRL_LEVEL_4)
    {
        /***** sensorless to openloop *****/
        f4_temp0 = fabsf(aid_f4_ref_speed_rad_ctrl);
        if (AID_ROTOR_ANGLE_MODE_CLOSEDLOOP == aid_s4_rotor_angle_mode)
        {
            if (f4_temp0 < aid_f4_id_up_speed_rad)
            {
                aid_u1_flag_down_to_ol = AID_FLG_SET;
            }
        }

        /***** Id, Iq, speed reference setting *****/
        aid_f4_ref_speed_rad_ctrl = core_generate_speed_ref();
        aid_f4_iq_ref = core_generate_iq_ref();
        aid_f4_id_ref = core_generate_id_ref();

        aid_u1_flag_down_to_ol = AID_FLG_CLR;

        if (AID_IQ_SPEED_PI_OUTPUT == aid_u1_flag_iq_ref)
        {
            aid_u2_ctrl_conf |= AID_CONTROL_SPEED;
        }
        else
        {
            aid_u2_ctrl_conf = AID_CONTROL_CURRENT;
        }
    }
} /* End of function aid_core_spd_ctrl_handler */

/*******************************************************************************************************************//**
 * @brief Stops driving motor, this also stops the PWM output
 **********************************************************************************************************************/
void aid_core_stop(void)
{
    /* Stop inverter */
    aid_hal_pwm_disable();                                /* PWM output disable */

    aid_f4_vd_ref = 0.0f;
    aid_f4_vq_ref = 0.0f;
    aid_f4_id_ref = 0.0f;
    aid_f4_iq_ref = 0.0f;
    aid_f4_speed_rad = 0.0f;
    aid_f4_ref_speed_rad = 0.0f;

    aid_u2_run_mode = AID_RUNMODE_INIT;
    aid_core_set_ctrl_level(AID_CTRL_LEVEL_0);
} /* End of function aid_core_stop */

/*******************************************************************************************************************//**
 * @brief     Core module initialization
 *
 * @param[in] pwm_tick_per_irq     The PWM tick per interrupt
 * @param[in] speed_ctrl_period    The speed control period
 * @param[in] before_current_ctrl  The handler of the event (function pointer) happens before current control
 * @param[in] after_pwm_output     The handler of the event (function pointer) happens after PWM output
 **********************************************************************************************************************/
void aid_core_init(uint8_t pwm_tick_per_irq,
                   float speed_ctrl_period,
                   core_delegate_t before_current_ctrl,
                   core_delegate_t after_pwm_output)
{
    float pwm_tick_cycle_ms;

    gs_before_current_ctrl = before_current_ctrl;
    gs_after_pwm_output = after_pwm_output;

    /* Check prerequisites */
    AID_ASSERT(pwm_tick_per_irq >= 1);

    /* Get inverter information from driver */
    aid_hal_get_inv_info(&aid_st_inv_info);

    /* Calculate cycles */
    AID_ASSERT(aid_st_inv_info.pwm_cycle_s > 0.0f);
    pwm_tick_cycle_ms = aid_st_inv_info.pwm_cycle_s * 1000.0f;
    aid_f4_pwm_period_ms = pwm_tick_cycle_ms;
    aid_f4_ctrl_period_ms = pwm_tick_cycle_ms * pwm_tick_per_irq;
    aid_f4_ctrl_freq_hz = 1.0f / aid_st_inv_info.pwm_cycle_s / pwm_tick_per_irq;

    AID_ASSERT((speed_ctrl_period * 1000.0f) > aid_f4_pwm_period_ms);
    aid_f4_spd_ctrl_period_ms = speed_ctrl_period * 1000.0f;
    aid_f4_spd_ctrl_freq_hz = 1.0f / speed_ctrl_period;

    /* Calculate voltage limiter coefficient, STM should be difference from BLDC,
     * actual maximum voltage magnitude will be calculated before offset calibration */
    AID_ASSERT(aid_st_inv_info.duty_min >= 0.0f);
    AID_ASSERT(aid_st_inv_info.duty_max > aid_st_inv_info.duty_min);
    if (aid_st_inv_info.duty_min > (1.0f - aid_st_inv_info.duty_max))
    {
        aid_f4_duty_available = 1.0f - (2.0f * aid_st_inv_info.duty_min);
    }
    else
    {
        aid_f4_duty_available = (aid_st_inv_info.duty_max * 2.0f) - 1.0f;
    }

    /* Set up protection thresholds */
    AID_ASSERT(aid_st_inv_info.overcurrent_th > 0.0f);
    aid_f4_overcurrent_limit_hw = aid_st_inv_info.overcurrent_th;
    aid_f4_overcurrent_limit = aid_f4_overcurrent_limit_hw;

    AID_ASSERT(aid_st_inv_info.overvoltage_th > 0.0f);
    aid_f4_overvoltage_limit = aid_st_inv_info.overvoltage_th;

    AID_ASSERT(aid_st_inv_info.undervoltage_th > 0.0f &&
            (aid_st_inv_info.undervoltage_th < aid_st_inv_info.overvoltage_th));
    aid_f4_undervoltage_limit = aid_st_inv_info.undervoltage_th;

    AID_ASSERT(aid_st_inv_info.current_lsb > 0.0f)
    aid_f4_current_lsb = aid_st_inv_info.current_lsb;

    AID_ASSERT(aid_st_inv_info.pwm_lsb > 0.0f)
    aid_f4_pwm_duty_lsb = aid_st_inv_info.pwm_lsb;

    aid_rotor_angle_init(&aid_st_rotor_angle);
    aid_volterr_comp_init(&st_aid_volt_comp_p, 0, 0, 1);
} /* End of function aid_core_init */

/*******************************************************************************************************************//**
 * @brief     Sets the control level of CORE module
 * @param[in] ctrl_level  The control level
 **********************************************************************************************************************/
void aid_core_set_ctrl_level(uint16_t ctrl_level)
{
    AID_ASSERT(ctrl_level <= AID_CTRL_LEVEL_4);
    aid_u2_ctrl_level = ctrl_level;
} /* End of function aid_core_set_ctrl_level */

/*******************************************************************************************************************//**
 * @brief   Resets CORE modules includes gain and motor parameters
 * @warning This function will resets all parameters in the module, and should be used only when new identification
 *          starts
 **********************************************************************************************************************/
void aid_core_reset(void)
{
    /* Common */
    aid_u4_sample_cnt = 0;
    aid_u2_1ms_cnt    = 0;
    aid_u2_sum_cnt    = 0;

    /* Reset modes */
    aid_s4_rotor_angle_mode = AID_ROTOR_ANGLE_MODE_OPENLOOP;
    aid_core_set_ctrl_level(AID_CTRL_LEVEL_0);
    aid_u2_error_status    = AID_ERROR_NONE;
    aid_u2_run_mode        = AID_RUNMODE_INIT;
    aid_u1_flag_id_ref     = AID_ID_UP;
    aid_u1_flag_iq_ref     = AID_IQ_ZERO_CONST;
    aid_u1_flag_speed_ref  = AID_SPEED_ZERO_CONST;
    aid_u1_flag_down_to_ol = 0;
    aid_u2_ctrl_conf = 0;

    /* Reset motor parameters */
    aid_f4_r_dc  = 0.0f;
    aid_f4_v_err = 0.0f;
    aid_f4_r     = 0.0f;
    aid_f4_ld    = 0.0f;
    aid_f4_lq    = 0.0f;
    aid_f4_ke    = 0.0f;
    aid_f4_j     = 0.0f;

    /* Motor driver */
    aid_f4_ia_ad  = 0.0f;
    aid_f4_ib_ad  = 0.0f;
    aid_f4_vd_ref = 0.0f;
    aid_f4_vq_ref = 0.0f;
    aid_f4_ia_ref = 0.0f;
    aid_f4_ib_ref = 0.0f;
    aid_f4_id_ref = 0.0f;
    aid_f4_iq_ref = 0.0f;
    aid_f4_id_ad  = 0.0f;
    aid_f4_iq_ad  = 0.0f;
    aid_f4_vdc_ad = 0.0f;

    aid_f4_imag_max = 1.0f;         /* This value will be overrided when setting rated current */


    /* Reset offset calibration */
    aid_f4_offset_ia = 0.0f;
    aid_f4_offset_ib = 0.0f;
    aid_f4_pre_offset_ia = 0.0f;
    aid_f4_pre_offset_ib = 0.0f;
    aid_f4_offset_lpf_k = g_f4_aid_offset_lpf_k;
    aid_f4_offset_calc_time = g_f4_aid_offset_calc_time;
    aid_u2_cnt_adjust = 0;

    /* Reset PLL */
    aid_f4_phase_error = 0.0f;
    aid_f4_speed_rad = 0.0f;
    aid_f4_angle_rad = 0.0f;
    aid_f4_kp_est_speed = 2 * AID_TWOPI * g_f4_aid_pll_est_omega * g_f4_aid_pll_est_zeta;
    aid_f4_ki_est_speed = (AID_TWOPI * g_f4_aid_pll_est_omega) * (AID_TWOPI * g_f4_aid_pll_est_omega) *
            (aid_f4_ctrl_period_ms * 0.001f);
    aid_f4_i_est_speed = 0.0f;
    aid_f4_temp_cos = 0.0f;
    aid_f4_temp_sin = 0.0f;

    /* Reset current control */
    aid_f4_omega_current = AID_TWOPI * g_f4_aid_current_omega;
    aid_f4_zeta_current = g_f4_aid_current_zeta;
    aid_f4_kp_id = 0.0f;
    aid_f4_ki_id = 0.0f;
    aid_f4_kp_iq = 0.0f;
    aid_f4_ki_iq = 0.0f;
    st_aid_vd.f4_kp = 0.0f;
    st_aid_vd.f4_ki = 0.0f;
    st_aid_vd.f4_refi = 0.0f;
    st_aid_vd.f4_ilimit = aid_f4_overvoltage_limit;
    st_aid_vq.f4_kp = 0.0f;
    st_aid_vq.f4_ki = 0.0f;
    st_aid_vq.f4_refi = 0.0f;
    st_aid_vq.f4_ilimit = aid_f4_overvoltage_limit;

    /* Reset BEMF observer */
    aid_f4_omega_e_obs = AID_TWOPI * g_f4_aid_e_obs_omega;
    aid_f4_zeta_e_obs = g_f4_aid_e_obs_zeta;
    aid_f4_k_e_obs_d_1 = 0.0f;
    aid_f4_k_e_obs_d_2 = 0.0f;
    aid_f4_k_e_obs_q_1 = 0.0f;
    aid_f4_k_e_obs_q_2 = 0.0f;
    aid_f4_pre_id_ad = 0.0f;
    aid_f4_pre_id_est = 0.0f;
    aid_f4_pre_dd = 0.0f;
    aid_f4_id_est = 0.0f;
    aid_f4_dd = 0.0f;
    aid_f4_pre_iq_ad = 0.0f;
    aid_f4_pre_iq_est = 0.0f;
    aid_f4_pre_dq = 0.0f;
    aid_f4_iq_est = 0.0f;
    aid_f4_dq = 0.0f;
    aid_f4_ed = 0.0f;
    aid_f4_eq = 0.0f;
    aid_f4_e = 0.0f;
    aid_f4_e_lpf = 0.0f;

    /* Reset speed control, speed change rate limiter */
    aid_f4_kp_speed = 0.0f;
    aid_f4_ki_speed = 0.0f;
    aid_f4_lim_iq = 0.0f;
    aid_f4_ref_speed_rad_ctrl = 0.0f;
    aid_f4_ref_speed_rad = 0.0f;
    aid_f4_limit_speed_change = g_f4_aid_speed_change_rate_limit;
    st_aid_speed.f4_kp = 0.0f;
    st_aid_speed.f4_ki = 0.0f;
    st_aid_speed.f4_refi = 0.0f;
    st_aid_speed.f4_ilimit = 0.0f;

    /* Reset start-up sequence parameters */
    aid_f4_id_down_speed_rad = 0.0f;
    aid_f4_id_up_speed_rad = 0.0f;
    aid_f4_id_up_step = 0.0f;
    aid_f4_id_down_step = 0.0f;
    aid_f4_ref_id = 0.0f;
    aid_f4_iq_down_step = 0.0f;
    aid_f4_ol_iq_down_step = 1.0f / g_f4_aid_iq_down_time;

    /* Resets modulation */
    aid_f4_va_ref = 0.0f;
    aid_f4_vb_ref = 0.0f;
    aid_f4_va_ref_comp = 0.0f;
    aid_f4_vb_ref_comp = 0.0f;
    aid_f4_vphase_limit = 0.0f;
    aid_f4_moda = 0.0f;
    aid_f4_modb = 0.0f;

    /* Reset voltage error compensation */
    aid_f4_vcomp_array[0] = 0.0f;
    aid_f4_vcomp_array[1] = 0.0f;
    aid_f4_vcomp_i_array[0] = 0.0f;
    aid_f4_vcomp_i_array[1] = 0.0f;
    st_aid_volt_comp_p.u1_volt_err_comp_enable = AID_FLG_SET;

    /* Reset speed LPF */
    aid_f4_speed_lpf_rad = 0.0f;
    aid_f4_1ms_speed_rad = 0.0f;
    aid_f4_speed_lpf_omega_hz = 125.0f;
    R_AID_FirstOrderLpffInit(&aid_st_speed_lpf);
    R_AID_FirstOrderLpffGainCalc(&aid_st_speed_lpf, aid_f4_speed_lpf_omega_hz, aid_f4_ctrl_period_ms * 0.001f);

    /* Reset flags */
    aid_st_warn.u4_warn_core_flags = 0;

#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    aid_f4_ic_ad = 0.0f;
    aid_f4_vc_ref = 0.0f;
    aid_f4_va_ref_comp = 0.0f;
    aid_f4_modc = 0.0f;
    aid_rotor_angle_reset(&aid_st_rotor_angle);
#endif
} /* End of function aid_core_reset */

/*******************************************************************************************************************//**
 * @brief   Resets start-up sequence and related variables
 **********************************************************************************************************************/
void aid_core_reset_startup_seq(void)
{
    aid_f4_id_ref               = 0.0f;
    aid_f4_iq_ref               = 0.0f;
    aid_f4_ref_speed_rad_ctrl   = 0.0f;
    aid_u1_flag_speed_ref       = AID_SPEED_ZERO_CONST;
    aid_u1_flag_id_ref          = AID_ID_UP;
    aid_u1_flag_iq_ref          = AID_IQ_ZERO_CONST;
    aid_f4_torque_current       = 0.0f;
} /* End of function aid_core_reset_startup_seq */

/***********************************************************************************************************************
 * Function Name: aid_core_warning
 * Description  : Send warning message without stopping identification
 * Arguments    : u4_warn_code -
 *                    Which warning has happened
 *                f4_related_val -
 *                    The value related to the warning to send
 * Return Value : None
 **********************************************************************************************************************/
void aid_core_warning(uint32_t u4_warn_code)
{
    switch(u4_warn_code)
    {
        case AID_WARN_NONE:
        {
            __asm __volatile("nop\n");
        }
        break;

        /* These warning code conditions are intentionally combined */
        case AID_WARN_VOLT_LIMIT:
        case AID_WARN_CURNT_LIMIT:
        {
            aid_st_warn.u4_warn_core_flags |= u4_warn_code;
        }
        break;

        /* Not a predefined warning in core, TODO: require father implementation */
        default:
        {
            /* Do nothing */
            __asm __volatile("nop\n");
        }
    }
} /* End of function aid_core_warning */

/** @} */
/* End of file r_aid_core.c */
