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
 * File Name    : r_aid_jd.c
 * Version      : 1.0
 * Description  : Inertia & friction identification module
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <string.h>
//#include <machine.h>
#include "r_aid_jd.h"
#include "r_aid_core.h"
#include "r_aid_config.h"
#include "r_aid_function.h"
#include "r_aid_rls.h"

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#ifndef AID_PRV_FILE_CODE
#define AID_PRV_FILE_CODE (106)
#endif
#define JD_PRV_KI_MULT              (0.2f)
#define JD_PRV_RLS_MODE             (2) /* Defines RLS mode, 2:general RLS function */
#define JD_PRV_SPEED_USE_PLL        (1)
#define JD_PRV_SPEED_USE_1MS        (2)
#define JD_PRV_IQ_USE_LPF           (1)
#define JD_PRV_IQ_USE_AVG           (2)
#define JD_PRV_SPEED                (JD_PRV_SPEED_USE_PLL)
#define JD_PRV_IQ                   (JD_PRV_IQ_USE_AVG)

#define JD_PRV_MEASURE_SEQ_STOP     (0)
#define JD_PRV_MEASURE_SEQ_KE       (1)
#define JD_PRV_MEASURE_SEQ_SETUP    (2)
#define JD_PRV_MEASURE_SEQ_RLS      (3)
#define JD_PRV_MEASURE_SEQ_DFT      (4)
#define JD_PRV_MEASURE_SEQ_COMP     (5)

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
typedef struct
{
    e_aid_seq_j_t e_status;         /* JD main sequence status see definition of type e_aid_seq_j_t */
    uint8_t u1_j_seq;               /* JD measure sequence status see macro with prefix JD_PRV_MEASURE_SEQ */
    float f4_iq_sum;                /* Iq sum from last measure function execution */
    float f4_iq_amp;                /* Amplitude of Iq */
    float f4_iq;                    /* Iq used in JD measurement */
    float f4_iq_lpf;                /* Iq filtered by 1st order LPF */
    float f4_refspeed_freq_hz;      /* Frequency of the sine signal used as speed command */
    float f4_refspeed_amp_max;      /* Maximum amplitude of the sine signal in [rad/s]  */
    float f4_refspeed_amp;          /* Current amplitude of the sine signal in [rad/s]  */
    float f4_refspeed_offset;       /* Offset of the sine signal in [rad/s] */
    float f4_rls_respns_pre;        /* The last response (the speed) from the mechanical system for RLS method */
    float f4_speed_rad;             /* Speed used in the JD measurement */
    float f4_speed_amp;             /* Speed vibration amplitude, measured by DFT method */
    float f4_ol2cl_speed;           /* The threshold that transient from open-loop drive mode to the closed-loop mode */
    float f4_cl2ol_speed;           /* The threshold that transient from closed-loop drive mode to the open-loop mode */
    float f4_dft_cycle_cnt;         /* The counter to count how many cycles has been sampled by DFT method */
    float f4_dft_j;                 /* The inertia estimated by DFT method */
    float f4_dft_d;                 /* The viscous friction coefficient estimated by DFT method */
    float f4_rls_coefa;             /* The weight a estimated by RLS */
    float f4_rls_coefb;             /* The weight b estimated by RLS */
    float f4_iq_offset;             /* The iq offset measured with fixed speed command, to remove offset for RLS */
    float f4_j_max;                 /* The maximum tolerable estimated inertia */
    float f4_j_min;                 /* The minimum tolerable estimated inertia */
    float f4_j_pi_ctrl;             /* The inertia that is currently used to design speed PI controller */
    float f4_time_elapsed;          /* Elapsed Time [ms] */
#if (AID_JD_PRV_RLS_MODE == 3)
    st_rls3_t rls3;
#endif
} st_aid_jd_t;

/***********************************************************************************************************************
 Private global variables and functions
 **********************************************************************************************************************/
static void jd_design_speed_pi(float f4_inertia);
static void jd_adjust_speed_pi(float f4_iq_amp, float f4_speed_amp);
static void jd_estimate_jd_rls(void);
static void jd_estimate_jd_dft(void);
static void jd_seq_init(void);
static void jd_seq_ready(void);
static void jd_seq_measure(void);
static void jd_seq_check(void);
static void jd_seq_reset(void);
static float gs_f4_res_wait_cnt;
static float gs_f4_cfg_inertia_range = 0.0f;
static float gs_f4_cfg_assumed_inertia;
st_aid_jd_t aid_st_jd;

/***********************************************************************************************************************
 Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/
float aid_f4_j_rls;
float aid_f4_d_rls;
float aid_f4_j_dft;
float aid_f4_d_dft;
float dbg_f4_jd_max_overshot_ratio;

/******************************************************************************
* Function Name : aid_jd_act
* Description   :
* Arguments     : None
* Return Value  : Whether the process is done (1) or not (0)
******************************************************************************/
uint16_t aid_jd_act(void)
{
    uint16_t ret = 0;
    switch (aid_st_jd.e_status)
    {
        case AID_SEQ_J_INIT:
        {
            jd_seq_init();
        }
        break;

        case AID_SEQ_J_READY:
        {
            jd_seq_ready();
        }
        break;

        case AID_SEQ_J_MEASURE:
        {
            jd_seq_measure();
        }
        break;

        case AID_SEQ_J_CHECK:
        {
            jd_seq_check();
        }
        break;

        case AID_SEQ_J_RESET:
        {
            jd_seq_reset();
        }
        break;

        case AID_SEQ_J_COMPLETED:
        {
            ret = 1;
        }
        break;

        default:
        {
            AID_ASSERT_FAIL();
        }
        break;
    }

    return ret;
} /* End of function aid_jd_act */

/***********************************************************************************************************************
* Function Name: aid_jd_reset
* Description  : Resets JD module without changing configurations
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void aid_jd_reset(void)
{
    aid_f4_j_rls = 0.0f;
    aid_f4_j_dft = 0.0f;
    aid_f4_d_rls = 0.0f;
    aid_f4_d_rls = 0.0f;
    aid_st_jd.e_status = AID_SEQ_J_INIT;
    aid_st_jd.u1_j_seq = JD_PRV_MEASURE_SEQ_STOP;
    aid_st_jd.f4_iq_sum = 0.0f;
    aid_st_jd.f4_iq_amp = 0.0f;
    aid_st_jd.f4_iq = 0.0f;
    aid_st_jd.f4_iq_lpf = 0.0f;
    aid_st_jd.f4_refspeed_freq_hz = 0.0f;
    aid_st_jd.f4_refspeed_amp_max = 0.0f;
    aid_st_jd.f4_refspeed_amp = 0.0f;
    aid_st_jd.f4_refspeed_offset = 0.0f;
    aid_st_jd.f4_rls_respns_pre = 0.0f;
    aid_st_jd.f4_speed_rad = 0.0f;
    aid_st_jd.f4_speed_amp = 0.0f;
    aid_st_jd.f4_ol2cl_speed = 0.0f;
    aid_st_jd.f4_cl2ol_speed = 0.0f;
    aid_st_jd.f4_dft_cycle_cnt = 0.0f;
    aid_st_jd.f4_dft_d = 0.0f;
    aid_st_jd.f4_dft_j = 0.0f;
    aid_st_jd.f4_rls_coefa = 0.0f;
    aid_st_jd.f4_rls_coefb = 0.0f;
    aid_st_jd.f4_iq_offset = 0.0f;
    aid_st_jd.f4_time_elapsed = 0.0f;
} /* End of function aid_jd_reset */

/***********************************************************************************************************************
* Function Name: aid_jd_config_inertia_range
* Description  : Estimate inertia and friction coefficient by recursive mean square method
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void aid_jd_config_inertia_range(float f4_inertia_range)
{
    AID_ASSERT(f4_inertia_range >= 0.0f);
    AID_ASSERT(f4_inertia_range <= 1.0f);

    gs_f4_cfg_inertia_range = f4_inertia_range;
} /* End of function aid_jd_config_inertia_range */

/**
 * Set assumed inertia which used to design speed controller and etc.
 * @param f4_assumed_inertia    The assumed inertia, should be a value above 0
 */
void aid_jd_config_assumed_inertia(float f4_assumed_inertia)
{
    gs_f4_cfg_assumed_inertia = f4_assumed_inertia;
} /* End of function aid_jd_config_assumed_inertia */

/***********************************************************************************************************************
* Function Name: jd_design_speed_pi
* Description  : Design speed PI gains with given inertia and fixed bandwidth configuration
* Arguments    : f4_inertia - The inertia moment to be used to design speed PI controller
* Return Value : None
***********************************************************************************************************************/
static void jd_design_speed_pi(float f4_inertia)
{
    float f4_kp;
    float f4_kidt;
    float f4_speed_over_iq;
    float f4_temp;

    f4_speed_over_iq = aid_f4_ke * aid_f4_pole_pairs * aid_f4_pole_pairs;
    f4_temp = f4_inertia / f4_speed_over_iq;
    f4_kp = (g_f4_aid_jd_speedpi_omega * AID_TWOPI * 2.0f)  * f4_temp;
    f4_kidt = (AID_TWOPI * g_f4_aid_jd_speedpi_omega * AID_TWOPI * g_f4_aid_jd_speedpi_omega) * JD_PRV_KI_MULT * f4_temp * aid_f4_spd_ctrl_period_ms * 0.001f;

    /* Set PI gains through the common AID core interface */
    aid_core_set_speed_pi(f4_kp, f4_kidt);
} /* End of function jd_design_speed_pi */

/***********************************************************************************************************************
* Function Name: jd_adjust_speed_pi
* Description  : Adjust speed PI to maximize the Iq amplitude base on the actual speed, Iq amplitude
* Arguments    : f4_iq_amp - The amplitude of Iq, measured by DFT
*                f4_speed_amp - The amplitude of speed, measured by DFT
* Return Value : None
***********************************************************************************************************************/
static void jd_adjust_speed_pi(float f4_iq_amp, float f4_speed_amp)
{
    float f4_temp;
    float f4_temp1;

    if ((f4_speed_amp < (aid_st_jd.f4_refspeed_amp * g_f4_aid_jd_measure_start_amp_rate)) && (f4_iq_amp < 0.025f))
    {
        f4_temp1 = aid_st_jd.f4_refspeed_amp / f4_speed_amp;
        f4_temp = 0.025f / f4_iq_amp;
        f4_temp = (f4_temp1 < f4_temp) ? f4_temp1 : f4_temp;
        if (f4_temp > 2.0f)
        {
            f4_temp = 2.0f;
        }
        else if (f4_temp < 1.0f)
        {
            f4_temp = 1.0f;
        }
        else
        {
            /* Do nothing */
            __asm __volatile("nop\n");
        }

        aid_st_jd.f4_j_pi_ctrl *= f4_temp;
        jd_design_speed_pi(aid_st_jd.f4_j_pi_ctrl);
    }
} /* End of function jd_adjust_speed_pi */

/***********************************************************************************************************************
* Function Name: jd_estimate_jd_rls
* Description  : Estimate inertia and friction coefficient by recursive mean square method
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void jd_estimate_jd_rls(void)
{
    float f4_respns;
    float f4_linearized_iq;
    float f4_elespeed_over_iq;

    /* RLS filter , offset must be remove from both response and input terms before estimation */
    f4_respns = aid_st_jd.f4_speed_rad - aid_st_jd.f4_refspeed_offset;
    f4_linearized_iq = aid_st_jd.f4_iq - aid_st_jd.f4_iq_offset;
    f4_elespeed_over_iq = (aid_f4_ke * aid_f4_pole_pairs * aid_f4_pole_pairs);

#if (AID_JD_PRV_RLS_MODE == 2)
    aidf_rls2_exec(aid_st_jd.f4_rls_respns_pre,
                  f4_linearized_iq,
                  f4_respns,
                  &aid_st_jd.f4_rls_coefa,
                  &aid_st_jd.f4_rls_coefb);
    aid_st_jd.f4_rls_respns_pre = f4_respns;
    aid_f4_d_rls = ((1.0f - aid_st_jd.f4_rls_coefa) * f4_elespeed_over_iq) / aid_st_jd.f4_rls_coefb;
    aid_f4_j_rls = (AID_1MS_PERIOD * aid_st_jd.f4_rls_coefa * f4_elespeed_over_iq) / aid_st_jd.f4_rls_coefb;
#elif (AID_JD_PRV_RLS_MODE == 3)
    {
        float in[3];
        float coef[3];
        f4_respns = aid_st_jd.f4_speed_rad;
        in[0] = aid_st_jd.f4_rls_respns_pre;
        in[1] = aid_st_jd.f4_iq;
        in[2] = 1.0f;
        aidf_rls3_exec(&aid_st_jd.rls3,
                      in,
                      f4_respns,
                      coef);
        aid_st_jd.f4_rls_respns_pre = f4_respns;
        aid_st_jd.f4_rls_coefa = coef[0];
        aid_st_jd.f4_rls_coefb = coef[1];
        aid_f4_d_rls = (1.0f - aid_st_jd.f4_rls_coefa) * f4_elespeed_over_iq / aid_st_jd.f4_rls_coefb;
        aid_f4_j_rls = (AID_1MS_PERIOD * aid_st_jd.f4_rls_coefa * f4_elespeed_over_iq) / aid_st_jd.f4_rls_coefb;
    }
#else
    aidf_rls_exec(f4_linearized_iq, f4_respns, &aid_st_jd.f4_rls_coefa, &aid_st_jd.f4_rls_coefb);
    aid_f4_d_rls = ((1 + aid_st_jd.f4_rls_coefa) * f4_elespeed_over_iq) / aid_st_jd.f4_rls_coefb;
    aid_f4_j_rls = (-(aid_f4_spd_ctrl_period_ms * 0.001f * aid_st_jd.f4_rls_coefa * f4_elespeed_over_iq)) / aid_st_jd.f4_rls_coefb;
#endif
} /* End of function jd_estimate_jd_rls */

/***********************************************************************************************************************
* Function Name: jd_estimate_jd_dft
* Description  : Estimate inertia and friction coefficient by DFT method
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void jd_estimate_jd_dft(void)
{
    float f4_real;
    float f4_imag;
    float f4_elespeed_over_iq;

    f4_elespeed_over_iq = aid_f4_ke * aid_f4_pole_pairs * aid_f4_pole_pairs;
    aid_st_jd.f4_dft_cycle_cnt++;
    aidf_dftdiff_sum(aidf_sinref_get_angle(),
                    aid_st_jd.f4_iq,
                    aid_st_jd.f4_speed_rad);

    if (aid_st_jd.f4_dft_cycle_cnt > (aid_f4_spd_ctrl_freq_hz / aid_st_jd.f4_refspeed_freq_hz))
    {
        aid_st_jd.f4_dft_cycle_cnt -= (aid_f4_spd_ctrl_freq_hz / aid_st_jd.f4_refspeed_freq_hz);
        aidf_dftdiff_result_div(&f4_real, &f4_imag);
        aid_st_jd.f4_dft_j = (f4_imag * f4_elespeed_over_iq) / (AID_TWOPI * aid_st_jd.f4_refspeed_freq_hz) ;
        aid_st_jd.f4_dft_d = f4_real * f4_elespeed_over_iq;
    }
} /* End of function jd_estimate_jd_dft */

/******************************************************************************
* Function Name : jd_seq_init
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void jd_seq_init(void)
{
    float f4_max_speed;
    float f4_amp_coef;

    gs_f4_cfg_assumed_inertia = g_f4_aid_jd_assumed_inertia;

    /* This is duplicated with ones in KE, but is required if KE ID is skipped */
    aid_core_config_current_pi_gains(g_f4_aid_current_omega, g_f4_aid_current_zeta);
    aid_core_config_bemf_obsv(g_f4_aid_e_obs_omega, g_f4_aid_e_obs_zeta);

    f4_max_speed = aid_f4_vmag_max / aid_f4_ke;
    /* Speed Kp */
    jd_design_speed_pi(gs_f4_cfg_assumed_inertia);

    /* Change parameter of sensor-less start-up sequence */
    aid_st_jd.f4_ol2cl_speed = g_f4_aid_ol2cl_bemf_th_coef * f4_max_speed;
    aid_st_jd.f4_cl2ol_speed = g_f4_aid_cl2ol_bemf_th_coef * f4_max_speed;
    aid_core_set_startup_params(aid_f4_rated_current * g_f4_aid_jd_id_ref_coef,
                       aid_st_jd.f4_ol2cl_speed,
                       aid_st_jd.f4_cl2ol_speed);

    /* Set parameter of speed reference signal for identification */
    aid_st_jd.f4_refspeed_freq_hz = g_f4_aid_jd_rls_freq;
    f4_amp_coef = g_f4_aid_jd_speed_amp_coef_min +
                ((g_f4_aid_jd_speed_amp_coef - g_f4_aid_jd_speed_amp_coef_min) * (1.0f - gs_f4_cfg_inertia_range));
    aid_st_jd.f4_refspeed_amp = f4_max_speed * f4_amp_coef;
    aid_st_jd.f4_refspeed_offset = f4_max_speed * g_f4_aid_jd_speed_offset_coef;
    aid_st_jd.f4_refspeed_amp_max = (aid_st_jd.f4_refspeed_offset - aid_st_jd.f4_cl2ol_speed) * 0.95f;

    /* Initialize sine signal generator, DFT */
    aidf_sinref_init(aid_st_jd.f4_refspeed_freq_hz,
                     aid_st_jd.f4_refspeed_amp,
                     aid_st_jd.f4_refspeed_offset,
                     aid_f4_spd_ctrl_period_ms * 0.001f);
    aidf_dftdiff_init();
    gs_f4_res_wait_cnt = 0.0f;

    /* Initialization of RLS filter */
#if (AID_JD_PRV_RLS_MODE == 2)
    {
        float f4_temp;

        f4_temp = aid_f4_ke * (aid_f4_pole_pairs * aid_f4_pole_pairs);

        /* Assumed the variance of 'a' is less than 0.04 (20%^2), initial value should be 100 times of it */
        aidf_rls2_init(4.0f,
                      g_f4_aid_jd_rls_init,
                      g_f4_aid_jd_rls_forget_k,
                      0.999f,                                           /* Initial weight for last output term */
                      (f4_temp * AID_1MS_PERIOD) / g_f4_aid_jd_assumed_inertia);        /* Initial weight for Iq */
    }
#elif (AID_JD_PRV_RLS_MODE == 3)
    aidf_rls3_init(&aid_st_jd.rls3,
            g_f4_aid_jd_rls_init,
            g_f4_aid_jd_rls_init,
            g_f4_aid_jd_rls_init,
            g_f4_aid_jd_rls_forget_k,
            1.0f,
            0.0f,
            0.0f);
#else
    aidf_rls_init(g_f4_aid_jd_rls_init, g_f4_aid_jd_rls_forget_k);
#endif

    aid_st_jd.f4_j_min = 0.1E-6f;
    aid_st_jd.f4_j_max = 5.0E-3f;
    aid_st_jd.f4_j_pi_ctrl = gs_f4_cfg_assumed_inertia;

    aid_st_jd.u1_j_seq = JD_PRV_MEASURE_SEQ_KE;
    aid_st_jd.e_status = AID_SEQ_J_READY;
    aid_core_set_ctrl_level(AID_CTRL_LEVEL_4);
    aid_f4_limit_speed_change = g_f4_aid_speed_change_rate_limit;

    /* If motor is in closed-loop drive mode, do not reset the start-up sequence, go ahead */
    if (AID_ROTOR_ANGLE_MODE_OPENLOOP == aid_s4_rotor_angle_mode)
    {
        aid_core_reset_startup_seq();
    }
} /* End of function jd_seq_init */

/******************************************************************************
* Function Name : jd_seq_ready
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void jd_seq_ready(void)
{
    aid_f4_ref_speed_rad = aid_st_jd.f4_refspeed_offset;

    {
        float overshot_ratio;

        overshot_ratio = (aid_f4_speed_lpf_rad / aid_f4_ref_speed_rad_ctrl) - 1.0f;
        if ((aid_f4_ref_speed_rad_ctrl > 0.0f) && (overshot_ratio > dbg_f4_jd_max_overshot_ratio))
        {
             dbg_f4_jd_max_overshot_ratio = overshot_ratio;
        }
    }

    /* If the startup sequence failed, throw startup error */
    if ((aid_f4_speed_lpf_rad < 0.0f) && (AID_ROTOR_ANGLE_MODE_CLOSEDLOOP == aid_s4_rotor_angle_mode))
    {
        aid_core_throw_error(AID_ERROR_J_STARTUP);
        aid_st_jd.e_status = AID_SEQ_J_RESET;
    }

    if (aid_f4_ref_speed_rad_ctrl >= aid_st_jd.f4_refspeed_offset)
    {
        aidf_dftdiff_init();
        aid_st_jd.f4_dft_cycle_cnt = 0;
        aid_st_jd.f4_iq_lpf = aid_f4_iq_ad;
        aid_st_jd.e_status = AID_SEQ_J_MEASURE;
    }
} /* End of function jd_seq_ready */

/******************************************************************************
* Function Name : jd_seq_measure
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void jd_seq_measure(void)
{
    aid_u2_1ms_cnt++;
    aid_st_jd.f4_iq_sum += aid_f4_iq_ad;

    /* AID_CTRL_FREQ/AID_1MS_FREQ is float, convert to uint16_t */
    if (((uint16_t)(aid_f4_ctrl_freq_hz / aid_f4_spd_ctrl_freq_hz)) == aid_u2_1ms_cnt)
    {
        /* Iq LPF , in: iq_ad, out:jd_iq */
#if (JD_PRV_SPEED == JD_PRV_SPEED_USE_PLL)
        aid_st_jd.f4_speed_rad = aid_f4_speed_rad;
#elif (JD_PRV_SPEED == JD_PRV_SPEED_USE_1MS)
        /* Use the speed used on all 1ms processes, this value is a LPFed value */
        aid_st_jd.f4_speed_rad = aid_f4_1ms_speed_rad;
#endif

#if (JD_PRV_IQ == JD_PRV_IQ_USE_LPF)
        /* The LPF method, it is hard to determine a appropriate LPF bandwidth */
        aid_f4_jd_iq = aid_f4_jd_iq_z + 0.157f * (aid_f4_iq_ad - aid_f4_jd_iq_z);
        aid_f4_jd_iq_z = aid_f4_jd_iq;
#elif (JD_PRV_IQ == JD_PRV_IQ_USE_AVG)
        aid_st_jd.f4_iq = aid_st_jd.f4_iq_sum / aid_u2_1ms_cnt;
        aid_st_jd.f4_iq_sum = 0;
#endif
        aid_u2_1ms_cnt = 0;
        aid_st_jd.f4_time_elapsed += aid_f4_spd_ctrl_period_ms;

        if (aid_st_jd.u1_j_seq >= JD_PRV_MEASURE_SEQ_SETUP)
        {
            /* Generate sinusoidal speed command signal,
             * frequency of the signal should be low enough than the specified bandwidth of speed PI */
            aid_f4_ref_speed_rad = aidf_sinref_generate();

            jd_estimate_jd_rls();
            jd_estimate_jd_dft();
        }

        switch(aid_st_jd.u1_j_seq)
        {
            case JD_PRV_MEASURE_SEQ_KE:
            {
                if (aid_st_jd.f4_time_elapsed <= g_f4_aid_jd_stab_wait)
                {
                    /* Wait until the speed became stable (500msec)
                     * NOTE: Fixed waiting time may be insufficient
                     * when the actual inertia is far more larger than the assumed inertia
                     *  */
                    __asm __volatile("nop\n");
                }
                else
                {
                    aid_st_jd.u1_j_seq = JD_PRV_MEASURE_SEQ_SETUP;
                    aid_st_jd.f4_time_elapsed = 0.0f;
                    aid_st_jd.f4_iq_offset = aid_st_jd.f4_iq_lpf;
                    aid_st_jd.f4_rls_respns_pre = 0.0f;

                    /* Acceleration limit change */
                    aid_f4_limit_speed_change = g_f4_aid_jd_limit_speed_change;
                }
            }
            break;

            case JD_PRV_MEASURE_SEQ_SETUP:
            {
                /* Measure the iq amplitude by DFT every 3 periods, adjust speed PI base on the result
                 * Note: gs_f4_res_wait_cnt is counter based timer, OK */
                gs_f4_res_wait_cnt++;
                if (gs_f4_res_wait_cnt >= ((aid_f4_spd_ctrl_freq_hz / aid_st_jd.f4_refspeed_freq_hz) * 3))
                {
                    gs_f4_res_wait_cnt = 0;
                    aidf_dftdiff_get_result_amp(&aid_st_jd.f4_iq_amp, &aid_st_jd.f4_speed_amp);
                    jd_adjust_speed_pi(aid_st_jd.f4_iq_amp, aid_st_jd.f4_speed_amp);
                    aidf_dftdiff_init();             /* Clear DFT */
                }

                if (aid_st_jd.f4_time_elapsed >= ((1000.0f / aid_st_jd.f4_refspeed_freq_hz) * g_f4_aid_jd_dft_num))
                {
                    aid_st_jd.f4_time_elapsed = 0;
                    aid_u4_sample_cnt = 0;
                    aidf_dftdiff_init();
                    aid_st_jd.u1_j_seq = JD_PRV_MEASURE_SEQ_DFT;
                }
            }
            break;

            case JD_PRV_MEASURE_SEQ_RLS:
            {
                aid_u4_sample_cnt++;
                if (((aid_f4_spd_ctrl_freq_hz / aid_st_jd.f4_refspeed_freq_hz) * 50) <= (float)aid_u4_sample_cnt)
                {
                    aid_f4_d_rls = ((1 + aid_st_jd.f4_rls_coefa) * (aid_f4_ke * aid_f4_pole_pairs * aid_f4_pole_pairs)) / aid_st_jd.f4_rls_coefb;
                    aid_f4_j_rls = (-(aid_f4_spd_ctrl_period_ms * 0.001f * aid_st_jd.f4_rls_coefa * (aid_f4_ke * aid_f4_pole_pairs * aid_f4_pole_pairs))) / aid_st_jd.f4_rls_coefb;
                    aid_u4_sample_cnt = 0;
                    aid_st_jd.u1_j_seq = JD_PRV_MEASURE_SEQ_DFT;
                    aidf_dftdiff_init();
                }
            }
            break;

            case JD_PRV_MEASURE_SEQ_DFT:
            {
                aid_u4_sample_cnt++;

                if (((aid_f4_spd_ctrl_freq_hz * 50)/ aid_st_jd.f4_refspeed_freq_hz) <= (float)aid_u4_sample_cnt)
                {
                    aidf_dftdiff_result_div(&aid_f4_d_dft, &aid_f4_j_dft);
                    aid_f4_j_dft = (aid_f4_j_dft  / (AID_TWOPI * aid_st_jd.f4_refspeed_freq_hz)) * (aid_f4_ke * aid_f4_pole_pairs * aid_f4_pole_pairs);
                    aid_u4_sample_cnt = 0;
                    aid_st_jd.u1_j_seq = JD_PRV_MEASURE_SEQ_COMP;
                    aid_st_jd.e_status = AID_SEQ_J_CHECK;
                }
            }
            break;

            default:
            {
                AID_ASSERT_FAIL();
            }
            break;

        }
    }
} /* End of function jd_seq_measure */

/******************************************************************************
* Function Name : jd_seq_check
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void jd_seq_check(void)
{
    float f4_j;

    f4_j = aid_f4_j_dft;
    if ((f4_j > aid_st_jd.f4_j_max) || (f4_j <= aid_st_jd.f4_j_min))
    {
        aid_core_throw_error(AID_ERROR_J);
    }

    aid_st_jd.e_status = AID_SEQ_J_RESET;
} /* End of function jd_seq_check */

/******************************************************************************
* Function Name : jd_seq_reset
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void jd_seq_reset(void)
{
    /* Note: Slow down too fast may cause over-voltage */
    aid_f4_ref_speed_rad -= (aid_st_jd.f4_refspeed_offset * aid_f4_ctrl_period_ms / g_f4_aid_jd_stop_time);
    if (aid_f4_ref_speed_rad < 0)
    {
        aid_f4_ref_speed_rad = 0.0f;
        aid_st_jd.e_status = AID_SEQ_J_COMPLETED;
    }
} /* End of function jd_seq_reset */
