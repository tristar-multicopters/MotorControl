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
 * File Name    : r_aid_ldq.c
 * Version      : 1.0
 * Description  : Inductance identification module
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <math.h>
//#include <machine.h>
#include "r_aid_ldq.h"
#include "r_aid_config.h"
#include "r_aid_function.h"

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#ifndef AID_PRV_FILE_CODE
#define AID_PRV_FILE_CODE (104)
#endif
#define AID_PRV_LDQ_CTRL_LEVEL  (AID_CTRL_LEVEL_0)

/***********************************************************************************************************************
 Private global variables and functions
 **********************************************************************************************************************/
static void ldq_seq_rld_rls_init(st_aid_signal_conf_t *st_signal_conf);
static void ldq_seq_rld_rls_ready(void);
static void ldq_seq_rld_rls_measure(void);
static void ldq_seq_rld_rls_check(void);
static void ldq_seq_rld_rls_reset(void);
static void ldq_seq_rld_dft_init(st_aid_signal_conf_t *st_signal_conf);
static void ldq_seq_rld_dft_ready(void);
static void ldq_seq_rld_dft_measure(void);
static void ldq_seq_rld_dft_check(void);
static void ldq_seq_rld_dft_reset(void);
static void ldq_seq_lq_rls_init(st_aid_signal_conf_t *st_signal_conf);
static void ldq_seq_lq_rls_ready(void);
static void ldq_seq_lq_rls_measure(void);
static void ldq_seq_lq_rls_check(void);
static void ldq_seq_lq_rls_reset(void);
static void ldq_seq_lq_dft_init(st_aid_signal_conf_t *st_signal_conf);
static void ldq_seq_lq_dft_ready(void);
static void ldq_seq_lq_dft_measure(void);
static void ldq_seq_lq_dft_check(void);
static void ldq_seq_lq_dft_reset(void);

/* DFT & Recursive least squares filter */
static float gs_f4_ldq_freq_ref_hz;
static float gs_f4_time_elapsed_ms;
e_aid_seq_rld_rls_t aid_s4_ldrls_status;
e_aid_seq_rld_dft_t aid_s4_lddft_status;
e_aid_seq_rlq_rls_t aid_s4_lqrls_status;
e_aid_seq_rlq_dft_t aid_s4_lqdft_status;
float aid_f4_vd_ref_amp;
float aid_f4_vq_ref_amp;
float aid_f4_vd_ref_offset;
float  aid_f4_dft_real;
float  aid_f4_dft_imag;
float  aid_f4_dft_amp;
float  aid_f4_dft_amp2;
float  aid_f4_current_pre;
float  aid_f4_voltage_pre;
float  aid_f4_id_offset;
float  aid_f4_id_sum;
float  aid_f4_id_sum_cnt;
float  aid_f4_a_est;
float  aid_f4_b_est;
uint16_t aid_f4_dft_stab_wait_ms;
float aid_f4_angle_pre;

/***********************************************************************************************************************
 Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/
/* Debug */
float dbg_f4_lddft_v;
float dbg_f4_lddft_i_mag;
float dbg_f4_lddft_i_img;
float dbg_f4_lddft_i_real;

float dbg_f4_lqdft_v;
float dbg_f4_lqdft_i_mag;
float dbg_f4_lqdft_i_img;
float dbg_f4_lqdft_i_real;

float  aid_f4_r_dft;
float  aid_f4_r_rls;
float  aid_f4_ld_dft;
float  aid_f4_ld_rls;
float  aid_f4_lq_dft;
float  aid_f4_lq_rls;

/******************************************************************************
* Function Name : aid_rld_dft_act
* Description   :
* Arguments     : None
* Return Value  : Whether the process is done (1) or not (0)
******************************************************************************/
uint16_t aid_rld_dft_act(st_aid_signal_conf_t *st_signal_conf)
{
    uint16_t ret = 0;
    switch (aid_s4_lddft_status)
    {
        case AID_SEQ_RLD_DFT_INIT:
        {
            ldq_seq_rld_dft_init(st_signal_conf);
        }
        break;

        case AID_SEQ_RLD_DFT_READY:
        {
            ldq_seq_rld_dft_ready();
        }
        break;

        case AID_SEQ_RLD_DFT_MEASURE:
        {
            ldq_seq_rld_dft_measure();
        }
        break;

        case AID_SEQ_RLD_DFT_CHECK:
        {
            ldq_seq_rld_dft_check();
        }
        break;

        case AID_SEQ_RLD_DFT_RESET:
        {
            ldq_seq_rld_dft_reset();
        }
        break;

        case AID_SEQ_RLD_DFT_COMPLETED:
        {
            ret = 1;        /* Inform that the identification is completed */
        }
        break;

        default:
        {
            AID_ASSERT_FAIL();
        }
        break;
    }

    return ret;
} /* End of function aid_rld_dft_act */

/******************************************************************************
* Function Name : aid_rld_rls_act
* Description   :
* Arguments     : None
* Return Value  : Whether the process is done (1) or not (0)
******************************************************************************/
uint16_t aid_rld_rls_act(st_aid_signal_conf_t *st_signal_conf)
{
    uint16_t ret = 0;
    switch (aid_s4_ldrls_status)
    {
        case AID_SEQ_RLD_RLS_INIT:
        {
            ldq_seq_rld_rls_init(st_signal_conf);
        }
        break;

        case AID_SEQ_RLD_RLS_READY:
        {
            ldq_seq_rld_rls_ready();
        }
        break;

        case AID_SEQ_RLD_RLS_MEASURE:
        {
            ldq_seq_rld_rls_measure();
        }
        break;

        case AID_SEQ_RLD_RLS_CHECK:
        {
            ldq_seq_rld_rls_check();
        }
        break;

        case AID_SEQ_RLD_RLS_RESET:
        {
            ldq_seq_rld_rls_reset();
        }
        break;

        case AID_SEQ_RLD_RLS_COMPLETED:
        {
            ret = 1;        /* Inform that the identification is completed */
        }
        break;

        default:
        {
            AID_ASSERT_FAIL();
        }
        break;
    }

    return ret;
} /* End of function aid_rld_rls_act */

/******************************************************************************
* Function Name : aid_lq_dft_act
* Description   :
* Arguments     : None
* Return Value  : Whether the process is done (1) or not (0)
******************************************************************************/
uint16_t aid_lq_dft_act(st_aid_signal_conf_t *st_signal_conf)
{
    uint16_t ret = 0;

    switch (aid_s4_lqdft_status)
    {
        case AID_SEQ_RLQ_DFT_INIT:
        {
            ldq_seq_lq_dft_init(st_signal_conf);
        }
        break;

        case AID_SEQ_RLQ_DFT_READY:
        {
            ldq_seq_lq_dft_ready();
        }
        break;

        case AID_SEQ_RLQ_DFT_MEASURE:
        {
            ldq_seq_lq_dft_measure();
        }
        break;

        case AID_SEQ_RLQ_DFT_CHECK:
        {
            ldq_seq_lq_dft_check();
        }
        break;

        case AID_SEQ_RLQ_DFT_RESET:
        {
            ldq_seq_lq_dft_reset();
        }
        break;

        case AID_SEQ_RLQ_DFT_COMPLETED:
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
} /* End of function aid_lq_dft_act */

/******************************************************************************
* Function Name : aid_lq_rls_act
* Description   :
* Arguments     : None
* Return Value  : Whether the process is done (1) or not (0)
******************************************************************************/
uint16_t aid_lq_rls_act(st_aid_signal_conf_t *st_signal_conf)
{
    uint16_t ret = 0;

    switch (aid_s4_lqrls_status)
    {
        case AID_SEQ_RLQ_RLS_INIT:
        {
            ldq_seq_lq_rls_init(st_signal_conf);
        }
        break;

        case AID_SEQ_RLQ_RLS_READY:
        {
            ldq_seq_lq_rls_ready();
        }
        break;

        case AID_SEQ_RLQ_RLS_MEASURE:
        {
            ldq_seq_lq_rls_measure();
        }
        break;

        case AID_SEQ_RLQ_RLS_CHECK:
        {
            ldq_seq_lq_rls_check();
        }
        break;

        case AID_SEQ_RLQ_RLS_RESET:
        {
            ldq_seq_lq_rls_reset();
        }
        break;

        case AID_SEQ_RLQ_RLS_COMPLETED:
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
} /* End of function aid_lq_rls_act */

/***********************************************************************************************************************
 * Function Name: aid_ldq_reset
 * Description  : Initialize inductance identification
 * Arguments    : None
 * Return Value : None
 **********************************************************************************************************************/
void aid_ldq_reset(void)
{
    /* DFT & Recursive least squares filter */
    aid_s4_ldrls_status = AID_SEQ_RLD_RLS_INIT;
    aid_s4_lddft_status = AID_SEQ_RLD_DFT_INIT;
    aid_s4_lqrls_status = AID_SEQ_RLQ_RLS_INIT;
    aid_s4_lqdft_status = AID_SEQ_RLQ_DFT_INIT;

    aid_f4_vd_ref_amp = 0.0f;
    aid_f4_vq_ref_amp = 0.0f;
    aid_f4_vd_ref_offset = 0.0f;
    aid_f4_dft_real = 0.0f;
    aid_f4_dft_imag = 0.0f;
    aid_f4_dft_amp = 0.0f;
    aid_f4_dft_amp2 = 0.0f;
    aid_f4_current_pre = 0.0f;
    aid_f4_voltage_pre = 0.0f;
    aid_f4_id_offset = 0.0f;
    aid_f4_id_sum = 0.0f;
    aid_f4_id_sum_cnt = 0.0f;
    aid_f4_a_est = 0.0f;
    aid_f4_b_est = 0.0f;
    aid_f4_angle_pre = 0.0f;
    aid_f4_dft_stab_wait_ms = 0;
    gs_f4_time_elapsed_ms = 0.0f;
} /* End of function aid_ldq_reset */

/******************************************************************************
* Function Name : ldq_seq_rld_rls_init
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_rls_init(st_aid_signal_conf_t *st_signal_conf)
{
    if (AID_SIGNAL_CONF_DEFAULT == st_signal_conf)
    {
        /* Use default signal configuration */
        gs_f4_ldq_freq_ref_hz = g_f4_aid_rld_dft_freq;

        /* Set offset, use R to estimate current */
        aid_f4_vd_ref_offset = (aid_f4_r * (aid_f4_rated_current * g_f4_aid_rld_rls_vd_offset_coef)) + aid_f4_v_err;

        /* Set amplitude */
        aid_f4_vd_ref_amp = aid_f4_r * aid_f4_rated_current * g_f4_aid_rld_rls_vd_amp_coef;
    }
    else
    {
        gs_f4_ldq_freq_ref_hz   = st_signal_conf->f4_signal_freg_hz;
        aid_f4_vd_ref_offset    = st_signal_conf->f4_signal_offset;
        aid_f4_vd_ref_amp       = st_signal_conf->f4_signal_amp;
    }

    aidf_sinref_init(gs_f4_ldq_freq_ref_hz, aid_f4_vd_ref_amp, aid_f4_vd_ref_offset, aid_f4_ctrl_period_ms * 0.001f);
    aidf_rls_init(g_f4_aid_rld_rls_init, g_f4_aid_rld_rls_forget_k);

    aid_s4_ldrls_status = AID_SEQ_RLD_RLS_READY;
    aid_core_set_ctrl_level(AID_PRV_LDQ_CTRL_LEVEL);
    gs_f4_time_elapsed_ms = 0.0f;
} /* End of function ldq_seq_rld_rls_init */

/******************************************************************************
* Function Name : ldq_seq_rld_rls_ready
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_rls_ready(void)
{
    gs_f4_time_elapsed_ms += aid_f4_ctrl_period_ms;

    aid_f4_vd_ref = aid_f4_vd_ref_offset;
    if ((gs_f4_time_elapsed_ms > g_f4_aid_rld_rls_vd_dc_wait_time) &&
        (gs_f4_time_elapsed_ms <= (g_f4_aid_rld_rls_vd_dc_wait_time + g_f4_aid_rld_rls_measure_offset_time)))
    {
        /* Check the offset current, if the offset current is too small, correct the voltage error and repeat */
        if (aid_f4_id_ad >= (0.9f * (aid_f4_rated_current * g_f4_aid_rld_rls_vd_offset_coef)))
        {
            aid_f4_id_sum += aid_f4_id_ad;
            aid_f4_id_sum_cnt++;
        }
        else
        {
            aid_f4_v_err = aid_f4_vd_ref - (aid_f4_id_ad * aid_f4_r);
            aid_f4_vd_ref_offset = (aid_f4_r * (aid_f4_rated_current * g_f4_aid_rld_rls_vd_offset_coef)) + aid_f4_v_err;
            aidf_sinref_init(gs_f4_ldq_freq_ref_hz, aid_f4_vd_ref_amp, aid_f4_vd_ref_offset, aid_f4_ctrl_period_ms * 0.001f);
            aid_f4_vd_ref = aid_f4_vd_ref_offset;
            gs_f4_time_elapsed_ms = 0.0f;
            aid_f4_id_sum = 0.0f;
            aid_f4_id_sum_cnt = 0.0f;
        }
    }
    else if ((gs_f4_time_elapsed_ms > (g_f4_aid_rld_rls_vd_dc_wait_time + g_f4_aid_rld_rls_measure_offset_time)) &&
             (gs_f4_time_elapsed_ms <= (g_f4_aid_rld_rls_vd_dc_wait_time + g_f4_aid_rld_rls_measure_offset_time + g_f4_aid_rld_rls_vd_sin_wait_time)))
    {
        aid_f4_vd_ref = aidf_sinref_generate();
    }
    else if (gs_f4_time_elapsed_ms > (g_f4_aid_rld_rls_vd_dc_wait_time + g_f4_aid_rld_rls_measure_offset_time + g_f4_aid_rld_rls_vd_sin_wait_time))
    {
        aid_f4_id_offset = aid_f4_id_sum / aid_f4_id_sum_cnt;
        aid_f4_id_sum = 0.0f;
        aid_f4_id_sum_cnt = 0.0f;
        gs_f4_time_elapsed_ms = 0;
        aid_s4_ldrls_status = AID_SEQ_RLD_RLS_MEASURE;
    }
    else
    {
        /* Do nothing */
        __asm __volatile("nop\n");
    }
} /* End of function ldq_seq_rld_rls_ready */

uint32_t wLdqDebugTemp = 0;
uint8_t bLdqDebugTemp = 0;

/******************************************************************************
* Function Name : ldq_seq_rld_rls_measure
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_rls_measure(void)
{
    float f4_u;
    float f4_y;
    
    aid_u2_sum_cnt++;

    f4_u = aid_f4_vd_ref - aid_f4_vd_ref_offset;
    f4_y = aid_f4_id_ad - aid_f4_id_offset;
    aidf_rls_exec(f4_u, f4_y, &aid_f4_a_est, &aid_f4_b_est);

    aid_f4_vd_ref = aidf_sinref_generate();

    if (aid_u2_sum_cnt >= ((aid_f4_ctrl_freq_hz / gs_f4_ldq_freq_ref_hz) * g_f4_aid_rld_rls_measure_num))
    {
        aid_f4_r_rls = (1 + aid_f4_a_est) / aid_f4_b_est;
        aid_f4_ld_rls = (-(aid_f4_ctrl_period_ms * 0.001f * aid_f4_a_est)) / aid_f4_b_est;
        aid_u2_sum_cnt = 0;
        aid_s4_ldrls_status = AID_SEQ_RLD_RLS_CHECK;
    }
} /* End of function ldq_seq_rld_rls_measure */

/******************************************************************************
* Function Name : ldq_seq_rld_rls_check
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_rls_check(void)
{
    if (aid_f4_r_rls > aid_f4_r_max)
    {
        aid_core_throw_error(AID_ERROR_R_RLS);
    }

    if (aid_f4_r_rls <= aid_f4_r_min)
    {
        aid_core_throw_error(AID_ERROR_R_RLS);
    }

    if (aid_f4_ld_rls <= aid_f4_ld_min)
    {
        aid_core_throw_error(AID_ERROR_LD_RLS);
    }

    aid_s4_ldrls_status = AID_SEQ_RLD_RLS_RESET;
} /* End of function ldq_seq_rld_rls_check */

/******************************************************************************
* Function Name : ldq_seq_rld_rls_reset
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_rls_reset(void)
{
    gs_f4_time_elapsed_ms += aid_f4_ctrl_period_ms;
    if (gs_f4_time_elapsed_ms > g_f4_aid_rld_rls_reset_time)
    {
        gs_f4_time_elapsed_ms = 0;
        aid_s4_ldrls_status = AID_SEQ_RLD_RLS_COMPLETED;
    }
} /* End of function ldq_seq_rld_rls_reset */

/******************************************************************************
* Function Name : ldq_seq_rld_dft_init
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_dft_init(st_aid_signal_conf_t *st_signal_conf)
{
    if (AID_SIGNAL_CONF_DEFAULT == st_signal_conf)
    {
        /* Use default signal configuration */
        gs_f4_ldq_freq_ref_hz = g_f4_aid_rld_dft_freq;

        /* Set offset, use R to estimate current */
        aid_f4_vd_ref_offset = (aid_f4_r * aid_f4_rated_current * g_f4_aid_rld_dft_vd_offset_coef) + aid_f4_v_err;

        /* Set amplitude */
        aid_f4_vd_ref_amp = aid_f4_r * aid_f4_rated_current * g_f4_aid_rld_dft_vd_amp_coef;
    }
    else
    {
        gs_f4_ldq_freq_ref_hz   = st_signal_conf->f4_signal_freg_hz;
        aid_f4_vd_ref_offset    = st_signal_conf->f4_signal_offset;
        aid_f4_vd_ref_amp       = st_signal_conf->f4_signal_amp;
    }

    /* Init sinref */
    aidf_sinref_init(gs_f4_ldq_freq_ref_hz, aid_f4_vd_ref_amp, aid_f4_vd_ref_offset, aid_f4_ctrl_period_ms * 0.001f);

    /* Init DFT*/
    aidf_dft_init();

    aid_f4_dft_stab_wait_ms = (uint16_t)((aid_f4_ld * 1000.0f * g_f4_aid_rld_dft_stab_coef) / aid_f4_r);

    gs_f4_time_elapsed_ms = 0.0f;
    aid_s4_lddft_status = AID_SEQ_RLD_DFT_READY;
    aid_core_set_ctrl_level(AID_PRV_LDQ_CTRL_LEVEL);
} /* End of function ldq_seq_rld_dft_init */

/******************************************************************************
* Function Name : ldq_seq_rld_dft_ready
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_dft_ready(void)
{
    gs_f4_time_elapsed_ms += aid_f4_ctrl_period_ms;
    aid_f4_vd_ref = aid_f4_rated_current * 0.5f * aid_f4_r_dc;
    if ((gs_f4_time_elapsed_ms > (aid_f4_dft_stab_wait_ms * 0.5f)) && (gs_f4_time_elapsed_ms <= aid_f4_dft_stab_wait_ms))
    {
        aid_f4_vd_ref -= (aid_f4_rated_current * 0.005f * aid_f4_r_dc);
        if (aid_f4_vd_ref < 0.0f)
        {
            aid_f4_vd_ref = 0.0f;
        }
    }
    else if ((gs_f4_time_elapsed_ms > aid_f4_dft_stab_wait_ms) && (gs_f4_time_elapsed_ms <= (aid_f4_dft_stab_wait_ms * 2)))
    {
        aid_f4_vd_ref = aidf_sinref_generate();
    }
    else if (gs_f4_time_elapsed_ms > (aid_f4_dft_stab_wait_ms * 2))
    {
        gs_f4_time_elapsed_ms = 0;
        aid_s4_lddft_status = AID_SEQ_RLD_DFT_MEASURE;
    }
    else
    {
        /* Do nothing */
        __asm __volatile("nop\n");
    }
} /* End of function ldq_seq_rld_dft_ready */

/******************************************************************************
* Function Name : ldq_seq_rld_dft_measure
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_dft_measure(void)
{
    aid_u2_sum_cnt++;
    aid_f4_angle_pre = aidf_sinref_get_angle();
    aid_f4_vd_ref = aidf_sinref_generate();
    
    aidf_dft_sum(aid_f4_angle_pre, aid_f4_id_ad);

    if (aid_u2_sum_cnt >= ((aid_f4_ctrl_freq_hz / gs_f4_ldq_freq_ref_hz) * g_f4_aid_rld_dft_measure_num))
    {
        aidf_dft_result(&aid_f4_dft_real, &aid_f4_dft_imag);
        aid_f4_dft_amp2 = (aid_f4_dft_real * aid_f4_dft_real) + (aid_f4_dft_imag * aid_f4_dft_imag);
        aid_f4_ld_dft = ((aid_f4_vd_ref_amp * aid_f4_dft_imag) / aid_f4_dft_amp2) / (gs_f4_ldq_freq_ref_hz * AID_TWOPI);
        aid_f4_r_dft = (aid_f4_vd_ref_amp * aid_f4_dft_real) / aid_f4_dft_amp2;
        aid_u2_sum_cnt = 0;
        aid_s4_lddft_status = AID_SEQ_RLD_DFT_CHECK;

        dbg_f4_lddft_v = aid_f4_vd_ref_amp;
        dbg_f4_lddft_i_mag = sqrtf(aid_f4_dft_amp2);
        dbg_f4_lddft_i_img = aid_f4_dft_imag;
        dbg_f4_lddft_i_real = aid_f4_dft_real;
    }
    
} /* End of function ldq_seq_rld_dft_measure */

/******************************************************************************
* Function Name : ldq_seq_rld_dft_check
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_dft_check(void)
{
    if (aid_f4_r_dft > aid_f4_r_max)
    {
        aid_core_throw_error(AID_ERROR_R_DFT);
    }

    if (aid_f4_r_dft <= aid_f4_r_min)
    {
        aid_core_throw_error(AID_ERROR_R_DFT);
    }

    if (aid_f4_ld_dft <= aid_f4_ld_min)
    {
        aid_core_throw_error(AID_ERROR_LD_DFT);
    }

    aid_s4_lddft_status = AID_SEQ_RLD_DFT_RESET;
} /* End of function ldq_seq_rld_dft_check */

/******************************************************************************
* Function Name : ldq_seq_rld_dft_reset
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_rld_dft_reset(void)
{
    gs_f4_time_elapsed_ms += aid_f4_ctrl_period_ms;
    if (gs_f4_time_elapsed_ms > g_f4_aid_rld_dft_reset_time)
    {
        gs_f4_time_elapsed_ms = 0;
        aid_s4_lddft_status = AID_SEQ_RLD_DFT_COMPLETED;
    }
} /* End of function ldq_seq_rld_dft_reset */

/******************************************************************************
* Function Name : ldq_seq_lq_rls_init
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_rls_init(st_aid_signal_conf_t *st_signal_conf)
{
    float f4_temp;
    float f4_inpedence;

    if (AID_SIGNAL_CONF_DEFAULT == st_signal_conf)
    {
        /* Set frequency */
        gs_f4_ldq_freq_ref_hz = g_f4_aid_lq_rls_freq;

        /* Set offset, use R to estimate current */
        aid_f4_vd_ref_offset = (aid_f4_r * aid_f4_rated_current * g_f4_aid_lq_rls_vd_offset_coef) + aid_f4_v_err;

        /* Set amplitude */
        f4_temp = gs_f4_ldq_freq_ref_hz * aid_f4_ld * AID_TWOPI;
        f4_inpedence = sqrtf((aid_f4_r * aid_f4_r) + (f4_temp * f4_temp));
        aid_f4_vq_ref_amp = f4_inpedence * aid_f4_rated_current * g_f4_aid_lq_rls_vq_amp_coef;
    }
    else
    {
        gs_f4_ldq_freq_ref_hz            = st_signal_conf->f4_signal_freg_hz;
        aid_f4_vd_ref_offset    = st_signal_conf->f4_signal_offset;
        aid_f4_vd_ref_amp       = st_signal_conf->f4_signal_amp;
    }

    /* Initialize sine waveform generator */
    aidf_sinref_init(gs_f4_ldq_freq_ref_hz, aid_f4_vq_ref_amp, 0, aid_f4_ctrl_period_ms * 0.001f);

    /* Initialize RLS */
    aidf_rls_init(g_f4_aid_lq_rls_init, g_f4_aid_lq_rls_forget_k);

    gs_f4_time_elapsed_ms = 0.0f;
    aid_s4_lqrls_status = AID_SEQ_RLQ_RLS_READY;
    aid_core_set_ctrl_level(AID_PRV_LDQ_CTRL_LEVEL);
} /* End of function ldq_seq_lq_rls_init */

/******************************************************************************
* Function Name : ldq_seq_lq_rls_ready
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_rls_ready(void)
{
    gs_f4_time_elapsed_ms += aid_f4_ctrl_period_ms;
    aid_f4_vd_ref = aid_f4_vd_ref_offset;
    if ((gs_f4_time_elapsed_ms > g_f4_aid_lq_rls_vd_dc_wait_time) &&
            (gs_f4_time_elapsed_ms <= (g_f4_aid_lq_rls_vd_dc_wait_time + g_f4_aid_lq_rls_vq_sin_wait_time)))
    {
        aid_f4_vq_ref = aidf_sinref_generate();
    }
    else if (gs_f4_time_elapsed_ms > (g_f4_aid_lq_rls_vd_dc_wait_time + g_f4_aid_lq_rls_vq_sin_wait_time))
    {
        gs_f4_time_elapsed_ms = 0;
        aid_s4_lqrls_status = AID_SEQ_RLQ_RLS_MEASURE;
    }
    else
    {
        /* Do nothing */
        __asm __volatile("nop\n");
    }
} /* End of function ldq_seq_lq_rls_ready */

/******************************************************************************
* Function Name : ldq_seq_lq_rls_measure
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_rls_measure(void)
{
    float f4_u;
    float f4_y;

    aid_u2_sum_cnt++;

    /* RLS filter */
    f4_u = aid_f4_vq_ref;
    f4_y = aid_f4_iq_ad;
    aidf_rls_exec(f4_u, f4_y, &aid_f4_a_est, &aid_f4_b_est);

    aid_f4_vq_ref = aidf_sinref_generate();

    if (aid_u2_sum_cnt >= ((aid_f4_ctrl_freq_hz * g_f4_aid_lq_rls_measure_num) / gs_f4_ldq_freq_ref_hz))
    {
        aid_f4_lq_rls = (-(aid_f4_ctrl_period_ms * 0.001f * aid_f4_a_est)) / aid_f4_b_est;
        aid_u2_sum_cnt = 0;
        aid_f4_vq_ref = 0.0f;
        aid_s4_lqrls_status = AID_SEQ_RLQ_RLS_CHECK;
    }
} /* End of function ldq_seq_lq_rls_measure */

/******************************************************************************
* Function Name : ldq_seq_lq_rls_check
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_rls_check(void)
{
    if (aid_f4_lq_rls <= aid_f4_lq_min)
    {
        aid_core_throw_error(AID_ERROR_LQ_RLS);
    }

    if (aid_f4_ld > aid_f4_lq_rls)
    {
        aid_f4_lq = aid_f4_ld;
    }
    else
    {
        aid_f4_lq = aid_f4_lq_rls;
    }

    aid_s4_lqrls_status = AID_SEQ_RLQ_RLS_RESET;
} /* End of function ldq_seq_lq_rls_check */

/******************************************************************************
* Function Name : ldq_seq_lq_rls_reset
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_rls_reset(void)
{
    gs_f4_time_elapsed_ms += aid_f4_ctrl_period_ms;
    if (gs_f4_time_elapsed_ms > g_f4_aid_lq_rls_reset_time)
    {
        gs_f4_time_elapsed_ms = 0;
        aid_s4_lqrls_status = AID_SEQ_RLQ_RLS_COMPLETED;
    }
} /* End of function ldq_seq_lq_rls_reset */

/******************************************************************************
* Function Name : ldq_seq_lq_dft_init
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_dft_init(st_aid_signal_conf_t *st_signal_conf)
{
    float f4_temp;
    float f4_inpedence;

    if (AID_SIGNAL_CONF_DEFAULT == st_signal_conf)
    {
        /* Set frequency */
        gs_f4_ldq_freq_ref_hz = g_f4_aid_lq_dft_freq;

        /* Set offset, use R to estimate current */
        aid_f4_vd_ref_offset = (aid_f4_r * aid_f4_rated_current * g_f4_aid_lq_dft_vd_offset_coef) + aid_f4_v_err;

        /* Set amplitude */
        f4_temp = gs_f4_ldq_freq_ref_hz * aid_f4_ld * AID_TWOPI;
        f4_inpedence = sqrtf((aid_f4_r * aid_f4_r) + (f4_temp * f4_temp));
        aid_f4_vq_ref_amp = f4_inpedence * aid_f4_rated_current * g_f4_aid_lq_dft_vq_amp_coef;
    }
    else
    {
        gs_f4_ldq_freq_ref_hz   = st_signal_conf->f4_signal_freg_hz;
        aid_f4_vd_ref_offset    = st_signal_conf->f4_signal_offset;
        aid_f4_vq_ref_amp       = st_signal_conf->f4_signal_amp;
    }


    /* init sinref */
    aidf_sinref_init(gs_f4_ldq_freq_ref_hz, aid_f4_vq_ref_amp, 0, aid_f4_ctrl_period_ms * 0.001f);

    /* Init DFT */
    aidf_dft_init();

    aid_f4_dft_stab_wait_ms = (uint16_t)((aid_f4_lq * 1000.0f * g_f4_aid_rld_dft_stab_coef) / aid_f4_r);

    gs_f4_time_elapsed_ms = 0.0f;
    aid_s4_lqdft_status = AID_SEQ_RLQ_DFT_READY;
    aid_core_set_ctrl_level(AID_PRV_LDQ_CTRL_LEVEL);
} /* End of function ldq_seq_lq_dft_init */

/******************************************************************************
* Function Name : ldq_seq_lq_dft_ready
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_dft_ready(void)
{
    gs_f4_time_elapsed_ms += aid_f4_ctrl_period_ms;

    aid_f4_vd_ref = aid_f4_vd_ref_offset;
    if ((gs_f4_time_elapsed_ms > aid_f4_dft_stab_wait_ms) && (gs_f4_time_elapsed_ms <= (aid_f4_dft_stab_wait_ms * 2)))
    {
        aid_f4_vq_ref = aidf_sinref_generate();
    }
    else if (gs_f4_time_elapsed_ms > (aid_f4_dft_stab_wait_ms * 2))
    {
        gs_f4_time_elapsed_ms = 0;
        aid_s4_lqdft_status = AID_SEQ_RLQ_DFT_MEASURE;
    }
    else
    {
        /* Do nothing */
        __asm __volatile("nop\n");
    }
} /* End of function ldq_seq_lq_dft_ready */

/******************************************************************************
* Function Name : ldq_seq_lq_dft_measure
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_dft_measure(void)
{
    aid_u2_sum_cnt++;
    aid_f4_angle_pre = aidf_sinref_get_angle();
    aid_f4_vq_ref = aidf_sinref_generate();

    aidf_dft_sum(aid_f4_angle_pre, aid_f4_iq_ad);

    /* Sample an integer multiple number of samples to improve accuracy of DFT */
    if (aid_u2_sum_cnt >= ((aid_f4_ctrl_freq_hz / gs_f4_ldq_freq_ref_hz) * g_f4_aid_lq_dft_measure_num))
    {
        aidf_dft_result(&aid_f4_dft_real, &aid_f4_dft_imag);
        aid_f4_dft_amp2 = (aid_f4_dft_real * aid_f4_dft_real) + (aid_f4_dft_imag * aid_f4_dft_imag);
        aid_f4_dft_amp = sqrtf(aid_f4_dft_amp2);
        aid_f4_lq_dft = ((aid_f4_vq_ref_amp * aid_f4_dft_imag) / aid_f4_dft_amp2) / (gs_f4_ldq_freq_ref_hz * AID_TWOPI);
        aid_u2_sum_cnt = 0;
        aid_s4_lqdft_status = AID_SEQ_RLQ_DFT_CHECK;

        dbg_f4_lqdft_v = aid_f4_vq_ref_amp;
        dbg_f4_lqdft_i_mag = aid_f4_dft_amp;
        dbg_f4_lqdft_i_img = aid_f4_dft_imag;
        dbg_f4_lqdft_i_real = aid_f4_dft_real;
    }
} /* End of function ldq_seq_lq_dft_measure */

/******************************************************************************
* Function Name : ldq_seq_lq_dft_check
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_dft_check(void)
{
    if (aid_f4_lq_dft <= aid_f4_lq_min)
    {
        aid_core_throw_error(AID_ERROR_LQ_DFT);
    }

    aid_s4_lqdft_status = AID_SEQ_RLQ_DFT_RESET;
} /* End of function ldq_seq_lq_dft_check */

/******************************************************************************
* Function Name : ldq_seq_lq_dft_reset
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ldq_seq_lq_dft_reset(void)
{
    gs_f4_time_elapsed_ms += aid_f4_ctrl_period_ms;

    aid_f4_vd_ref = 0.0f;
    aid_f4_vq_ref = 0.0f;
    aid_f4_angle_pre = 0.0f;
    if (gs_f4_time_elapsed_ms > g_f4_aid_lq_dft_reset_time)
    {
        gs_f4_time_elapsed_ms = 0;
        aid_s4_lqdft_status = AID_SEQ_RLQ_DFT_COMPLETED;
    }
    else
    {
        /* Do nothing */
        __asm __volatile("nop\n");
    }
} /* End of function ldq_seq_lq_dft_reset */
