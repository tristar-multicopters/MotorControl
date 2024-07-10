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
 * File Name    : r_aid_volterr.c
 * Version      : 1.0
 * Description  : This module solves all the world's problems
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
#include "r_aid_volterr.h"
#include "r_aid_core.h"
#include "r_aid_function.h"

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#define AID_VOLTERR_PRV_LINEREG_SIZE_TAB        (2)
#define AID_VOLTERR_PRV_LINEREG_TAB_NUM         (AID_VOLTERR_PRV_LINEREG_SIZE_TAB * 2 + 1)
#define AID_VOLTERR_PRV_START_CURRENT_LSB       (-16)
#ifndef AID_PRV_FILE_CODE
#define AID_PRV_FILE_CODE (102)
#endif

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Private global variables and functions
 **********************************************************************************************************************/
static const float gs_f4_output_v_table[5] =
{
    0.409f,
    0.696f,
    0.878f,
    0.970f,
    1.0f
};

static void volterr_seq_init(void);
static void volterr_seq_id_r(void);
static void volterr_seq_id_verr_over_i(void);
static void volterr_seq_calc(void);
static void volterr_seq_check(void);
static void volterr_seq_reset(void);

static e_volterr_cfg_rid_t gs_e_cft_rid = AID_VOLTERR_RID_RLS;
static float gs_f4_r_init = 0.5f;
static float gs_f4_i_lsb = 0.01f;
static float gs_f4_current_step_ilsb;
static int32_t gs_s4_start_current_lsb = AID_VOLTERR_PRV_START_CURRENT_LSB;
st_aid_volterr_t aid_st_volterr;

/***********************************************************************************************************************
 Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/
float aid_f4_volterr_current_tab[AID_VOLTERR_OUTPUT_TAB_SIZE];
float aid_f4_volterr_voltage_tab[AID_VOLTERR_OUTPUT_TAB_SIZE];
float aid_f4_volterr_vdc_ref;
float aid_f4_volterr_rdc;

/***********************************************************************************************************************
* Function Name : aid_volterr_init
* Description   : Init
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void aid_volterr_init(void)
{
    gs_f4_current_step_ilsb = g_f4_aid_volterr_min_current_step_lsb;
} /* End of function aid_volterr_reset */

/******************************************************************************
* Function Name : aid_volterr_act
* Description   : Voltage error identification top layer sequence
* Arguments     : None
* Return Value  : Whether the process is done (1) or not (0)
******************************************************************************/
uint16_t aid_volterr_act(void)
{
    uint16_t ret = 0;
    switch (aid_st_volterr.e_status)
    {
        case AID_VOLTERR_SEQ_INIT:
        {
            volterr_seq_init();
        }
        break;

        case AID_VOLTERR_SEQ_ID_R:
        {
            volterr_seq_id_r();
        }
        break;

        case AID_VOLTERR_SEQ_ID_V_OVER_I:
        {
            volterr_seq_id_verr_over_i();
        }
        break;

        case AID_VOLTERR_SEQ_CALC:
        {
            volterr_seq_calc();
        }
        break;

        case AID_VOLTERR_SEQ_CHECK:
        {
            volterr_seq_check();
        }
        break;

        case AID_VOLTERR_SEQ_RESET:
        {
            volterr_seq_reset();
        }
        break;

        case AID_VOLTERR_SEQ_COMPLETED:
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
} /* End of function aid_volterr_act */

/***********************************************************************************************************************
* Function Name : aid_volterr_reset
* Description   : Reset voltage error identification
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void aid_volterr_reset(void)
{
    aid_st_volterr.e_status = AID_VOLTERR_SEQ_INIT;
} /* End of function aid_volterr_reset */

/***********************************************************************************************************************
* Function Name : aid_volterr_config
* Description   : Configure basic settings
* Arguments     :
* Return Value  : None
***********************************************************************************************************************/
void aid_volterr_config(float f4_r_initval, float f4_i_lsb, e_volterr_cfg_rid_t e_rid_cfg)
{
    gs_f4_r_init = f4_r_initval;
#if (AID_MOTOR_TYPE_BLDC)
    /* Considering scaling from phase current to dq current */
    gs_f4_i_lsb  = f4_i_lsb * AID_SQRT_3;
#else
    gs_f4_i_lsb  = f4_i_lsb;
#endif
    gs_e_cft_rid = e_rid_cfg;
} /* End of function aid_volterr_config */

/***********************************************************************************************************************
* Function Name : aid_volterr_config_current_step
* Description   : Set current step multiplier (integer, at least g_f4_aid_volterr_min_current_step_lsb)
* Arguments     :
* Return Value  : None
***********************************************************************************************************************/
void aid_volterr_config_current_step(uint16_t u2_ilsb)
{
    if (u2_ilsb < g_f4_aid_volterr_min_current_step_lsb)
    {
        gs_f4_current_step_ilsb = g_f4_aid_volterr_min_current_step_lsb;
    }
    else
    {
        /* Integer to float */
        gs_f4_current_step_ilsb = (float)u2_ilsb;
    }
} /* End of function aid_volterr_config_current_step */

/******************************************************************************
* Function Name : volterr_seq_init
* Description   : Initializes parameters
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void volterr_seq_init(void)
{
    memset(aid_st_volterr.f4_current_table, 0, sizeof(aid_st_volterr.f4_current_table));
    memset(aid_st_volterr.f4_voltage_table, 0, sizeof(aid_st_volterr.f4_voltage_table));
    memset(aid_st_volterr.f4_slope_table, 0, sizeof(aid_st_volterr.f4_slope_table));
    memset(aid_st_volterr.f4_intercept_table, 0, sizeof(aid_st_volterr.f4_intercept_table));
    aid_st_volterr.u4_count = 0;
    aid_st_volterr.u4_meas_index = 0;
    aid_st_volterr.f4_meas_volt_sum = 0.0f;
    aid_st_volterr.f4_meas_current_sum = 0.0f;
    aid_st_volterr.f4_sat_current = 0.0f;
    aid_st_volterr.f4_sat_voltage = 0.0f;
    aid_st_volterr.f4_current_target = 0.0f;
    aid_st_volterr.f4_r_dc = gs_f4_r_init;
    aid_st_volterr.f4_voltage_step = (aid_st_volterr.f4_r_dc * gs_f4_i_lsb * gs_f4_current_step_ilsb);
    aidf_rls2_init(g_f4_aid_rld_rls_init, g_f4_aid_rld_rls_init, g_f4_aid_volterr_rls_fgt_factor, aid_st_volterr.f4_r_dc, 0.0f);

    if (AID_VOLTERR_RID_IGNORE == gs_e_cft_rid)
    {
        aid_st_volterr.e_status = AID_VOLTERR_SEQ_ID_V_OVER_I;
    }
    else
    {
        aid_st_volterr.e_status = AID_VOLTERR_SEQ_ID_R;
    }

    aid_core_set_ctrl_level(AID_CTRL_LEVEL_0);
    aid_f4_vd_ref =  aid_st_volterr.f4_voltage_step;

#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
    /* Inject current with 30 degree will eliminate current through phase W (presented by ic_ad) */
    aid_f4_angle_rad = (AID_TWOPI * (-0.3333333f * 0.25f));
#endif
} /* End of function volterr_seq_init */

/******************************************************************************
* Function Name : volterr_seq_id_r
* Description   : Estimate maximum voltage error and precisive resistance by RLS algorithm
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void volterr_seq_id_r(void)
{
    float f4_v_mean;
    float f4_i_mean;
    float f4_vd_ref;
    float f4_id;
    float f4_verr_est;

    /* Input signals */
    f4_vd_ref = aid_f4_vd_ref;
    f4_id = aid_f4_ia_ad * 2.0f;

    aid_st_volterr.u4_count++;

    if (aid_st_volterr.u4_count >= ((uint32_t)g_f4_aid_volterr_avg_sample_fast + (uint32_t)(aid_f4_ctrl_freq_hz * 0.001f * g_f4_aid_volterr_avg_wait_time)))
    {
        f4_i_mean = aid_st_volterr.f4_meas_current_sum / g_f4_aid_volterr_avg_sample_fast;
        f4_v_mean = aid_st_volterr.f4_meas_volt_sum / g_f4_aid_volterr_avg_sample_fast;
        aid_st_volterr.u4_count = 0;
        aid_st_volterr.f4_meas_current_sum = 0.0f;
        aid_st_volterr.f4_meas_volt_sum = 0.0f;

        aidf_rls2_exec(f4_i_mean, 1.0f, f4_v_mean, &aid_st_volterr.f4_r_dc, &f4_verr_est);

        /* When current reaches the specified boundary */
        if (((aid_st_volterr.f4_voltage_step > 0.0f) &&
           (f4_i_mean > (aid_f4_rated_current * g_f4_aid_volterr_rls_upper_current))) ||
           ((aid_st_volterr.f4_voltage_step < 0.0f) &&
           (f4_i_mean < (aid_f4_rated_current * g_f4_aid_volterr_rls_lower_current))))
        {
            aid_st_volterr.u4_iteration_cnt++;

            /* Update magnitude of voltage step with latest R */
            aid_st_volterr.f4_voltage_step = (aid_st_volterr.f4_r_dc * gs_f4_i_lsb * gs_f4_current_step_ilsb);

            /* Reverse direction of voltage step */
            if (aid_st_volterr.u4_iteration_cnt & 0x01)
            {
                aid_st_volterr.f4_voltage_step = -aid_st_volterr.f4_voltage_step;
            }

            /* If all test is ended */
            if (aid_st_volterr.u4_iteration_cnt > g_u4_aid_volterr_iteration_num)
            {
                aid_st_volterr.e_status = AID_VOLTERR_SEQ_ID_V_OVER_I;
                aid_st_volterr.f4_voltage_step = (aid_st_volterr.f4_r_dc * gs_f4_i_lsb * gs_f4_current_step_ilsb);
                aid_st_volterr.f4_current_target = gs_f4_i_lsb *
                                                  (gs_f4_current_step_ilsb * (float)gs_s4_start_current_lsb);
                f4_vd_ref = aid_st_volterr.f4_r_dc * aid_st_volterr.f4_current_target;
                aid_st_volterr.u4_iteration_cnt = 0;
            }
        }
        else
        {
            f4_vd_ref += aid_st_volterr.f4_voltage_step;
        }
    }
    else if (aid_st_volterr.u4_count >= (uint32_t)(aid_f4_ctrl_freq_hz * 0.001f * g_f4_aid_volterr_avg_wait_time))
    {
        aid_st_volterr.f4_meas_current_sum += f4_id;
        aid_st_volterr.f4_meas_volt_sum += aid_f4_va_ref_pwm;
    }
    else
    {
        /* Do nothing */
        __asm __volatile("nop\n");
    }


    /* Output signals */
    aid_f4_vd_ref = f4_vd_ref;
} /* End of function volterr_seq_id_r */

/******************************************************************************
* Function Name : volterr_seq_id_verr_over_i
* Description   : Measure the voltage error at each current measurement point
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void volterr_seq_id_verr_over_i(void)
{
    float f4_v_mean;
    float f4_i_mean;
    float f4_vd_ref;
    float f4_id;

    /* Input signals */
    f4_vd_ref = aid_f4_vd_ref;
    f4_id = aid_f4_id_ad;


    aid_st_volterr.u4_count++;
    if (aid_st_volterr.u4_count >=
       ((uint32_t)(g_f4_aid_volterr_avg_sample) + (uint32_t)(aid_f4_ctrl_freq_hz * 0.001f * g_f4_aid_volterr_avg_wait_time)))
    {
        f4_i_mean = aid_st_volterr.f4_meas_current_sum / g_f4_aid_volterr_avg_sample;
        f4_v_mean = aid_st_volterr.f4_meas_volt_sum / g_f4_aid_volterr_avg_sample;

        if ((aid_st_volterr.f4_current_target - f4_i_mean) > (gs_f4_i_lsb))
        {
            f4_vd_ref += (gs_f4_i_lsb * aid_st_volterr.f4_r_dc);
        }
        else
        {
            aid_st_volterr.f4_current_table[aid_st_volterr.u4_meas_index] = f4_i_mean;
            aid_st_volterr.f4_voltage_table[aid_st_volterr.u4_meas_index] = f4_v_mean;
            aid_st_volterr.u4_meas_index ++;
            if ((aid_st_volterr.u4_meas_index >= g_u4_aid_volterr_point_num) || (f4_i_mean > aid_f4_rated_current))
            {
                f4_vd_ref = 0.0f;
                aid_st_volterr.e_status = AID_VOLTERR_SEQ_CALC;
            }
            else
            {
                aid_st_volterr.f4_current_target = f4_i_mean + (gs_f4_i_lsb * gs_f4_current_step_ilsb);
                f4_vd_ref += aid_st_volterr.f4_voltage_step;
            }
        }
        aid_st_volterr.u4_count = 0;
        aid_st_volterr.f4_meas_current_sum = 0.0f;
        aid_st_volterr.f4_meas_volt_sum = 0.0f;
    }
    else if (aid_st_volterr.u4_count >= (uint32_t)(aid_f4_ctrl_freq_hz * 0.001f * g_f4_aid_volterr_avg_wait_time))
    {
        aid_st_volterr.f4_meas_current_sum += f4_id;
        aid_st_volterr.f4_meas_volt_sum += aid_f4_va_ref_pwm;
    }
    else
    {
        /* Do nothing */
        __asm __volatile("nop\n");
    }

    /* Output signals */
    aid_f4_vd_ref = f4_vd_ref;
} /* End of function volterr_seq_id_verr_over_i */

/******************************************************************************
* Function Name : volterr_seq_calc
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void volterr_seq_calc(void)
{
    int32_t tab_n;
    int32_t i;
    int32_t j;
    float f4_max_verr = 0.0f;
    int32_t u4_sat_index;
    float f4_cursor;
    float f4_volt_err;

    tab_n = (int32_t)(aid_st_volterr.u4_meas_index);
    for(i = AID_VOLTERR_PRV_LINEREG_SIZE_TAB; i < (tab_n - AID_VOLTERR_PRV_LINEREG_SIZE_TAB); i++)
    {
        aidf_linereg(&aid_st_volterr.f4_current_table[i - AID_VOLTERR_PRV_LINEREG_SIZE_TAB],
                &aid_st_volterr.f4_voltage_table[i - AID_VOLTERR_PRV_LINEREG_SIZE_TAB],
                AID_VOLTERR_PRV_LINEREG_TAB_NUM,
                &aid_st_volterr.f4_slope_table[i],
                &aid_st_volterr.f4_intercept_table[i]);

        if (f4_max_verr < aid_st_volterr.f4_intercept_table[i])
        {
            f4_max_verr = aid_st_volterr.f4_intercept_table[i];
            u4_sat_index = i;
        }
    }

    /* Smoother the data */
#if (VOLTERR_PRV_SMOOTH_DATA == 1)
    for(i = AID_VOLTERR_PRV_LINEREG_SIZE_TAB; i < tab_n - AID_VOLTERR_PRV_LINEREG_SIZE_TAB; i++)
    {
        aid_st_volterr.f4_voltage_table[i] = aid_st_volterr.f4_slope_table[i] * aid_st_volterr.f4_current_table[i] +
                aid_st_volterr.f4_intercept_table[i];
        if (aid_st_volterr.f4_intercept_table[i] > f4_max_verr)
        {
            f4_max_verr = aid_st_volterr.f4_intercept_table[i];
            u4_sat_index = i;
        }
    }
#endif

    /* Determine the resistance to calculate voltage error, store saturated voltage error and current at that point*/
    aid_st_volterr.f4_r_dc = aid_st_volterr.f4_slope_table[u4_sat_index];
    aid_st_volterr.f4_sat_current = aid_st_volterr.f4_current_table[u4_sat_index];
    aid_st_volterr.f4_sat_voltage = aid_st_volterr.f4_voltage_table[u4_sat_index] -
            (aid_st_volterr.f4_sat_current * aid_st_volterr.f4_r_dc);

    /* Construct the interpolation table */
    j = 0;
    for(i = 0; i < AID_VOLTERR_OUTPUT_TAB_SIZE; i++)
    {
        f4_cursor = gs_f4_output_v_table[i] * aid_st_volterr.f4_sat_voltage;

        do
        {
            j++;
            f4_volt_err = aid_st_volterr.f4_voltage_table[j] -
                    (aid_st_volterr.f4_current_table[j] * aid_st_volterr.f4_r_dc);
        }while((f4_volt_err < f4_cursor) && (j < (tab_n - 1)));

        aid_f4_volterr_current_tab[i] = aid_st_volterr.f4_current_table[j];
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        aid_f4_volterr_voltage_tab[i] = f4_volt_err * 0.5f;
#else
        aid_f4_volterr_voltage_tab[i] = f4_volt_err;
#endif
    }

    aid_st_volterr.e_status = AID_VOLTERR_SEQ_CHECK;
} /* End of function volterr_seq_calc */

/******************************************************************************
* Function Name : volterr_seq_check
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void volterr_seq_check(void)
{
    aid_f4_volterr_vdc_ref = aid_f4_vdc_ad;
    aid_f4_volterr_rdc = aid_st_volterr.f4_r_dc;
    aid_st_volterr.e_status = AID_VOLTERR_SEQ_RESET;
} /* End of function volterr_seq_check */

/******************************************************************************
* Function Name : volterr_seq_reset
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void volterr_seq_reset(void)
{
    aid_st_volterr.e_status = AID_VOLTERR_SEQ_COMPLETED;
} /* End of function volterr_seq_reset */
