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
 * File Name    : r_aid_ke.c
 * Version      : 1.0
 * Description  : Open-loop flux identification module
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
//#include <machine.h>
#include <math.h>
#include "r_aid_ke.h"
#include "r_aid_config.h"
#include "r_aid_core.h"
#include "r_aid_function.h"

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#ifndef AID_PRV_FILE_CODE
#define AID_PRV_FILE_CODE (105)
#endif
#define KE_PRV_LINEREG_NUM  (1)
#define KE_PRV_KI_MULT              (0.2f)

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
typedef struct {
    e_aid_seq_ke_t s4_ke_status;
    enum aid_ke_sub_seq
    {
        KE_IDLE = 0,
        KE_OL_ID_RAMPUP,
        KE_OL_SPEED_RAMPUP,
        KE_OL_MEASURE_WAIT_STAB,
        KE_OL_MEASURE_SAMPLING,
        KE_OL_SPEED_SLOPE_DOWN,
        KE_CL_SPEED_RAMPUP,
        KE_CL_WAIT_STAB,
        KE_CL_MEASURE,
        KE_CL_SPEED_SLOPE_DOWN
    } s4_ke_sub_seq;
    uint32_t u4_sample_cnt;
    float f4_time_elapsed_ms;
    float f4_time_steady_ms;
    float f4_freq_ref_hz;
    float f4_id_ref_step;
    float f4_f_ref_step;
    float f4_e_sum;
    float f4_ed_sum;
    float f4_eq_sum;
    float f4_speed_rad_sum;
    float f4_vd_ref_sum;
    float f4_vq_ref_sum;
    float f4_speed_rad_ave;
    float f4_vd_ref_ave;
    float f4_vq_ref_ave;
    float f4_ed_ave;
    float f4_eq_ave;
    float f4_id_ave;
    float f4_ke_max;
    float f4_ke_min;
    float f4_ke_over_id[KE_PRV_LINEREG_NUM];
    float f4_id[KE_PRV_LINEREG_NUM];
    uint32_t u4_index;
    uint8_t u1_no_stop;                         /* Do not stop on finish if true */
} st_aid_ke_t;
/***********************************************************************************************************************
 Private global variables and functions
 **********************************************************************************************************************/
static void ke_seq_init(void);
static void ke_seq_ready(void);
static void ke_seq_measure(void);
static void ke_seq_check(void);
static void ke_seq_reset(void);
static void ke_prepare_closed_loop(void);

/* Ke */
st_aid_ke_t aid_st_ke;
static float gs_f4_cfg_assumed_inertia;

/***********************************************************************************************************************
 Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/
/* Open-loop flux measurement result, measured in AID_PARAMODE_KE stage */
float  aid_f4_ke_open;
float  aid_f4_ke_closed_loop;

/******************************************************************************
* Function Name : aid_ke_act
* Description   :
* Arguments     : None
* Return Value  : Whether the process is done (1) or not (0)
******************************************************************************/
uint16_t aid_ke_act(void)
{
    uint16_t ret = 0;

    switch (aid_st_ke.s4_ke_status)
    {
        case AID_SEQ_KE_INIT:
        {
            ke_seq_init();
        }
        break;

        case AID_SEQ_KE_READY:
        {
            ke_seq_ready();
        }
        break;

        case AID_SEQ_KE_MEASURE:
        {
            ke_seq_measure();
        }
        break;

        case AID_SEQ_KE_CHECK:
        {
            ke_seq_check();
        }
        break;

        case AID_SEQ_KE_RESET:
        {
            ke_seq_reset();
        }
        break;

        case AID_SEQ_KE_COMPLETED:
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
} /* End of function aid_ke_act */

/***********************************************************************************************************************
* Function Name : aid_ke_reset
* Description   : Initializes Ke (BEMF constant) identification module
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void aid_ke_reset(void)
{
    int32_t i;

    /* Ke */
    aid_st_ke.s4_ke_status = AID_SEQ_KE_INIT;
    aid_st_ke.u4_sample_cnt = 0;
    aid_st_ke.f4_time_elapsed_ms = 0.0f;
    aid_st_ke.f4_time_steady_ms = 0.0f;
    aid_st_ke.f4_freq_ref_hz = 0.0f;                /* Will be determined in seq_init */
    aid_st_ke.f4_id_ref_step = 0.0f;                /* Will be determined in seq_init */
    aid_st_ke.f4_f_ref_step = 0.0f;                 /* Will be determined in seq_init */
    aid_st_ke.f4_e_sum = 0.0f;
    aid_st_ke.f4_ed_sum = 0.0f;
    aid_st_ke.f4_eq_sum = 0.0f;
    aid_st_ke.f4_speed_rad_sum = 0.0f;
    aid_st_ke.f4_vd_ref_sum = 0.0f;
    aid_st_ke.f4_vq_ref_sum = 0.0f;
    aid_st_ke.f4_speed_rad_ave = 0.0f;
    aid_st_ke.f4_vd_ref_ave = 0.0f;
    aid_st_ke.f4_vq_ref_ave = 0.0f;
    aid_st_ke.f4_ed_ave = 0.0f;
    aid_st_ke.f4_eq_ave = 0.0f;
    aid_st_ke.f4_id_ave = 0.0f;
    aid_st_ke.f4_ke_max = 0.0f;
    aid_st_ke.f4_ke_min = 0.0f;
    aid_st_ke.u1_no_stop = false;
    for (i = 0; i < KE_PRV_LINEREG_NUM; i++)
    {
        aid_st_ke.f4_ke_over_id[i] = 0.0f;
        aid_st_ke.f4_id[i] = 0.0f;
    }
    aid_st_ke.u4_index = 0;
} /* End of function aid_ke_reset */

void aid_ke_config_assumed_inertia(float f4_assumed_inertia)
{
    gs_f4_cfg_assumed_inertia = f4_assumed_inertia;
} /* End of function aid_ke_config_assumed_inertia */

/**
 * Set no stop flag to determine whether the motor should be stopped after identification finished
 * @param no_stop   true(1): Do not stop, false(0): Stop on identification completed
 */
void aid_ke_config_no_stop_flag(uint8_t no_stop)
{
    aid_st_ke.u1_no_stop = no_stop;
} /* End of function aid_ke_config_no_stop_flag */

/******************************************************************************
* Function Name : ke_seq_init
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ke_seq_init(void)
{
    /* PI gains will be designed with motor parameters stored in CORE module */
    aid_core_config_current_pi_gains(g_f4_aid_current_omega, g_f4_aid_current_zeta);
    aid_core_config_bemf_obsv(g_f4_aid_e_obs_omega, g_f4_aid_e_obs_zeta);

    /* Set step of Id and omega */
    aid_st_ke.f4_id_ref_step = (aid_f4_rated_current * g_f4_aid_ke_id_ref_coef) / g_f4_aid_ke_id_up_time * aid_f4_ctrl_period_ms;
    aid_st_ke.f4_f_ref_step = g_f4_aid_ke_freq / g_f4_aid_ke_omega_up_time * aid_f4_ctrl_period_ms;
    aid_st_ke.f4_freq_ref_hz = 0.0f;

    aid_st_ke.f4_e_sum = 0.0f;
    aid_st_ke.f4_speed_rad_sum = 0.0f;
    aid_st_ke.f4_vd_ref_sum = 0.0f;
    aid_st_ke.f4_vq_ref_sum = 0.0f;

    aid_st_ke.f4_time_elapsed_ms = 0.0f;
    aid_st_ke.s4_ke_status = AID_SEQ_KE_READY;
    aid_st_ke.s4_ke_sub_seq = KE_OL_ID_RAMPUP;

    aid_f4_id_ref = 0.0f;

    /* Upgrade control level to enable BEMF observer and current PI controller */
    aid_core_set_ctrl_level(AID_CTRL_LEVEL_3);
} /* End of function ke_seq_init */

uint32_t wKeDebugTemp = 0;
bool bKeDebugTemp = false;

/******************************************************************************
* Function Name : ke_seq_ready
* Description   : Ramp-up Id, accelerate to the target speed
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ke_seq_ready(void)
{
    aid_st_ke.f4_time_elapsed_ms += aid_f4_ctrl_period_ms;
    switch(aid_st_ke.s4_ke_sub_seq)
    {
        case KE_OL_ID_RAMPUP:
        {
            /* Ramp-up id to prepare starting acceleration */
            aid_f4_id_ref += aid_st_ke.f4_id_ref_step;
            aid_f4_iq_ref = 0.0f;
            if (aid_st_ke.f4_time_elapsed_ms > g_f4_aid_ke_id_up_time)
            {
                aid_st_ke.s4_ke_sub_seq = KE_OL_SPEED_RAMPUP;
                aid_st_ke.f4_time_elapsed_ms = 0.0f;
            }
        }
        break;

        case KE_OL_SPEED_RAMPUP:
        {
            /* Accelerate motor to apparent speed to achieve enough BEMF,
             * acceleration will be stopped at g_f4_aid_ke_freq
             *  */
//            if((fabsf(aid_f4_e_lpf) > (aid_f4_vmag_max * 0.25f)) ||
//               (aid_st_ke.f4_time_elapsed_ms > g_f4_aid_ke_omega_up_time))
            if(aid_st_ke.f4_time_elapsed_ms > g_f4_aid_ke_omega_up_time)
            {
                aid_st_ke.s4_ke_sub_seq = KE_OL_MEASURE_WAIT_STAB;
                aid_st_ke.s4_ke_status = AID_SEQ_KE_MEASURE;
                aid_st_ke.f4_time_elapsed_ms = 0.0f;
                aid_st_ke.u4_sample_cnt = 0;
            }
            else
            {
                aid_st_ke.f4_freq_ref_hz += aid_st_ke.f4_f_ref_step;
                aid_f4_ref_speed_rad_ctrl = aid_st_ke.f4_freq_ref_hz * AID_TWOPI;
                aid_f4_speed_rad = aid_f4_ref_speed_rad_ctrl;
            }
        }
        break;

        default:
            AID_ASSERT_FAIL();
    }
} /* End of function ke_seq_ready */

/******************************************************************************
* Function Name : ke_seq_measure
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ke_seq_measure(void)
{
    aid_st_ke.f4_time_elapsed_ms += aid_f4_ctrl_period_ms;
    switch(aid_st_ke.s4_ke_sub_seq)
    {
        case KE_OL_MEASURE_WAIT_STAB:
        {
            if(aid_st_ke.f4_time_elapsed_ms >= (g_f4_aid_ke_measure_id_stab_wait + 1000))
            {
                aid_st_ke.f4_time_elapsed_ms = 0.0f;
                aid_st_ke.u4_sample_cnt = 0;
                aid_st_ke.s4_ke_sub_seq = KE_OL_MEASURE_SAMPLING;
            }
            break;
        }

        case KE_OL_MEASURE_SAMPLING:
        {
            /* Maintain the speed and acquire data for flux estimation */
            aid_f4_ref_speed_rad_ctrl = aid_st_ke.f4_freq_ref_hz * AID_TWOPI;

            aid_st_ke.f4_e_sum += aid_f4_e;
            aid_st_ke.f4_speed_rad_sum += (aid_st_ke.f4_freq_ref_hz * AID_TWOPI);
            aid_st_ke.f4_vd_ref_sum += aid_f4_vd_ref;
            aid_st_ke.f4_vq_ref_sum += aid_f4_vq_ref;
            aid_st_ke.f4_ed_sum += aid_f4_ed;
            aid_st_ke.f4_eq_sum += aid_f4_eq;
            aid_st_ke.u4_sample_cnt++;

            if (aid_st_ke.f4_time_elapsed_ms >= (g_f4_aid_ke_measure_time))
            {
                if (aid_st_ke.u4_index < KE_PRV_LINEREG_NUM)
                {
                    float f4_1_div_sample_cnt;

                    aid_f4_ke_open = aid_st_ke.f4_e_sum / aid_st_ke.f4_speed_rad_sum;

                    /* Cache the 1/(number of sample) to improve performance */
                    f4_1_div_sample_cnt = 1.0f / (float)aid_st_ke.u4_sample_cnt;
                    aid_st_ke.f4_vd_ref_ave = aid_st_ke.f4_vd_ref_sum * f4_1_div_sample_cnt;
                    aid_st_ke.f4_vq_ref_ave = aid_st_ke.f4_vq_ref_sum * f4_1_div_sample_cnt;
                    aid_st_ke.f4_speed_rad_ave = aid_st_ke.f4_speed_rad_sum * f4_1_div_sample_cnt;
                    aid_st_ke.f4_ed_ave = aid_st_ke.f4_ed_sum * f4_1_div_sample_cnt;
                    aid_st_ke.f4_eq_ave = aid_st_ke.f4_eq_sum * f4_1_div_sample_cnt;
                    aid_st_ke.f4_id_ave = aid_f4_id_ref;

                    /* Reset accumulated values and start measurement with lower id */
                    aid_f4_iq_ref = 0.0f;
                    aid_st_ke.f4_e_sum = 0.0f;
                    aid_st_ke.f4_ed_sum = 0.0f;
                    aid_st_ke.f4_eq_sum = 0.0f;
                    aid_st_ke.f4_speed_rad_sum = 0.0f;
                    aid_st_ke.f4_vd_ref_sum = 0.0f;
                    aid_st_ke.f4_vq_ref_sum = 0.0f;
                    aid_st_ke.f4_id[aid_st_ke.u4_index] = aid_st_ke.f4_id_ave;
                    aid_st_ke.f4_ke_over_id[aid_st_ke.u4_index] = aid_f4_ke_open;
                    aid_st_ke.u4_index ++;

                    if (KE_PRV_LINEREG_NUM == aid_st_ke.u4_index)
                    {
        #if (AID_KE_PRV_LINEREG_NUM > 1)
                        float slope;
                        float intercept;

                        aidf_linereg(aid_st_ke.f4_id, aid_st_ke.f4_ke_over_id, KE_PRV_LINEREG_NUM, &slope, &intercept);
                        aid_f4_ke_open = intercept;
        #elif (KE_PRV_LINEREG_NUM == 1)
                        aid_f4_ke_open = aid_st_ke.f4_ke_over_id[0];
        #else
        #error "Invalid number of samples of Ke linear regression"
        #endif
                        aid_st_ke.f4_time_elapsed_ms = 0;

                        ke_prepare_closed_loop();
                        aid_st_ke.s4_ke_sub_seq = KE_CL_SPEED_RAMPUP;
                    }
                    else
                    {
                        /* Reset timer and acquire more sample on different Id conditions */
                        aid_f4_id_ref = aid_f4_id_ref * 0.8f;
                        aid_st_ke.f4_time_elapsed_ms = 0;
                        aid_st_ke.s4_ke_sub_seq = KE_OL_MEASURE_WAIT_STAB;
                    }
                }
            }
        }
        break;

        case KE_OL_SPEED_SLOPE_DOWN:
        {
            /* Slow-down the speed after acquired enough samples */
            aid_f4_ref_speed_rad_ctrl -= (aid_st_ke.f4_f_ref_step * AID_TWOPI);
            aid_f4_ref_speed_rad = aid_f4_ref_speed_rad_ctrl;
            if (aid_f4_ref_speed_rad_ctrl <= 0.0f)
            {
                ke_prepare_closed_loop();
                aid_st_ke.s4_ke_sub_seq = KE_CL_SPEED_RAMPUP;
            }
        }
        break;

        case KE_CL_SPEED_RAMPUP:
        {
            if (aid_f4_ref_speed_rad_ctrl >= aid_f4_ref_speed_rad)
            {
                aid_st_ke.s4_ke_sub_seq = KE_CL_WAIT_STAB;
                aid_st_ke.f4_time_elapsed_ms = 0;
            }
        }
        break;

        case KE_CL_WAIT_STAB:
        {
            float f4_speed_err_ratio;

            aid_st_ke.f4_time_steady_ms += aid_f4_ctrl_period_ms;
            f4_speed_err_ratio = aid_f4_speed_lpf_rad / aid_f4_ref_speed_rad - 1.0f;
            if (fabs(f4_speed_err_ratio) > 0.02f)
            {
                aid_st_ke.f4_time_steady_ms = 0.0f;
            }
            if ((aid_st_ke.f4_time_steady_ms > 1000.0f) || (aid_st_ke.f4_time_elapsed_ms > 10000.0f))
            {
                aid_st_ke.f4_time_elapsed_ms = 0;
                aid_st_ke.f4_e_sum = 0.0f;
                aid_st_ke.f4_speed_rad_sum = 0.0f;
                aid_st_ke.s4_ke_sub_seq = KE_CL_MEASURE;
            }
        }
        break;

        case KE_CL_MEASURE:
        {
            aid_st_ke.f4_e_sum += aid_f4_e;
            aid_st_ke.f4_speed_rad_sum += aid_f4_speed_rad;
            aid_st_ke.u4_sample_cnt++;
            if (aid_st_ke.f4_time_elapsed_ms > g_f4_aid_ke_measure_time)
            {
                aid_f4_ke_closed_loop = aid_st_ke.f4_e_sum / aid_st_ke.f4_speed_rad_sum;
                if (true == aid_st_ke.u1_no_stop)
                {
                    aid_st_ke.s4_ke_status = AID_SEQ_KE_CHECK;
                }
                else
                {
                    aid_st_ke.s4_ke_sub_seq = KE_CL_SPEED_SLOPE_DOWN;
                }
            }
        }
        break;

        case KE_CL_SPEED_SLOPE_DOWN:
        {
            /* Slow-down the speed after acquired enough samples */
            aid_f4_ref_speed_rad_ctrl -= (aid_st_ke.f4_f_ref_step * AID_TWOPI);
            if (aid_f4_ref_speed_rad_ctrl <= 0.0f)
            {
                aid_st_ke.s4_ke_status = AID_SEQ_KE_CHECK;
            }
            aid_f4_ref_speed_rad = aid_f4_ref_speed_rad_ctrl;
        }
        break;

        default:
            AID_ASSERT_FAIL();
    }
} /* End of function ke_seq_measure */

/******************************************************************************
* Function Name : ke_seq_check
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ke_seq_check(void)
{
    float f4_temp0;
    float f4_temp1;

    f4_temp0 = aid_st_ke.f4_vd_ref_ave * aid_st_ke.f4_vd_ref_ave;
    f4_temp1 = aid_st_ke.f4_vq_ref_ave * aid_st_ke.f4_vq_ref_ave;
    f4_temp0 = sqrtf(f4_temp0 + f4_temp1);
    aid_st_ke.f4_ke_max = f4_temp0 / aid_st_ke.f4_speed_rad_ave;

    f4_temp0 = aid_st_ke.f4_vq_ref_ave - (aid_st_ke.f4_speed_rad_ave * aid_f4_ld * aid_st_ke.f4_id_ave);
    f4_temp0 = fabsf(f4_temp0);
    aid_st_ke.f4_ke_min = (f4_temp0 / aid_st_ke.f4_speed_rad_ave) * 0.8f;

    if ((aid_f4_ke_open > aid_st_ke.f4_ke_max) || (aid_f4_ke_open <= aid_st_ke.f4_ke_min))
    {
        aid_core_throw_error(AID_ERROR_KE);
    }

    aid_st_ke.s4_ke_status = AID_SEQ_KE_RESET;
    aid_st_ke.f4_time_elapsed_ms = 0.0f;
} /* End of function ke_seq_check */

/******************************************************************************
* Function Name : ke_seq_reset
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void ke_seq_reset(void)
{
    aid_st_ke.f4_time_elapsed_ms += aid_f4_ctrl_period_ms;

    if (aid_st_ke.u1_no_stop == false)
    {
        aid_f4_vd_ref = 0.0f;
        aid_f4_vq_ref = 0.0f;
        aid_f4_id_ref = 0.0f;
        aid_f4_iq_ref = 0.0f;
        aid_f4_angle_rad = 0.0f;
        aid_f4_ref_speed_rad_ctrl = 0.0f;
        aid_f4_speed_rad = 0.0f;
    }
    aid_st_ke.f4_freq_ref_hz = 0.0f;

    if (aid_st_ke.f4_time_elapsed_ms > g_f4_aid_ke_reset_time)
    {
        aid_st_ke.f4_time_elapsed_ms = 0;
        aid_st_ke.s4_ke_status = AID_SEQ_KE_COMPLETED;
        aid_st_ke.s4_ke_sub_seq = KE_IDLE;
    }

} /* End of function ke_seq_reset */

/*******************************************************************************************************************//**
 * @brief Configure start-up parameter, gains of speed controller and other necessary parameters for starting
 *        KE identification with closed-loop FOC
 **********************************************************************************************************************/
static void ke_prepare_closed_loop(void)
{
    float f4_max_speed;
    float f4_kp;
    float f4_kidt;
    float f4_speed_over_iq;
    float f4_temp;

    gs_f4_cfg_assumed_inertia = g_f4_aid_ke_assumed_inertia;

    f4_max_speed = aid_f4_vmag_max / aid_f4_ke_open;
    aid_core_set_startup_params(aid_f4_rated_current * g_f4_aid_jd_id_ref_coef,
                                g_f4_aid_ol2cl_bemf_th_coef * f4_max_speed,
                                g_f4_aid_cl2ol_bemf_th_coef * f4_max_speed);

    f4_speed_over_iq = aid_f4_ke_open * aid_f4_pole_pairs * aid_f4_pole_pairs;
    f4_temp = gs_f4_cfg_assumed_inertia / f4_speed_over_iq;
    f4_kp = (g_f4_aid_jd_speedpi_omega * AID_TWOPI * 2.0f)  * f4_temp;
    f4_kidt = (AID_TWOPI * g_f4_aid_jd_speedpi_omega * AID_TWOPI * g_f4_aid_jd_speedpi_omega) * KE_PRV_KI_MULT * f4_temp * aid_f4_spd_ctrl_period_ms * 0.001f;

    /* Set PI gains through the common AID core interface */
    aid_core_set_speed_pi(f4_kp, f4_kidt);
    aid_core_set_ctrl_level(AID_CTRL_LEVEL_4);

    aid_f4_ref_speed_rad = f4_max_speed * g_f4_aid_jd_speed_offset_coef;
    aid_f4_limit_speed_change = g_f4_aid_speed_change_rate_limit;
} /* End of function ke_prepare_closed_loop */
