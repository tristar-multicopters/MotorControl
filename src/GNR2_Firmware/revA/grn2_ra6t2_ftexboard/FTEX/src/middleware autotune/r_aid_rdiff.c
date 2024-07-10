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
 * File Name    : r_aid_rdiff.c
 * Version      : 1.0
 * Description  : Resistance identification module (voltage differential method)
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
#include <stdint.h>
#include "r_aid_rdiff.h"
#include "r_aid_core.h"
#include "r_aid_config.h"
#include "r_aid_function.h"

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#ifndef AID_PRV_FILE_CODE
#define AID_PRV_FILE_CODE (103)
#endif

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
/* RL measurement */
typedef struct {
    e_aid_seq_r_diff_t s4_rdiff_status;
    float f4_current1;
    float f4_current2;
    float f4_current3;
    float f4_current4;
    float f4_vd_sample1;
    float f4_vd_sample2;
    float f4_id_sample1;
    float f4_id_sample2;
    float f4_sum_vu;
    float f4_sum_iu;
    uint32_t u4_cnt_stab;
    uint32_t u4_timeout_cnt;
} st_aid_rdiff_t;

/***********************************************************************************************************************
 Private global variables and functions
 **********************************************************************************************************************/
static void rdiff_seq_init(void);
static void rdiff_seq_ready1(void);
static void rdiff_seq_ready12(void);
static void rdiff_seq_measure1(void);
static void rdiff_seq_ready2(void);
static void rdiff_seq_ready22(void);
static void rdiff_seq_measure2(void);
static void rdiff_seq_check(void);
static void rdiff_seq_reset(void);

st_aid_rdiff_t aid_st_rdiff;

/***********************************************************************************************************************
 Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/
float  aid_f4_r_diff;
float  aid_f4_volterr_est;

/******************************************************************************
* Function Name : aid_rdiff_act
* Description   :
* Arguments     : None
* Return Value  : Whether the process is done (1) or not (0)
******************************************************************************/
uint16_t aid_rdiff_act(void)
{
    uint16_t ret = 0;
    switch (aid_st_rdiff.s4_rdiff_status)
    {
        case AID_SEQ_R_DIFF_INIT:
        {
            rdiff_seq_init();
        }
        break;

        case AID_SEQ_R_DIFF_READY1:
        {
            rdiff_seq_ready1();
        }
        break;

        case AID_SEQ_R_DIFF_READY12:
        {
            rdiff_seq_ready12();
        }
        break;

        case AID_SEQ_R_DIFF_MEASURE1:
        {
            rdiff_seq_measure1();
        }
        break;

        case AID_SEQ_R_DIFF_READY2:
        {
            rdiff_seq_ready2();
        }
        break;

        case AID_SEQ_R_DIFF_READY22:
        {
            rdiff_seq_ready22();
        }
        break;

        case AID_SEQ_R_DIFF_MEASURE2:
        {
            rdiff_seq_measure2();
        }
        break;

        case AID_SEQ_R_DIFF_CHECK:
        {
            rdiff_seq_check();
        }
        break;

        case AID_SEQ_R_DIFF_RESET:
        {
            rdiff_seq_reset();
        }
        break;

        case AID_SEQ_R_DIFF_COMPLETED:
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
} /* End of function aid_rdiff_act */

/***********************************************************************************************************************
* Function Name: aid_rdiff_reset
* Description  : Throw an error with the given error code and stop PWM output
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void aid_rdiff_reset(void)
{
    aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_INIT;

    /* RL measurement */
    aid_st_rdiff.f4_current1 = 0.0f;
    aid_st_rdiff.f4_current2 = 0.0f;
    aid_st_rdiff.f4_current3 = 0.0f;
    aid_st_rdiff.f4_current4 = 0.0f;
    aid_st_rdiff.f4_vd_sample1 = 0.0f;
    aid_st_rdiff.f4_vd_sample2 = 0.0f;
    aid_st_rdiff.f4_id_sample1 = 0.0f;
    aid_st_rdiff.f4_id_sample2 = 0.0f;
    aid_st_rdiff.f4_sum_vu = 0.0f;
    aid_st_rdiff.f4_sum_iu = 0.0f;
    aid_st_rdiff.u4_cnt_stab = 0;
    aid_st_rdiff.u4_timeout_cnt = 0;
    aid_f4_r_diff = 0.0f;
    aid_f4_volterr_est = 0.0f;
} /* End of function aid_rdiff_reset */

/******************************************************************************
* Function Name : rdiff_seq_init
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_init(void)
{
    aid_st_rdiff.f4_current1 = aid_f4_rated_current * g_f4_aid_r_diff_measure_current1;
    aid_st_rdiff.f4_current2 = aid_f4_rated_current * g_f4_aid_r_diff_measure_current2;
    aid_st_rdiff.f4_current3 = aid_f4_rated_current * g_f4_aid_r_diff_measure_current3;
    aid_st_rdiff.f4_current4 = aid_f4_rated_current * g_f4_aid_r_diff_measure_current4;

    /* Make sure that the angle do not change during the measurement */
    aid_f4_angle_rad = 0.0f;
    aid_f4_ref_speed_rad = 0.0f;

    aid_f4_vd_ref = 0.0f;
    aid_f4_vq_ref = 0.0f;
    aid_u4_sample_cnt = 0;
    aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_READY1;
    aid_s4_rotor_angle_mode = AID_ROTOR_ANGLE_MODE_OPENLOOP;
    aid_core_set_ctrl_level(AID_CTRL_LEVEL_0);
} /* End of function rdiff_seq_init */

/******************************************************************************
* Function Name : rdiff_seq_ready1
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_ready1(void)
{
    aid_u4_sample_cnt++;
    if (1 == aid_u4_sample_cnt)
    {
        /* Adjust voltage to fix target current every g_f4_aid_r_diff_vd_step_wait sample cycles */
        /* Voltage is adjusted by +-g_f4_aid_r_diff_vd_step[V] every time */
        if ((aid_f4_id_ad < aid_st_rdiff.f4_current1) &&
            (aid_st_rdiff.u4_timeout_cnt <= g_u4_aid_r_diff_vd_adjust_timeout))
        {
            aid_f4_vd_ref += g_f4_aid_r_diff_vd_step;
            aid_st_rdiff.u4_cnt_stab = 0;
        }
        else if (aid_f4_id_ad > aid_st_rdiff.f4_current2)
        {
            aid_f4_vd_ref -= g_f4_aid_r_diff_vd_step;
            aid_st_rdiff.u4_cnt_stab = 0;
            aid_st_rdiff.u4_timeout_cnt++;
        }
        else
        {
            aid_st_rdiff.u4_cnt_stab++;
        }
    }
    /* Wait for g_f4_aid_r_diff_vd_step_wait [ms] */
    else if (aid_u4_sample_cnt > (uint32_t)(g_f4_aid_r_diff_vd_step_wait * aid_f4_ctrl_freq_hz * 0.001f))
    {
        aid_u4_sample_cnt = 0;

        /* Wait for g_f4_aid_r_diff_stab_wait [ms] */
        if (aid_st_rdiff.u4_cnt_stab > (uint32_t)(g_f4_aid_r_diff_stab_wait * aid_f4_ctrl_freq_hz * 0.001f))
        {
            /* Wait (g_f4_aid_r_diff_stab_wait * g_f4_aid_r_diff_vd_step_wait) sample cycles after reached target current*/
            aid_st_rdiff.u4_cnt_stab = 0;
            aid_st_rdiff.u4_timeout_cnt = 0;
            aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_READY12;
        }
    }
    else
    {
        /* Do nothing */
//        nop();
    }
} /* End of function rdiff_seq_ready1 */

/******************************************************************************
* Function Name : rdiff_seq_ready12
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_ready12(void)
{
    aid_u4_sample_cnt++;

    /* Wait for g_f4_aid_r_diff_measure_wait [ms] */
    if (aid_u4_sample_cnt > (uint32_t)(g_f4_aid_r_diff_measure_wait * aid_f4_ctrl_freq_hz * 0.001f))
    {
        aid_u4_sample_cnt = 0;
        aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_MEASURE1;
    }
} /* End of function rdiff_seq_ready12 */

/******************************************************************************
* Function Name : rdiff_seq_measure1
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_measure1(void)
{

    aid_u4_sample_cnt++;
    aid_st_rdiff.f4_vd_sample1 += aid_f4_vd_ref;
    aid_st_rdiff.f4_id_sample1 += aid_f4_id_ad;

    /* Wait for g_f4_aid_r_diff_measure_time [ms] */
    if (aid_u4_sample_cnt >= (uint32_t)(g_f4_aid_r_diff_measure_time * aid_f4_ctrl_freq_hz * 0.001f))
    {
        aid_st_rdiff.f4_vd_sample1 = aid_st_rdiff.f4_vd_sample1 / (float)aid_u4_sample_cnt;
        aid_st_rdiff.f4_id_sample1 = aid_st_rdiff.f4_id_sample1 / (float)aid_u4_sample_cnt;
        aid_u4_sample_cnt = 0;
        aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_READY2;
    }
} /* End of function rdiff_seq_measure1 */

/******************************************************************************
* Function Name : rdiff_seq_ready2
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_ready2(void)
{
    aid_u4_sample_cnt++;
    if (1 == aid_u4_sample_cnt)
    {
        if ((aid_f4_id_ad < aid_st_rdiff.f4_current3) &&
            (aid_st_rdiff.u4_timeout_cnt <= g_u4_aid_r_diff_vd_adjust_timeout))
        {
            aid_f4_vd_ref += g_f4_aid_r_diff_vd_step;
            aid_st_rdiff.u4_cnt_stab = 0;
        }
        else if (aid_f4_id_ad > aid_st_rdiff.f4_current4)
        {
            aid_f4_vd_ref -= g_f4_aid_r_diff_vd_step;
            aid_st_rdiff.u4_cnt_stab = 0;
            aid_st_rdiff.u4_timeout_cnt++;
        }
        else
        {
            aid_st_rdiff.u4_cnt_stab++;
        }
    }
    /* Wait for g_f4_aid_r_diff_vd_step_wait [ms] */
    else if (aid_u4_sample_cnt > (uint32_t)(g_f4_aid_r_diff_vd_step_wait * aid_f4_ctrl_freq_hz * 0.001f))
    {
        aid_u4_sample_cnt = 0;

        /* Wait for g_f4_aid_r_diff_stab_wait [ms] */
        if (aid_st_rdiff.u4_cnt_stab > (uint32_t)(g_f4_aid_r_diff_stab_wait * aid_f4_ctrl_freq_hz * 0.001f))
        {
            aid_st_rdiff.u4_cnt_stab = 0;
            aid_st_rdiff.u4_timeout_cnt = 0;
            aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_READY22;
        }
    }
    else
    {
        /* Do nothing */
//        nop();
    }
} /* End of function rdiff_seq_ready2 */

/******************************************************************************
* Function Name : rdiff_seq_ready22
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_ready22(void)
{
    aid_u4_sample_cnt++;

    /* Wait for g_f4_aid_r_diff_measure_wait [ms] */
    if (aid_u4_sample_cnt > (uint32_t)(g_f4_aid_r_diff_measure_wait * aid_f4_ctrl_freq_hz * 0.001f))
    {
        aid_u4_sample_cnt = 0;
        aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_MEASURE2;
    }
} /* End of function rdiff_seq_ready22 */

/******************************************************************************
* Function Name : rdiff_seq_measure2
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_measure2(void)
{
    aid_u4_sample_cnt++;
    aid_st_rdiff.f4_vd_sample2 += aid_f4_vd_ref;
    aid_st_rdiff.f4_id_sample2 += aid_f4_id_ad;

    /* Wait for g_f4_aid_r_diff_measure_time [ms] */
    if (aid_u4_sample_cnt >= (uint32_t)(g_f4_aid_r_diff_measure_time * aid_f4_ctrl_freq_hz * 0.001f))
    {
        aid_st_rdiff.f4_vd_sample2 = aid_st_rdiff.f4_vd_sample2 / (float)aid_u4_sample_cnt;
        aid_st_rdiff.f4_id_sample2 = aid_st_rdiff.f4_id_sample2 / (float)aid_u4_sample_cnt;
        aid_f4_r_diff = (aid_st_rdiff.f4_vd_sample2 - aid_st_rdiff.f4_vd_sample1) / (aid_st_rdiff.f4_id_sample2 - aid_st_rdiff.f4_id_sample1);
        aid_f4_volterr_est  = aid_st_rdiff.f4_vd_sample2 - (aid_f4_r_diff * aid_st_rdiff.f4_id_sample2);
        aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_RESET;
        aid_u4_sample_cnt = 0;
        aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_CHECK;
    }
} /* End of function rdiff_seq_measure2 */

/******************************************************************************
* Function Name : rdiff_seq_check
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_check(void)
{
    if (aid_f4_r_diff > aid_f4_r_max)
    {
        aid_core_throw_error(AID_ERROR_R_DIFF);
    }

    if (aid_f4_r_diff <= aid_f4_r_min)
    {
        aid_core_throw_error(AID_ERROR_R_DIFF);
    }

    aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_RESET;
} /* End of function rdiff_seq_check */

/******************************************************************************
* Function Name : rdiff_seq_reset
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void rdiff_seq_reset(void)
{
    aid_f4_vd_ref = 0.0f;
    aid_u4_sample_cnt++;

    /* Wait for g_f4_aid_r_diff_reset_time [ms] */
    if (aid_u4_sample_cnt > (uint32_t)(g_f4_aid_r_diff_reset_time * aid_f4_ctrl_freq_hz * 0.001f))
    {
        aid_u4_sample_cnt = 0;
        aid_st_rdiff.s4_rdiff_status = AID_SEQ_R_DIFF_COMPLETED;
    }
} /* End of function rdiff_seq_reset */
