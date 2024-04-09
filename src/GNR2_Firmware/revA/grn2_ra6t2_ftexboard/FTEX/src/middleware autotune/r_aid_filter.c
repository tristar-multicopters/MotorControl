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
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name   : r_aid_filter.c
* Description : Processes of filters
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version
*         : 05.04.2017 1.00
*         : 07.07.2017 1.01
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>
#include <math.h>
#include "r_aid_filter.h"

#define AID_FILTER_PRV_PI   (3.1415926f)
/***********************************************************************************************************************
* Function Name : R_AID_Lpff
* Description   : LPF process
* Arguments     : f4_lpf_input      - LPF input value
                  f4_pre_lpf_output - Previous LPF output value
                  f4_lpf_k          - LPF parameter
* Return Value  : LPF output value
***********************************************************************************************************************/
float R_AID_Lpff(float f4_lpf_input, float f4_pre_lpf_output, float f4_lpf_k)
{
    float f4_temp;

    f4_temp = f4_pre_lpf_output + (f4_lpf_k * (f4_lpf_input - f4_pre_lpf_output));

    return (f4_temp);
} /* End of function R_AID_Lpff */

/***********************************************************************************************************************
* Function Name : R_AID_Limitf
* Description   : Limit with maximum limit and minimum limit
* Arguments     : f4_value - Target value
                  f4_max   - Maximum limit
                  f4_min   - Minimum limit
* Return Value  : Limited target value
***********************************************************************************************************************/
float R_AID_Limitf(float f4_value, float f4_max, float f4_min)
{
    float f4_temp;

    f4_temp = f4_value;
    if (f4_value > f4_max)
    {
        f4_temp = f4_max;
    }
    else if (f4_value < f4_min)
    {
        f4_temp = f4_min;
    }
    else
    {
        /* Do Nothing */
    }

    return (f4_temp);
} /* End of function R_AID_Limitf */

/***********************************************************************************************************************
* Function Name : R_AID_LimitfAbs
* Description   : Limit with absolute value
* Arguments     : f4_value       - Target value
                  f4_limit_value - Limit
* Return Value  : Limited value
***********************************************************************************************************************/
float R_AID_LimitfAbs(float f4_value, float f4_limit_value)
{
    float f4_temp0;
    float f4_ret;

    f4_temp0 = fabsf(f4_value);
    if (f4_temp0 <= f4_limit_value)
    {
        f4_ret = f4_value;
    }
    else if (f4_value > f4_limit_value)
    {
        f4_ret = f4_limit_value;
    }
    else
    {
        f4_ret = (-f4_limit_value);
    }

    return (f4_ret);
} /* End of function R_AID_LimitfAbs */

/***********************************************************************************************************************
* Function Name : R_AID_FirstOrderLpffInit
* Description   : Initialize First Order LPF
* Arguments     : st_lpf - First order LPF structure (pointer)
* Return Value  : None
***********************************************************************************************************************/
void R_AID_FirstOrderLpffInit(st_aid_1st_order_lpf_t *st_lpf)
{
    st_lpf->f4_pre_output   = 0.0f;
    st_lpf->f4_pre_input    = 0.0f;
    st_lpf->f4_omega_t      = 0.0f;
    st_lpf->f4_gain_ka      = 0.0f;
    st_lpf->f4_gain_kb      = 0.0f;
} /* End of function R_AID_FirstOrderLpffInit */

/***********************************************************************************************************************
* Function Name : R_AID_FirstOrderLpffReset
* Description   : Reset First Order LPF
* Arguments     : st_lpf - First order LPF structure (pointer)
* Return Value  : None
***********************************************************************************************************************/
void R_AID_FirstOrderLpffReset(st_aid_1st_order_lpf_t *st_lpf)
{
    st_lpf->f4_pre_output   = 0.0f;
    st_lpf->f4_pre_input    = 0.0f;
} /* End of function R_AID_FirstOrderLpffReset */

/***********************************************************************************************************************
* Function Name : R_AID_FirstOrderLpffGainCalc
* Description   : Gain Calculation for First Order LPF
* Arguments     : st_lpf          - First order LPF structure (pointer)
*                 f4_omega        - Natural frequency
*                 f4_ctrl_period  - Control period
* Return Value  : None
***********************************************************************************************************************/
void R_AID_FirstOrderLpffGainCalc(st_aid_1st_order_lpf_t *st_lpf, float f4_omega, float f4_ctrl_period)
{
    st_lpf->f4_omega_t = (2.0f * AID_FILTER_PRV_PI * f4_omega) * f4_ctrl_period;
    st_lpf->f4_gain_ka = (2.0f - st_lpf->f4_omega_t) / (st_lpf->f4_omega_t + 2.0f);
    st_lpf->f4_gain_kb = st_lpf->f4_omega_t / (st_lpf->f4_omega_t + 2.0f);
} /* End of function R_AID_FirstOrderLpffGainCalc */

/***********************************************************************************************************************
* Function Name : R_AID_FirstOrderLpff
* Description   : First Order LPF
* Arguments     : st_lpf   - First order LPF structure (pointer)
                : f4_input - Input value
* Return Value  : Filtered value
***********************************************************************************************************************/
float R_AID_FirstOrderLpff(st_aid_1st_order_lpf_t *st_lpf, float f4_input)
{
    float f4_temp0;

    f4_temp0 = (st_lpf->f4_gain_ka * st_lpf->f4_pre_output);
    f4_temp0 += (st_lpf->f4_gain_kb * (f4_input + st_lpf->f4_pre_input));

    st_lpf->f4_pre_input = f4_input;
    st_lpf->f4_pre_output = f4_temp0;

    return (f4_temp0);
} /* End of function R_AID_FirstOrderLpff */
