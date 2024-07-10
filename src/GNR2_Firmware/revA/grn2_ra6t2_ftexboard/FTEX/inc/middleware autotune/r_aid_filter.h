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
* File Name   : r_aid_filter.h
* Description : Definitions of processes of filters
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version
*         : 05.04.2017 1.00
*         : 07.07.2017 1.01
***********************************************************************************************************************/

/* Guard against multiple inclusion */
#ifndef R_AID_FILTER_H
#define R_AID_FILTER_H

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>

/***********************************************************************************************************************
* Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Structure definitions
***********************************************************************************************************************/
/* First order LPF structure */
typedef struct
{
    float f4_pre_output;      /* Previous value of output */
    float f4_pre_input;       /* Previous value of input */
    float f4_omega_t;         /* Calculate value */
    float f4_gain_ka;         /* LPF gain */
    float f4_gain_kb;         /* LPF gain */
}st_aid_1st_order_lpf_t;

/***********************************************************************************************************************
* Global function definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name : R_AID_Lpff
* Description   : LPF process
* Arguments     : f4_lpf_input      - LPF input value
                  f4_pre_lpf_output - Previous LPF output value
                  f4_lpf_k          - LPF parameter
* Return Value  : LPF output value
***********************************************************************************************************************/
float R_AID_Lpff(float f4_lpf_input, float f4_pre_lpf_output, float f4_lpf_k);

/***********************************************************************************************************************
* Function Name : R_AID_Limitf
* Description   : Limit with maximum limit and minimum limit
* Arguments     : f4_value - Target value
                  f4_max   - Maximum limit
                  f4_min   - Minimum limit
* Return Value  : Limited target value
***********************************************************************************************************************/
float R_AID_Limitf(float f4_value, float f4_max, float f4_min);

/***********************************************************************************************************************
* Function Name : R_AID_LimitfAbs
* Description   : Limit with absolute value
* Arguments     : f4_value       - Target value
                  f4_limit_value - Limit
* Return Value  : Limited value
***********************************************************************************************************************/
float R_AID_LimitfAbs(float f4_value, float f4_limit_value);

/***********************************************************************************************************************
* Function Name : R_AID_FirstOrderLpffInit
* Description   : Initialize First Order LPF
* Arguments     : st_lpf - First order LPF structure (pointer)
* Return Value  : None
***********************************************************************************************************************/
void R_AID_FirstOrderLpffInit(st_aid_1st_order_lpf_t *st_lpf);

/***********************************************************************************************************************
* Function Name : R_AID_FirstOrderLpffReset
* Description   : Reset First Order LPF
* Arguments     : st_lpf - First order LPF structure (pointer)
* Return Value  : None
***********************************************************************************************************************/
void R_AID_FirstOrderLpffReset(st_aid_1st_order_lpf_t *st_lpf);

/***********************************************************************************************************************
* Function Name : R_AID_FirstOrderLpffGainCalc
* Description   : Gain Calculation for First Order LPF
* Arguments     : st_lpf          - First order LPF structure (pointer)
*                 f4_omega        - Natural frequency
*                 f4_ctrl_period  - Control period
* Return Value  : None
***********************************************************************************************************************/
void R_AID_FirstOrderLpffGainCalc(st_aid_1st_order_lpf_t *st_lpf, float f4_omega, float f4_ctrl_period);

/***********************************************************************************************************************
* Function Name : R_AID_FirstOrderLpff
* Description   : First Order LPF
* Arguments     : st_lpf   - First order LPF structure (pointer)
                : f4_input - Input value
* Return Value  : Filtered value
***********************************************************************************************************************/
float R_AID_FirstOrderLpff(st_aid_1st_order_lpf_t *st_lpf, float f4_input);
#endif /* R_AID_FILTER_H */
