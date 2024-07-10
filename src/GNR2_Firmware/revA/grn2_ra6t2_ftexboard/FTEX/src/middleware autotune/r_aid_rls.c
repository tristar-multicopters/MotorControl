/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2018 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name   : r_aid_rls.c
* Description : The function of recursive least square algorithm
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : 05.04.2017  Ver.1.00
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include "r_aid_rls.h"

/***********************************************************************************************************************
* Function Name: aidf_rls3_init
* Description  : Executes triple input 3nd order RLS module to estimate weight coefficients
* Arguments    : f4_a_in
*                   - The 1st input, corresponding to the coefficient a
*                f4_b_in
*                   - The 2nd input, corresponding to the coefficient b
*                f4_y
*                   - The measured output
*                f4_a_est
*                   - Where to store the newest estimated weight coefficient a
*                f4_b_est
*                   - Where to store the newest estimated weight coefficient b
* Return Value : None
***********************************************************************************************************************/
void aidf_rls3_init(st_rls3_t *rls3, float ini_p_1, float int_p_2, float int_p_3, float ini_k_forget,
                   float f4_in_1_init, float f4_in_2_init, float f4_in_3_init)
{
    rls3->f4_p[0][0] = ini_p_1;
    rls3->f4_p[0][1] = 0.0f;
    rls3->f4_p[0][2] = 0.0f;
    rls3->f4_p[1][0] = 0.0f;
    rls3->f4_p[1][1] = int_p_2;
    rls3->f4_p[1][2] = 0.0f;
    rls3->f4_p[2][0] = 0.0f;
    rls3->f4_p[2][1] = 0.0f;
    rls3->f4_p[2][2] = int_p_3;

    rls3->f4_y_est = 0.0f;
    rls3->f4_err   = 0.0f;
    rls3->f4_k[0]  = 0.0f;
    rls3->f4_k[1]  = 0.0f;
    rls3->f4_k[2]  = 0.0f;
    rls3->f4_w[0]  = f4_in_1_init;
    rls3->f4_w[1]  = f4_in_2_init;
    rls3->f4_w[2]  = f4_in_3_init;
    rls3->f4_k_forget = ini_k_forget;
} /* End of function aidf_rls3_init */

/***********************************************************************************************************************
* Function Name: aidf_rls3_exec
* Description  : Executes dual input 2nd order RLS module to estimate weight coefficients
* Arguments    : f4_a_in
*                   - The 1st input, corresponding to the coefficient a
*                f4_b_in
*                   - The 2nd input, corresponding to the coefficient b
*                f4_y
*                   - The measured output
*                f4_a_est
*                   - Where to store the newest estimated weight coefficient a
*                f4_b_est
*                   - Where to store the newest estimated weight coefficient b
* Return Value : None
***********************************************************************************************************************/
void aidf_rls3_exec(st_rls3_t *rls3, float *f4_in, float f4_y, float *f4_weight)
{
    float f4_in_1;
    float f4_in_2;
    float f4_in_3;
    float f4_upu;
    float f4_up_1;
    float f4_up_2;
    float f4_up_3;
    float f4_temp;
    float f4_1_div_fgt;
    float f4_pu_1;
    float f4_pu_2;
    float f4_pu_3;

    /* RLS identification
     * Input vector                     : u(n) = f4_in
     * Desired output(measured output)  : d(n) = f4_y
     * Estimated output                 : y(n) = rls3->f4_y_est
     * See https://jp.mathworks.com/help/dsp/ref/dsp.rlsfilter-system-object.html for detailed algorithm
     * */
    f4_in_1 = f4_in[0];
    f4_in_2 = f4_in[1];
    f4_in_3 = f4_in[2];

    /* Pu(n) = P(n - 1) * u(n) */
    f4_pu_1 = (rls3->f4_p[0][0] * f4_in_1) + (rls3->f4_p[0][1] * f4_in_2) + (rls3->f4_p[0][2] * f4_in_3);
    f4_pu_2 = (rls3->f4_p[1][0] * f4_in_1) + (rls3->f4_p[1][1] * f4_in_2) + (rls3->f4_p[1][2] * f4_in_3);
    f4_pu_3 = (rls3->f4_p[2][0] * f4_in_1) + (rls3->f4_p[2][1] * f4_in_2) + (rls3->f4_p[2][2] * f4_in_3);

    /* uPu(n) = u^H(n) * Pu(n), u(n) is all real number, Hermitian transpose = regular transpose  */
    f4_upu = (f4_in_1 * f4_pu_1) + (f4_in_2 * f4_pu_2) + (f4_in_3 * f4_pu_3);

    /* k(n) = Pu(n) / (forgetting factor + uPu(n)) , f4_temp = 1 / (forgetting factor + uPu(n))*/
    f4_temp = 1.0f / (rls3->f4_k_forget + f4_upu);
    rls3->f4_k[0] = f4_pu_1 * f4_temp;
    rls3->f4_k[1] = f4_pu_2 * f4_temp;
    rls3->f4_k[2] = f4_pu_3 * f4_temp;

    /* y(n) = w^T(n - 1) * u(n) */
    rls3->f4_y_est = (rls3->f4_w[0] * f4_in[0]) + (rls3->f4_w[1] * f4_in[1]) + (rls3->f4_w[2] * f4_in[2]);

    /* e(n) = d(n) - y(n) */
    rls3->f4_err = f4_y - rls3->f4_y_est;

    /* w(n) = w(n-1) + k(n)  * (d(n) - y(n)) */
    rls3->f4_w[0] = rls3->f4_w[0] + (rls3->f4_k[0] * rls3->f4_err);
    rls3->f4_w[1] = rls3->f4_w[1] + (rls3->f4_k[1] * rls3->f4_err);
    rls3->f4_w[2] = rls3->f4_w[2] + (rls3->f4_k[2] * rls3->f4_err);

    /* P(n) = (1 / forgetting factor) * (P(n-1) - k(n) * u^H(n) * P(n-1)) */
    f4_1_div_fgt = 1.0f / rls3->f4_k_forget;
    f4_up_1 = (f4_in_1 * rls3->f4_p[0][0]) + (f4_in_2 * rls3->f4_p[1][0]) + (f4_in_3 * rls3->f4_p[2][0]);
    f4_up_2 = (f4_in_1 * rls3->f4_p[0][1]) + (f4_in_2 * rls3->f4_p[1][1]) + (f4_in_3 * rls3->f4_p[2][1]);
    f4_up_3 = (f4_in_1 * rls3->f4_p[0][2]) + (f4_in_2 * rls3->f4_p[1][2]) + (f4_in_3 * rls3->f4_p[2][2]);

    rls3->f4_p[0][0] = f4_1_div_fgt * (rls3->f4_p[0][0] - (rls3->f4_k[0] * f4_up_1));
    rls3->f4_p[0][1] = f4_1_div_fgt * (rls3->f4_p[0][1] - (rls3->f4_k[0] * f4_up_2));
    rls3->f4_p[0][2] = f4_1_div_fgt * (rls3->f4_p[0][2] - (rls3->f4_k[0] * f4_up_3));
    rls3->f4_p[1][0] = f4_1_div_fgt * (rls3->f4_p[1][0] - (rls3->f4_k[1] * f4_up_1));
    rls3->f4_p[1][1] = f4_1_div_fgt * (rls3->f4_p[1][1] - (rls3->f4_k[1] * f4_up_2));
    rls3->f4_p[1][2] = f4_1_div_fgt * (rls3->f4_p[1][2] - (rls3->f4_k[1] * f4_up_3));
    rls3->f4_p[2][0] = f4_1_div_fgt * (rls3->f4_p[2][0] - (rls3->f4_k[2] * f4_up_1));
    rls3->f4_p[2][1] = f4_1_div_fgt * (rls3->f4_p[2][1] - (rls3->f4_k[2] * f4_up_2));
    rls3->f4_p[2][2] = f4_1_div_fgt * (rls3->f4_p[2][2] - (rls3->f4_k[2] * f4_up_3));

    /* Output to the specified address */
    f4_weight[0] = rls3->f4_w[0];
    f4_weight[1] = rls3->f4_w[1];
    f4_weight[2] = rls3->f4_w[2];
} /* End of function aidf_rls3_exec */
