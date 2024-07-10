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
* File Name   : r_aid_function.c
* Description : The function of Auto Identify
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : 05.04.2017  Ver.1.00
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
/* Standard library headers */
#include <math.h>
#include "r_aid_function.h"

/***********************************************************************************************************************
* Macro definitions
***********************************************************************************************************************/
#define AIDF_PRV_TWOPI                       (2 * 3.14159265f)         /* 2 * pi */

/* Recursive least squares filter */
static float  gs_f4_y;
static float  gs_f4_u;
static float  gs_f4_a;
static float  gs_f4_b11;
static float  gs_f4_b12;
static float  gs_f4_b21;
static float  gs_f4_b22;
static float  gs_f4_p11;
static float  gs_f4_p12;
static float  gs_f4_p21;
static float  gs_f4_p22;
static float  gs_f4_y_pre;
static float  gs_f4_u_pre;
static float  gs_f4_p11_pre;
static float  gs_f4_p12_pre;
static float  gs_f4_p21_pre;
static float  gs_f4_p22_pre;
static float  gs_f4_k_forget;
static float  gs_f4_k1;
static float  gs_f4_k2;
static float  gs_f4_pu_1;
static float  gs_f4_pu_2;
static float  gs_f4_a_est;
static float  gs_f4_b_est;
static float  gs_f4_a_est_pre;
static float  gs_f4_b_est_pre;
static float  gs_f4_v;
static float  gs_f4_y_est;
static float  gs_f4_err_est;

static float gs_f4_freq;
static float gs_f4_amp;
static float gs_f4_offset;
static float gs_f4_ctrl_period;
static float gs_f4_w_ref;
static float gs_f4_angle_ref;
static float gs_f4_sinref;

static float gs_f4_dft_a_sum;
static float gs_f4_dft_b_sum;
static float gs_f4_dft_cnt;

static float gs_f4_dftdiff_a_sum[2];
static float gs_f4_dftdiff_b_sum[2];
static float gs_f4_dftdiff_cnt;

/***********************************************************************************************************************
* Function Name: aidf_dft_init
* Description  : Initializes DFT module
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void aidf_dft_init(void)
{
    gs_f4_dft_a_sum = 0;
    gs_f4_dft_b_sum = 0;
    gs_f4_dft_cnt = 0;
} /* End of function aidf_dft_init */

/***********************************************************************************************************************
* Function Name: aidf_dft_sum
* Description  : Inputs data/signal to the DFT module
* Arguments    : f4_angle
*                   - The angle of signal in [rad]
*                f4_signal
*                   - The value of signal
* Return Value : None
***********************************************************************************************************************/
void aidf_dft_sum(float f4_angle, float f4_signal)
{
    float f4_sin;
    float f4_cos;

    f4_sin = sinf(f4_angle);
    f4_cos = cosf(f4_angle);
    gs_f4_dft_a_sum += (f4_signal * f4_sin);
    gs_f4_dft_b_sum -= (f4_signal * f4_cos);
    gs_f4_dft_cnt++;
} /* End of function aidf_dft_sum */

/***********************************************************************************************************************
* Function Name: aidf_dft_result
* Description  : Gets the latest result of DFT module
* Arguments    : f4_real
*                   - Where to store real part of the result
*                f4_imag
*                   - Where to store imaginary part of the result
* Return Value : None
***********************************************************************************************************************/
void aidf_dft_result(float *f4_real, float *f4_imag)
{
    *f4_real = (gs_f4_dft_a_sum / gs_f4_dft_cnt) * 2.0f;
    *f4_imag = (gs_f4_dft_b_sum / gs_f4_dft_cnt) * 2.0f;
} /* End of function aidf_dft_result */

/***********************************************************************************************************************
* Function Name: aidf_dftdiff_init
* Description  : Initializes DFT module
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void aidf_dftdiff_init(void)
{
    gs_f4_dftdiff_a_sum[0] = 0;
    gs_f4_dftdiff_a_sum[1] = 0;
    gs_f4_dftdiff_b_sum[0] = 0;
    gs_f4_dftdiff_b_sum[1] = 0;
    gs_f4_dftdiff_cnt = 0;
} /* End of function aidf_dftdiff_init */

/***********************************************************************************************************************
* Function Name: aidf_dftdiff_sum
* Description  : Inputs data/signal to the DFT module
* Arguments    : f4_angle
*                   - The angle of signal in [rad]
*                f4_signal_num
*                   - The value of signal that is numerator (input of system)
*                f4_signal_den
*                   - The value of signal that is denominator (output of system)
* Return Value : None
***********************************************************************************************************************/
void aidf_dftdiff_sum(float f4_angle, float f4_signal_num, float f4_signal_den)
{
    float f4_sin;
    float f4_cos;

    f4_sin = sinf(f4_angle);
    f4_cos = cosf(f4_angle);
    gs_f4_dftdiff_a_sum[0] += (f4_signal_num * f4_sin);
    gs_f4_dftdiff_b_sum[0] += (f4_signal_num * f4_cos);
    gs_f4_dftdiff_a_sum[1] += (f4_signal_den * f4_sin);
    gs_f4_dftdiff_b_sum[1] += (f4_signal_den * f4_cos);
    gs_f4_dftdiff_cnt++;
} /* End of function aidf_dftdiff_sum */

/***********************************************************************************************************************
* Function Name: aidf_dftdiff_result_div
* Description  : Gets the numerator/denominator (input/output) result of DFT module
* Arguments    : f4_real
*                   - Where to store real part of the result
*                f4_imag
*                   - Where to store imaginary part of the result
* Return Value : None
***********************************************************************************************************************/
void aidf_dftdiff_result_div(float *f4_real, float *f4_imag)
{
    float f4_real_num;
    float f4_real_den;
    float f4_imag_num;
    float f4_imag_den;
    float f4_den;

    f4_real_num = (gs_f4_dftdiff_a_sum[0] / gs_f4_dftdiff_cnt) * 2.0f;
    f4_imag_num = (gs_f4_dftdiff_b_sum[0] / gs_f4_dftdiff_cnt) * 2.0f;
    f4_real_den = (gs_f4_dftdiff_a_sum[1] / gs_f4_dftdiff_cnt) * 2.0f;
    f4_imag_den = (gs_f4_dftdiff_b_sum[1] / gs_f4_dftdiff_cnt) * 2.0f;

    f4_den = 1.0f / ((f4_real_den * f4_real_den) + (f4_imag_den * f4_imag_den));
    *f4_real = f4_den * ((f4_real_num * f4_real_den) + (f4_imag_num * f4_imag_den));
    *f4_imag = f4_den * ((f4_imag_num * f4_real_den) - (f4_real_num * f4_imag_den));
} /* End of function aidf_dftdiff_result_div */

/***********************************************************************************************************************
* Function Name: aidf_dftdiff_get_result_amp
* Description  : Gets the amplitude of numerator and denominator
* Arguments    : f4_real
*                   - Where to store real part of the result
*                f4_imag
*                   - Where to store imaginary part of the result
* Return Value : None
***********************************************************************************************************************/
void aidf_dftdiff_get_result_amp(float *f4_amp_num, float *f4_amp_den)
{
    float f4_real_num;
    float f4_real_den;
    float f4_imag_num;
    float f4_imag_den;

    f4_real_num = (gs_f4_dftdiff_a_sum[0] / gs_f4_dftdiff_cnt) * 2.0f;
    f4_imag_num = (gs_f4_dftdiff_b_sum[0] / gs_f4_dftdiff_cnt) * 2.0f;
    f4_real_den = (gs_f4_dftdiff_a_sum[1] / gs_f4_dftdiff_cnt) * 2.0f;
    f4_imag_den = (gs_f4_dftdiff_b_sum[1] / gs_f4_dftdiff_cnt) * 2.0f;

    *f4_amp_num = sqrtf((f4_real_num * f4_real_num) + (f4_imag_num * f4_imag_num));
    *f4_amp_den = sqrtf((f4_real_den * f4_real_den) + (f4_imag_den * f4_imag_den));
} /* End of function aidf_dftdiff_get_result_amp */

/***********************************************************************************************************************
* Function Name: aidf_sinref_init
* Description  : Initializes sine signal function generator, the angle of sine will be reset to 0
* Arguments    : f4_freq
*                   - The frequency of the output signal in [Hz]
*                f4_amp
*                   - The amplitude of the output signal
*                f4_offset
*                   - The offset of the output signal
*                f4_ctrl_period
*                   - The sampling/control period in [s]
* Return Value : None
***********************************************************************************************************************/
void aidf_sinref_init(float f4_freq, float f4_amp, float f4_offset, float f4_ctrl_period)
{
    gs_f4_freq = f4_freq;
    gs_f4_amp = f4_amp;
    gs_f4_offset = f4_offset;
    gs_f4_ctrl_period = f4_ctrl_period;
    gs_f4_w_ref = 0;
    gs_f4_angle_ref = 0;
    gs_f4_sinref = 0;
} /* End of function aidf_sinref_init */

/***********************************************************************************************************************
* Function Name: aidf_sinref_set_amp
* Description  : Set the amplitude of sine signal generator
* Arguments    : f4_freq
*                   - The frequency of the output signal in [Hz]
*                f4_amp
*                   - The amplitude of the output signal
*                f4_offset
*                   - The offset of the output signal
*                f4_ctrl_period
*                   - The sampling/control period in [s]
* Return Value : None
***********************************************************************************************************************/
void aidf_sinref_set_amp(float f4_amp)
{
    gs_f4_amp = f4_amp;
} /* End of function aidf_sinref_set_amp */

/***********************************************************************************************************************
* Function Name: aidf_sinref_generate
* Description  : Increases one step/sample and gets the output value from the sine signal generator
* Arguments    : None
* Return Value : Output value
***********************************************************************************************************************/
float aidf_sinref_generate(void)
{
    float f4_sin;

    gs_f4_w_ref = gs_f4_freq * AIDF_PRV_TWOPI;
    gs_f4_angle_ref += (gs_f4_w_ref * gs_f4_ctrl_period);
    if (gs_f4_angle_ref > AIDF_PRV_TWOPI)
    {
        gs_f4_angle_ref -= AIDF_PRV_TWOPI;
    }
    f4_sin = sinf(gs_f4_angle_ref);
    gs_f4_sinref = (gs_f4_amp * f4_sin) + gs_f4_offset;

    return (gs_f4_sinref);
} /* End of function aidf_sinref_generate */

/***********************************************************************************************************************
* Function Name: aidf_sinref_get_angle
* Description  : Gets angle of current sine signal in [rad]
* Arguments    : None
* Return Value : Current angle [rad]
***********************************************************************************************************************/
float aidf_sinref_get_angle(void)
{
    return (gs_f4_angle_ref);
} /* End of function aidf_sinref_get_angle */

/***********************************************************************************************************************
* Function Name: aidf_rls_init
* Description  : Initializes and reset dual input 2nd order RLS module
* Arguments    : ini_p
*                   - The multiplier to initialize diagonal parameter covariance matrix
*                ini_k_foget
*                   - The forgetting factor within (0, 1], usually 0.98 ~ 1.0, 1.0 means never forget
* Return Value : None
***********************************************************************************************************************/
void aidf_rls_init(float ini_p, float ini_k_foget)
{
    /* Initialization of RLS */
    gs_f4_y         = 0;
    gs_f4_u         = 0;
    gs_f4_a         = 0;
    gs_f4_b11       = 0;
    gs_f4_b12       = 0;
    gs_f4_b21       = 0;
    gs_f4_b22       = 0;
    gs_f4_p11       = ini_p;
    gs_f4_p12       = 0;
    gs_f4_p21       = 0;
    gs_f4_p22       = ini_p;
    gs_f4_y_pre     = 0;
    gs_f4_u_pre     = 0;
    gs_f4_p11_pre   = ini_p;
    gs_f4_p12_pre   = 0;
    gs_f4_p21_pre   = 0;
    gs_f4_p22_pre   = ini_p;
    gs_f4_k_forget  = ini_k_foget;
    gs_f4_pu_1      = 0;
    gs_f4_pu_2      = 0;
    gs_f4_a_est     = 0;
    gs_f4_b_est     = 0;
    gs_f4_a_est_pre = 0;
    gs_f4_b_est_pre = 0;
} /* End of function aidf_rls_init */

/***********************************************************************************************************************
* Function Name: aidf_rls_exec
* Description  : Executes single input 2nd order RLS module to estimate weight coefficients
* Arguments    : f4_u
*                   - The first input, corresponding to the coefficient a,b
*                f4_y
*                   - The measured output
*                f4_a_est
*                   - Where to store the newest estimated coefficient a
*                f4_b_est
*                   - Where to store the newest estimated coefficient b
* Return Value : None
***********************************************************************************************************************/
void aidf_rls_exec(float f4_u, float f4_y, float *f4_a_est, float *f4_b_est)
{
    /* RLS filter */
    gs_f4_u = f4_u;
    gs_f4_y = f4_y;

    gs_f4_a = (gs_f4_y_pre * gs_f4_y_pre * gs_f4_p11_pre) - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p12_pre)
              - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p21_pre) + (gs_f4_u_pre * gs_f4_u_pre * gs_f4_p22_pre);
    gs_f4_b11 = (gs_f4_y_pre * gs_f4_y_pre * gs_f4_p11_pre * gs_f4_p11_pre) - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p11_pre * gs_f4_p12_pre)
                - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p11_pre * gs_f4_p21_pre) + (gs_f4_u_pre * gs_f4_u_pre * gs_f4_p12_pre * gs_f4_p21_pre);
    gs_f4_b12 = (gs_f4_y_pre * gs_f4_y_pre * gs_f4_p11_pre * gs_f4_p12_pre) - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p12_pre * gs_f4_p12_pre)
                - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p11_pre * gs_f4_p22_pre) + (gs_f4_u_pre * gs_f4_u_pre * gs_f4_p12_pre * gs_f4_p22_pre);
    gs_f4_b21 = (gs_f4_y_pre * gs_f4_y_pre * gs_f4_p11_pre * gs_f4_p21_pre) - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p11_pre * gs_f4_p22_pre)
                - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p21_pre * gs_f4_p21_pre) + (gs_f4_u_pre * gs_f4_u_pre * gs_f4_p21_pre * gs_f4_p22_pre);
    gs_f4_b22 = (gs_f4_y_pre * gs_f4_y_pre * gs_f4_p12_pre * gs_f4_p21_pre) - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p12_pre * gs_f4_p22_pre)
                - (gs_f4_y_pre * gs_f4_u_pre * gs_f4_p21_pre * gs_f4_p22_pre) + (gs_f4_u_pre * gs_f4_u_pre * gs_f4_p22_pre * gs_f4_p22_pre);
    gs_f4_p11 = (gs_f4_p11_pre - (gs_f4_b11 / (gs_f4_k_forget + gs_f4_a))) / gs_f4_k_forget;
    gs_f4_p12 = (gs_f4_p12_pre - (gs_f4_b12 / (gs_f4_k_forget + gs_f4_a))) / gs_f4_k_forget;
    gs_f4_p21 = (gs_f4_p21_pre - (gs_f4_b21 / (gs_f4_k_forget + gs_f4_a))) / gs_f4_k_forget;
    gs_f4_p22 = (gs_f4_p22_pre - (gs_f4_b22 / (gs_f4_k_forget + gs_f4_a))) / gs_f4_k_forget;

    gs_f4_pu_1 = ((-gs_f4_y_pre) * gs_f4_p11_pre) + (gs_f4_u_pre * gs_f4_p12_pre);
    gs_f4_pu_2 = ((-gs_f4_y_pre) * gs_f4_p21_pre) + (gs_f4_u_pre * gs_f4_p22_pre);

    gs_f4_y_est = (gs_f4_u_pre * gs_f4_b_est_pre) - (gs_f4_y_pre * gs_f4_a_est_pre);
    gs_f4_a_est = gs_f4_a_est_pre + ((gs_f4_pu_1 / (gs_f4_k_forget + gs_f4_a)) *
            ((gs_f4_y + (gs_f4_y_pre * gs_f4_a_est_pre)) - (gs_f4_u_pre * gs_f4_b_est_pre)));
    gs_f4_b_est = gs_f4_b_est_pre + ((gs_f4_pu_2 / (gs_f4_k_forget + gs_f4_a)) *
            ((gs_f4_y + (gs_f4_y_pre * gs_f4_a_est_pre)) - (gs_f4_u_pre * gs_f4_b_est_pre)));

    gs_f4_u_pre = gs_f4_u;
    gs_f4_y_pre = gs_f4_y;

    gs_f4_p11_pre = gs_f4_p11;
    gs_f4_p12_pre = gs_f4_p12;
    gs_f4_p21_pre = gs_f4_p21;
    gs_f4_p22_pre = gs_f4_p22;
    gs_f4_a_est_pre = gs_f4_a_est;
    gs_f4_b_est_pre = gs_f4_b_est;

    *f4_a_est = gs_f4_a_est;
    *f4_b_est = gs_f4_b_est;
} /* End of function aidf_rls_exec */

/***********************************************************************************************************************
 * Function Name: aidf_rls2_init
 * Description  : Does an example task. Making this longer just to see how it wraps. Making this long just to see how it
 *                wraps.
 * Arguments    : index -
 *                    Where to start looking
 *                p_output -
 *                    Pointer of where to put the output data
 * Return Value : count -
 *                    How many entries were found
 **********************************************************************************************************************/
void aidf_rls2_init(float ini_p_a, float int_p_b, float ini_k_foget, float f4_a_init, float f4_b_init)
{
    /* Initialization of RLS */
    gs_f4_y         = 0;
    gs_f4_u         = 0;
    gs_f4_v         = 0;
    gs_f4_a         = 0;
    gs_f4_b11       = 0;
    gs_f4_b12       = 0;
    gs_f4_b21       = 0;
    gs_f4_b22       = 0;
    gs_f4_p11       = ini_p_a;
    gs_f4_p12       = 0;
    gs_f4_p21       = 0;
    gs_f4_p22       = int_p_b;
    gs_f4_y_pre     = 0;
    gs_f4_u_pre     = 0;
    gs_f4_p11_pre   = ini_p_a;
    gs_f4_p12_pre   = 0;
    gs_f4_p21_pre   = 0;
    gs_f4_p22_pre   = int_p_b;
    gs_f4_k_forget  = ini_k_foget;
    gs_f4_pu_1       = 0;
    gs_f4_pu_2       = 0;
    gs_f4_a_est     = 0;
    gs_f4_b_est     = 0;
    gs_f4_a_est_pre = f4_a_init;
    gs_f4_b_est_pre = f4_b_init;
} /* End of function aidf_rls2_init */

/***********************************************************************************************************************
* Function Name: aidf_rls2_exec
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
void aidf_rls2_exec(float f4_a_in, float f4_b_in, float f4_y, float *f4_a_est, float *f4_b_est)
{
    float f4_upu;
    float f4_up_1;
    float f4_up_2;
    float f4_temp;
    float f4_1_div_fgt;

    /* RLS identification
     * Input vector                     : u(n) = [f4_a_in; f4_b_in]
     * Desired output(measured output)  : d(n) = gs_f4_y
     * Estimated output                 : y(n) = gs_f4_y_est
     * See https://jp.mathworks.com/help/dsp/ref/dsp.rlsfilter-system-object.html for detailed algorithm
     * */
    gs_f4_v = f4_a_in;
    gs_f4_u = f4_b_in;

    gs_f4_y = f4_y;

    /* Pu(n) = P(n - 1) * u(n) */
    gs_f4_pu_1 = (gs_f4_p11 * gs_f4_v) + (gs_f4_p12 * gs_f4_u);
    gs_f4_pu_2 = (gs_f4_p21 * gs_f4_v) + (gs_f4_p22 * gs_f4_u);

    /* uPu(n) = u^H(n) * Pu(n), u(n) is all real number, Hermitian transpose = regular transpose  */
    f4_upu = (gs_f4_v * gs_f4_pu_1) + (gs_f4_u * gs_f4_pu_2);

    /* k(n) = Pu(n) / (forgetting factor + uPu(n)) , f4_temp = 1 / (forgetting factor + uPu(n))*/
    f4_temp = 1.0f / (gs_f4_k_forget + f4_upu);
    gs_f4_k1 = gs_f4_pu_1 * f4_temp;
    gs_f4_k2 = gs_f4_pu_2 * f4_temp;

    /* y(n) = w^T(n - 1) * u(n) */
    gs_f4_y_est = (gs_f4_v * gs_f4_a_est_pre) + (gs_f4_u * gs_f4_b_est_pre);

    /* e(n) = d(n) - y(n) */
    gs_f4_err_est = gs_f4_y - gs_f4_y_est;

    /* w(n) = w(n-1) + k(n)  * (d(n) - y(n)) */
    gs_f4_a_est = gs_f4_a_est_pre + (gs_f4_k1 * gs_f4_err_est);
    gs_f4_b_est = gs_f4_b_est_pre + (gs_f4_k2 * gs_f4_err_est);

    /* P(n) = (1 / forgetting factor) * (P(n-1) - k(n) * u^H(n) * P(n-1)) */
    f4_1_div_fgt = 1.0f / gs_f4_k_forget;
    f4_up_1 = (gs_f4_v * gs_f4_p11) + (gs_f4_u * gs_f4_p21);
    f4_up_2 = (gs_f4_v * gs_f4_p12) + (gs_f4_u * gs_f4_p22);

    gs_f4_p11 = f4_1_div_fgt * (gs_f4_p11 - (gs_f4_k1 * f4_up_1));
    gs_f4_p12 = f4_1_div_fgt * (gs_f4_p12 - (gs_f4_k1 * f4_up_2));
    gs_f4_p21 = f4_1_div_fgt * (gs_f4_p21 - (gs_f4_k2 * f4_up_1));
    gs_f4_p22 = f4_1_div_fgt * (gs_f4_p22 - (gs_f4_k2 * f4_up_2));

    gs_f4_u_pre = gs_f4_u;

    gs_f4_a_est_pre = gs_f4_a_est;
    gs_f4_b_est_pre = gs_f4_b_est;

    *f4_a_est = gs_f4_a_est;
    *f4_b_est = gs_f4_b_est;
} /* End of function aidf_rls2_exec */

/***********************************************************************************************************************
* Function Name : aidf_rls_set_init_ab
* Description   : Set the initial weight coefficient of RLS module
* Arguments     : f4_a_init - Initial value of coefficient a
*                 f4_b_init - Initial value of coefficient b
* Return Value  : None
***********************************************************************************************************************/
void aidf_rls_set_init_ab(float f4_a_init, float f4_b_init)
{
    gs_f4_a_est_pre = f4_a_init;
    gs_f4_b_est_pre = f4_b_init;
} /* End of function aidf_rls_set_init_ab */

/***********************************************************************************************************************
* Function Name : aidf_linereg
* Description   : Linear regression, calculate slope and intercept of the linear fitting of given points
* Arguments     : px - Start address of horizontal axis points
*                 py - Start address of vertical axis points
*                 n - Number of points
*                 f4_slope - Where to store the slope information
*                 f4_intercept - Where to store the intercept information
* Return Value  : None
***********************************************************************************************************************/
void aidf_linereg(const float *px, const float *py, int32_t n, float *f4_slope, float *f4_intercept)
{
    int32_t i = 0;
    float x_sum = 0.0f;
    float y_sum = 0.0f;
    float cov_xy_sum = 0.0f;    /* Accumulator for calculating covariance of x and y */
    float x_var_sum = 0.0f;
    float x_mean;
    float y_mean;
    float f4_x_diff;
    float f4_y_diff;

    /* Calculate mean */
    for(i = 0; i < n; i++)
    {
        x_sum += px[i];
        y_sum += py[i];
    }
    x_mean = (x_sum / (float)n);
    y_mean = (y_sum / (float)n);

    /* Calculate covariance and variance of x (not divided by number of samples) */
    for(i = 0; i < n; i++)
    {
        f4_x_diff = px[i] - x_mean;
        f4_y_diff = py[i] - y_mean;
        x_var_sum += (f4_x_diff * f4_x_diff);
        cov_xy_sum += (f4_x_diff * f4_y_diff);
    }

    /* Calculate slope and intercept */
    *f4_slope = cov_xy_sum / x_var_sum;
    *f4_intercept = y_mean - (x_mean * (*f4_slope));
} /* End of function aidf_linereg */
