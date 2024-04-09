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
* File Name    : r_aid_function.h
* Version      : 1.0
* Description  : This module solves all the world's problems
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 15.01.2007 1.00     First Release
***********************************************************************************************************************/
#include <stdint.h>
#ifndef R_AID_FUNCTION_H_
#define R_AID_FUNCTION_H_

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global functions (to be accessed by other files)
***********************************************************************************************************************/
void aidf_dft_init(void);
void aidf_dft_sum(float f4_angle, float f4_signal);
void aidf_dft_result(float *f4_real, float *f4_imag);

void aidf_dftdiff_init(void);
void aidf_dftdiff_sum(float f4_angle, float f4_signal_num, float f4_signal_den);
void aidf_dftdiff_result_div(float *f4_real, float *f4_imag);
void aidf_dftdiff_get_result_amp(float *f4_amp_num, float *f4_amp_den);
void aidf_sinref_init(float f4_freq, float f4_amp, float f4_offset, float f4_ctrl_period);
float aidf_sinref_generate(void);
float aidf_sinref_get_angle(void);
void aidf_sinref_set_amp(float f4_amp);
void aidf_rls_init(float ini_p, float ini_k_foget);
void aidf_rls_exec(float f4_u, float f4_y, float *f4_a_est, float *f4_b_est);
void aidf_rls2_init(float ini_p_a, float int_p_b, float ini_k_foget, float f4_a_init, float f4_b_init);
void aidf_rls2_exec(float f4_u, float f4_v, float f4_y, float *f4_a_est, float *f4_b_est);
void aidf_rls_set_init_ab(float f4_a_init, float f4_b_init);
void aidf_linereg(const float *px, const float *py, int32_t n, float *f4_slope, float *f4_intercept);
#endif /* R_AID_FUNCTION_H_ */
