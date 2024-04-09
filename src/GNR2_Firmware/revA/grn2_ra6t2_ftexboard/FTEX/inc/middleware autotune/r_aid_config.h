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
* File Name    : r_aid_config_bldc.h
* Version      : 1.0
* Description  : This module solves all the world's problems
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 15.01.2007 1.00     First Release
***********************************************************************************************************************/
#include <stdint.h>
#ifndef R_AID_CONFIG_BLDC_H_
#define R_AID_CONFIG_BLDC_H_

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
/* R_diff */
/* Adjusts voltage to ensure current is within CURRENT1 and CURRENT2 */
#define AID_R_DIFF_VD_STEP              (0.001f)
extern float g_f4_aid_r_diff_vd_step;

#define AID_R_DIFF_MEASURE_CURRENT1     (0.6f)
extern float g_f4_aid_r_diff_measure_current1;
/* Adjusts voltage to ensure current is within CURRENT3 and CURRENT4 */
#define AID_R_DIFF_MEASURE_CURRENT2     (0.65f)
extern float g_f4_aid_r_diff_measure_current2;

#define AID_R_DIFF_MEASURE_CURRENT3     (0.8f)
extern float g_f4_aid_r_diff_measure_current3;
#define AID_R_DIFF_MEASURE_CURRENT4     (0.85f)
extern float g_f4_aid_r_diff_measure_current4;
#define AID_R_DIFF_VD_STEP_WAIT         (0.5f)
extern float g_f4_aid_r_diff_vd_step_wait;
#define AID_R_DIFF_STAB_WAIT            (10.0f)
extern float g_f4_aid_r_diff_stab_wait;
#define AID_R_DIFF_MEASURE_WAIT         (10.0f)
extern float g_f4_aid_r_diff_measure_wait;
#define AID_R_DIFF_MEASURE_TIME         (200.0f)
extern float g_f4_aid_r_diff_measure_time;
#define AID_R_DIFF_RESET_TIME           (10.0f)
extern float g_f4_aid_r_diff_reset_time;
#define AID_R_DIFF_VD_ADJUST_TIMEOUT    (5)
extern uint32_t g_u4_aid_r_diff_vd_adjust_timeout;
/* RLd_DFT */
#define AID_RLD_DFT_FREQ                (200)
extern float g_f4_aid_rld_dft_freq;
#define AID_RLD_DFT_VD_AMP_COEF         (0.1f)
extern float g_f4_aid_rld_dft_vd_amp_coef;
#define AID_RLD_DFT_VD_OFFSET_COEF      (0.3f)
extern float g_f4_aid_rld_dft_vd_offset_coef;
#define AID_RLD_DFT_MEASURE_NUM         (32)
extern float g_f4_aid_rld_dft_measure_num;
#define AID_RLD_DFT_RESET_TIME          (10.0f)
extern float g_f4_aid_rld_dft_reset_time;
#define AID_RLD_DFT_STAB_COEF           (20)
extern float g_f4_aid_rld_dft_stab_coef;

/* RLd_RLS */
#define AID_RLD_RLS_VD_AMP_COEF         (0.1f)
extern float g_f4_aid_rld_rls_vd_amp_coef;
#define AID_RLD_RLS_VD_OFFSET_COEF      (0.3f)
extern float g_f4_aid_rld_rls_vd_offset_coef;
#define AID_RLD_RLS_INIT                (1000)
extern float g_f4_aid_rld_rls_init;
#define AID_RLD_RLS_FORGET_K            (0.9995f)
extern float g_f4_aid_rld_rls_forget_k;
#define AID_RLD_RLS_VD_DC_WAIT_TIME     (100.0f)
extern float g_f4_aid_rld_rls_vd_dc_wait_time;
#define AID_RLD_RLS_MEASURE_OFFSET_TIME (100.0f)
extern float g_f4_aid_rld_rls_measure_offset_time;
#define AID_RLD_RLS_VD_SIN_WAIT_TIME    (200.0f)
extern float g_f4_aid_rld_rls_vd_sin_wait_time;
#define AID_RLD_RLS_MEASURE_NUM         (32)
extern float g_f4_aid_rld_rls_measure_num;
#define AID_RLD_RLS_RESET_TIME          (10.0f)
extern float g_f4_aid_rld_rls_reset_time;

/* RLq_DFT */
#define AID_LQ_DFT_FREQ                 (1000)
extern float g_f4_aid_lq_dft_freq;
#define AID_LQ_DFT_VQ_AMP_COEF          (0.05f)
extern float g_f4_aid_lq_dft_vq_amp_coef;
#define AID_LQ_DFT_VD_OFFSET_COEF       (0.3f)
extern float g_f4_aid_lq_dft_vd_offset_coef;
#define AID_LQ_DFT_MEASURE_NUM          (256)
extern float g_f4_aid_lq_dft_measure_num;
#define AID_LQ_DFT_RESET_TIME           (10.0f)
extern float g_f4_aid_lq_dft_reset_time;

/* Lq_RLS */
#define AID_LQ_RLS_FREQ                 (1000)
extern float g_f4_aid_lq_rls_freq;
#define AID_LQ_RLS_VQ_AMP_COEF          (0.05f)
extern float g_f4_aid_lq_rls_vq_amp_coef;
#define AID_LQ_RLS_VD_OFFSET_COEF       (0.3f)
extern float g_f4_aid_lq_rls_vd_offset_coef;
#define AID_LQ_RLS_INIT                 (1000)
extern float g_f4_aid_lq_rls_init;
#define AID_LQ_RLS_FORGET_K             (0.9995f)
extern float g_f4_aid_lq_rls_forget_k;
#define AID_LQ_RLS_VD_DC_WAIT_TIME      (200.0f)
extern float g_f4_aid_lq_rls_vd_dc_wait_time;
#define AID_LQ_RLS_VQ_SIN_WAIT_TIME     (200.0f)
extern float g_f4_aid_lq_rls_vq_sin_wait_time;
#define AID_LQ_RLS_MEASURE_NUM          (32)
extern float g_f4_aid_lq_rls_measure_num;
#define AID_LQ_RLS_RESET_TIME           (10.0f)
extern float g_f4_aid_lq_rls_reset_time;

/* Ke */
#define AID_KE_ID_UP_TIME               (100.0f)
extern float g_f4_aid_ke_id_up_time;
#define AID_KE_ID_REF_COEF              (0.8f)
extern float g_f4_aid_ke_id_ref_coef;
#define AID_KE_OMEGA_UP_TIME            (AID_KE_FREQ * 80.0f)
extern float g_f4_aid_ke_omega_up_time;
#define AID_KE_MEASURE_TIME             (2000.0f)
extern float g_f4_aid_ke_measure_time;
#define AID_KE_MEASURE_ID_STAB_WAIT     (100.0f)
extern float g_f4_aid_ke_measure_id_stab_wait;
#define AID_KE_FREQ                     (60.0f)
extern float g_f4_aid_ke_freq;
#define AID_KE_RESET_TIME               (1000.0f)
extern float g_f4_aid_ke_reset_time;
#define AID_KE_ASSUMED_INERTIA          (1.0E-6f)
extern float g_f4_aid_ke_assumed_inertia;

/* JD */
#define AID_JD_SPEEDPI_OMEGA            (15.0f)
extern float g_f4_aid_jd_speedpi_omega;
#define AID_JD_ID_REF_COEF              (0.8f)
extern float g_f4_aid_jd_id_ref_coef;
#define AID_JD_RLS_FREQ                 (5.0f)
extern float g_f4_aid_jd_rls_freq;
#define AID_JD_SPEED_AMP_COEF           (0.15f)
extern float g_f4_aid_jd_speed_amp_coef;
#define AID_JD_SPEED_AMP_COEF_MIN       (0.05f)
extern float g_f4_aid_jd_speed_amp_coef_min;
#define AID_JD_SPEED_OFFSET_COEF        (0.5f)
extern float g_f4_aid_jd_speed_offset_coef;
#define AID_JD_RLS_INIT                 (50000)
extern float g_f4_aid_jd_rls_init;
#define AID_JD_RLS_FORGET_K             (0.9995f)
extern float g_f4_aid_jd_rls_forget_k;
#define AID_JD_LIMIT_SPEED_CHANGE       (300.0f)
extern float g_f4_aid_jd_limit_speed_change;
#define AID_JD_DFT_NUM                  (20)
extern float g_f4_aid_jd_dft_num;
#define AID_JD_MEASURE_START_AMP_RATE   (0.97f)
extern float g_f4_aid_jd_measure_start_amp_rate;
#define AID_JD_STAB_WAIT                (500)
extern float g_f4_aid_jd_stab_wait;
#define AID_JD_STOP_TIME                (2000)
extern float g_f4_aid_jd_stop_time;
#define AID_JD_ASSUMED_INERTIA          (1.0E-6f)
extern float g_f4_aid_jd_assumed_inertia;

#define AID_VOLTERR_ENABLE              (1)
extern uint8_t g_u1_aid_volterr_enable;
/* Forgetting factor for RLS, should be within (0,1] */
#define AID_VOLTERR_RLS_FGT_FACTOR      (0.995f)
extern float g_f4_aid_volterr_rls_fgt_factor;
#define AID_VOLTERR_AVG_SAMPLE          (250)
extern float g_f4_aid_volterr_avg_sample;
#define AID_VOLTERR_AVG_SAMPLE_FAST     (100)
extern float g_f4_aid_volterr_avg_sample_fast;
#define AID_VOLTERR_AVG_WAIT_TIME       (25.0f)
extern float g_f4_aid_volterr_avg_wait_time;
#define AID_VOLTERR_POINT_NUM           (64)
extern uint32_t g_u4_aid_volterr_point_num;
#define AID_VOLTERR_MIN_CURRENT_STEP_LSB (2)
extern float g_f4_aid_volterr_min_current_step_lsb;
#define AID_VOLTERR_ITERATION_NUM       (16)
extern uint32_t g_u4_aid_volterr_iteration_num;

/* Upper current limit when identifying R, voltage step reverse when current reach this value*/
#define AID_VOLTERR_RLS_UPPER_CURRENT   (1.0f)
extern float g_f4_aid_volterr_rls_upper_current;

/* Lower current limit when identifying R, voltage step reverse when current reach this value*/
#define AID_VOLTERR_RLS_LOWER_CURRENT   (0.9f)
extern float g_f4_aid_volterr_rls_lower_current;

/* Initial R value */
#define AID_VOLTERR_INIT_R              (0.5f)
extern float g_f4_aid_volterr_init_r;

/* Controller parameters */
#define AID_CURRENT_OMEGA               (400.0f)
extern float g_f4_aid_current_omega;
#define AID_CURRENT_ZETA                (1.0f)
extern float g_f4_aid_current_zeta;
#define AID_E_OBS_OMEGA                 (1000.0f)
extern float g_f4_aid_e_obs_omega;
#define AID_E_OBS_ZETA                  (1.0f)
extern float g_f4_aid_e_obs_zeta;
/* Natural frequency of PLL Speed estimate loop */
#define AID_PLL_EST_OMEGA               (50.0f)
extern float g_f4_aid_pll_est_omega;
/* Damping ratio of PLL Speed estimate loop */
#define AID_PLL_EST_ZETA                (1.0f)
extern float g_f4_aid_pll_est_zeta;
/* Current offset calibration LPF coefficient, the weight for latest value */
#define AID_OFFSET_LPF_K                (1.0f / 64.0f)
extern float g_f4_aid_offset_lpf_k;
/* The time[ms] for waiting LPFed offset value  */
#define AID_OFFSET_CALC_TIME            (512.0f)
extern float g_f4_aid_offset_calc_time;

/*
 * Parameters depend on inverter
 * */

/* Defines the multiplier that, (Over-current limit) = (multiplier) * (rated_currrent(peak)) */
#define AID_OVERCURRENT_LIMIT_MULT      (1.5f)
extern float g_f4_aid_overcurrent_limit_mult;

/*
 * Sensor-less FOC start-up related configurations
 * */
/* Defines multiplier, (target Iq) = (torque current) * (this value) */
#define AID_OL2CL_CRNT_INC_MULT         (2.0f)
extern float g_f4_aid_ol2cl_crnt_inc_mult;

/* Defines the phase error falling rate [rad/s] that pause to raise iq */
#define AID_OL2CL_INCIQ_PHASEERR_DEC_TH (0.5f)
extern float g_f4_aid_ol2cl_inciq_phaseerr_dec_th;

/* Defines the phase error deceleration rate [rad/s] to transition from OL to CL immediately */
#define AID_OL2CL_PHASEERR_MAX_DEC      (10.0f)
extern float g_f4_aid_ol2cl_phaseerr_max_dec;

/* Defines BEMF/Vdc threshold to start transition from OL to CL */
#define AID_OL2CL_BEMF_TH_COEF          (0.30f)
extern float g_f4_aid_ol2cl_bemf_th_coef;

/* Defines BEMF/Vdc threshold to start transition from CL to OL */
#define AID_CL2OL_BEMF_TH_COEF          (0.25f)
extern float g_f4_aid_cl2ol_bemf_th_coef;

/* Defines the maximum speed change rate */
#define AID_SPEED_CHANGE_RATE_LIMIT     (0.125f)
extern float g_f4_aid_speed_change_rate_limit;

/* Defines the slope down time (to 0A) of Iq at CL to OL */
#define AID_IQ_DOWN_TIME                (128.0f)
extern float g_f4_aid_iq_down_time;

/* Defines the ramp up time of Id when starting OL */
#define AID_ID_UP_TIME                  (128.0f)
extern float g_f4_aid_id_up_time;

/* Defines the slope down time (to 0A) of Id at OL to CL */
#define AID_ID_DOWN_TIME                (128.0f)
extern float g_f4_aid_id_down_time;

/* Signal processing configurations */
#define AID_PHASEERR_LPF_BAND_HZ        (10.0f)
extern float g_f4_aid_phaseerr_lpf_band_hz;

/*
 * Sanity check configuration
 */
/* Defines the minimum estimated resistance for the resistance sanity check
 * NOTE: this value is usually the resistance of shunt resistor */
#define AID_R_MIN                       (0.01f)
extern float g_f4_aid_r_min;

/* Defines the maximum pole pairs input that acceptable (do not cause an input error) */
#define AID_NUM_POLE_PAIR_LIMIT         (100)
extern float g_f4_aid_num_pole_pair_limit;

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
typedef void (*VOID_FUNC)(void);
typedef void (*MTR_ID_FUNC)(uint8_t u1_id);
typedef float (*MTR_GET_VDC_FUNC)(uint8_t u1_id);
typedef void (*MTR_GET_CURRENT_IUIW_FUNC)(float *f4_iu_ad, float *f4_iw_ad, uint8_t u1_id);
typedef void (*MTR_INV_SET_UV_FUNC)(float f4_duty_u, float f4_duty_v, float f4_duty_w, uint8_t u1_id);

/***********************************************************************************************************************
Exported global variables
***********************************************************************************************************************/


extern float g_f4_tune_current_setting;
extern uint16_t g_u2_tune_polepairs_setting;
extern float g_f4_tune_inertia_setting;
extern uint16_t g_u2_tune_volterr_setting;
extern uint8_t g_u1_volterr_is_enabled;

extern VOID_FUNC g_fp_aid_internal_clear_oc_flag;
extern MTR_ID_FUNC g_fp_aid_internal_ctrl_start;
extern MTR_ID_FUNC g_fp_aid_internal_ctrl_stop;
extern MTR_GET_VDC_FUNC g_fp_aid_internal_get_vdc;
extern MTR_GET_CURRENT_IUIW_FUNC g_fp_aid_internal_get_current_iuiw;
extern MTR_INV_SET_UV_FUNC g_fp_aid_internal_inv_set_uvw;
extern float g_f4_aid_inv_info_duty_min;
extern float g_f4_aid_inv_info_duty_max;
extern float g_f4_ip_current_limit;
extern float g_f4_aid_inv_info_overvoltage_th;
extern float g_f4_aid_inv_info_undervoltage_th;
extern float g_f4_aid_inv_info_pwm_cycle_s;
extern float g_f4_aid_inv_info_pwm_deadtime_s;
extern float g_f4_mtr_carrier_set_base;
extern float g_f4_ip_current_range;
extern float g_f4_mtr_deadtime_set;
extern uint32_t *g_u4_u_count_reg;
extern uint32_t *g_u4_v_count_reg;
extern uint32_t *g_u4_w_count_reg;

/***********************************************************************************************************************
Exported global functions (to be accessed by other files)
***********************************************************************************************************************/

#endif /* R_AID_CONFIG_H_ */
