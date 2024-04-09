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
* File Name   : r_aid_auto_identify.c
* Description : The processing of Auto Identify
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : 05.04.2017  Ver.1.00
***********************************************************************************************************************/
/*******************************************************************************************************************//**
 * @ingroup Tuner_Interface
 * @defgroup AUTO_IDENTIFY STM Tuner Library API
 * @brief STM tuner auto identification module
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
/* Standard library headers */
//#include <machine.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

/* Main associated headers */
#include "r_aid_auto_identify.h"

/* Configuration */
#include "r_aid_config.h"

/* Submodules */
#include "r_aid_core.h"
#include "r_aid_function.h"
#include "r_aid_volterr.h"
#include "r_aid_rdiff.h"
#include "r_aid_ldq.h"
#include "r_aid_ke.h"
#include "r_aid_jd.h"

/***********************************************************************************************************************
* Macro definitions
***********************************************************************************************************************/
#ifndef AID_PRV_FILE_CODE
#define AID_PRV_FILE_CODE (100)
#endif

#define AID_PRV_EVENT_COMPLETED (5U)        /**< The internal COMPLETED event */
#define AID_PRV_EVENT_ERROR     (6U)        /**< The internal ERROR event */

/***********************************************************************************************************************
 Private global variables and functions
 **********************************************************************************************************************/
static float gs_f4_cfg_init_r;              /**< The initial R, if 0 the respective identification will be skipped */
static float gs_f4_cfg_init_ld;             /**< The initial Ld, if 0 the respective identification will be skipped */
static float gs_f4_cfg_init_lq;             /**< The initial Lq, if 0 the respective identification will be skipped */
static float gs_f4_cfg_init_ke;             /**< The initial Ke, if 0 the respective identification will be skipped */

/* Function prototypes of static functions */
static void     aid_state_machine(uint16_t event);
static void     aid_reset(void);
static void     aid_revert_to_lastest_paramode(uint16_t *p_paramode);
static void     aid_motorid_sequence(void);
static void     aid_motorid_sequence_post(void);
static void     aid_act_tune_init(void);
static void     aid_act_tune_end(void);
static uint16_t aid_get_next_paramode(void);
static float    aid_design_base_speed(float f4_flux_wb, float f4_r, float f4_lq, float f4_va_max, float f4_ia_max);

/***********************************************************************************************************************
 Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/
/* WARNING These variables should be accessed only through RAM monitor tool (such as RMW GUI)
 * DO NOT access directly through "extern", use the API instead
 */
uint16_t    aid_u2_para_mode;               /**< The mode code that indicates which method is in progress*/
uint16_t    aid_u2_tune_status;             /**< The status code that indicates the tuning status */

/* Variables directly accessible from RMW (read/write) */
float       gui_f4_tune_current_setting;    /**< The rated current */
uint16_t    gui_u2_tune_polepairs_setting;  /**< The number of pole pairs */
float       gui_f4_tune_inertia_setting;    /**< The inertia identification range setting (0~1) */
uint16_t    gui_u2_tune_volterr_setting;    /**< The current step of voltage error identification */
uint8_t     gui_u1_volterr_is_enabled;      /**< Whether enable the voltage error identification */

/* Variables that is read only for RMW */
float       aid_f4_rated_speed_elec_rad;    /**< The rated speed (electrical) in [rad/s] */
float       aid_f4_rated_speed_mech_rpm;    /**< The rated speed (mechanical) in [rpm] */
float       aid_f4_rated_torque_nm;         /**< The rated torque [Nm] */
float       aid_f4_total_time_elapsed;      /**< Indicates how long time elapsed from starting identification */
float       aid_f4_rated_current_max;       /**< The maximum rated current [Arms] calculated from inverter parameter */

float g_f4_aid_r_diff_vd_step = AID_R_DIFF_VD_STEP;
float g_f4_aid_r_diff_measure_current1 = AID_R_DIFF_MEASURE_CURRENT1;
float g_f4_aid_r_diff_measure_current2 = AID_R_DIFF_MEASURE_CURRENT2;
float g_f4_aid_r_diff_measure_current3 = AID_R_DIFF_MEASURE_CURRENT3;
float g_f4_aid_r_diff_measure_current4 = AID_R_DIFF_MEASURE_CURRENT4;
float g_f4_aid_r_diff_vd_step_wait = AID_R_DIFF_VD_STEP_WAIT;
float g_f4_aid_r_diff_stab_wait = AID_R_DIFF_STAB_WAIT;
float g_f4_aid_r_diff_measure_wait = AID_R_DIFF_MEASURE_WAIT;
float g_f4_aid_r_diff_measure_time = AID_R_DIFF_MEASURE_TIME;
float g_f4_aid_r_diff_reset_time = AID_R_DIFF_RESET_TIME;
uint32_t g_u4_aid_r_diff_vd_adjust_timeout = AID_R_DIFF_VD_ADJUST_TIMEOUT;
float g_f4_aid_rld_dft_freq = AID_RLD_DFT_FREQ;
float g_f4_aid_rld_dft_vd_amp_coef = AID_RLD_DFT_VD_AMP_COEF;
float g_f4_aid_rld_dft_vd_offset_coef = AID_RLD_DFT_VD_OFFSET_COEF;
float g_f4_aid_rld_dft_measure_num = AID_RLD_DFT_MEASURE_NUM;
float g_f4_aid_rld_dft_reset_time = AID_RLD_DFT_RESET_TIME;
float g_f4_aid_rld_dft_stab_coef = AID_RLD_DFT_STAB_COEF;
uint32_t g_u4_aid_rld_rls_freq;
float g_f4_aid_rld_rls_vd_amp_coef = AID_RLD_RLS_VD_AMP_COEF;
float g_f4_aid_rld_rls_vd_offset_coef = AID_RLD_RLS_VD_OFFSET_COEF;
float g_f4_aid_rld_rls_init = AID_RLD_RLS_INIT;
float g_f4_aid_rld_rls_forget_k = AID_RLD_RLS_FORGET_K;
float g_f4_aid_rld_rls_vd_dc_wait_time = AID_RLD_RLS_VD_DC_WAIT_TIME;
float g_f4_aid_rld_rls_measure_offset_time = AID_RLD_RLS_MEASURE_OFFSET_TIME;
float g_f4_aid_rld_rls_vd_sin_wait_time = AID_RLD_RLS_VD_SIN_WAIT_TIME;
float g_f4_aid_rld_rls_measure_num = AID_RLD_RLS_MEASURE_NUM;
float g_f4_aid_rld_rls_reset_time = AID_RLD_RLS_RESET_TIME;
float g_f4_aid_lq_dft_freq = AID_LQ_DFT_FREQ;
float g_f4_aid_lq_dft_vq_amp_coef = AID_LQ_DFT_VQ_AMP_COEF;
float g_f4_aid_lq_dft_vd_offset_coef = AID_LQ_DFT_VD_OFFSET_COEF;
float g_f4_aid_lq_dft_measure_num = AID_LQ_DFT_MEASURE_NUM;
float g_f4_aid_lq_dft_reset_time = AID_LQ_DFT_RESET_TIME;
float g_f4_aid_lq_rls_freq = AID_LQ_RLS_FREQ;
float g_f4_aid_lq_rls_vq_amp_coef = AID_LQ_RLS_VQ_AMP_COEF;
float g_f4_aid_lq_rls_vd_offset_coef = AID_LQ_RLS_VD_OFFSET_COEF;
float g_f4_aid_lq_rls_init = AID_LQ_RLS_INIT;
float g_f4_aid_lq_rls_forget_k = AID_LQ_RLS_FORGET_K;
float g_f4_aid_lq_rls_vd_dc_wait_time = AID_LQ_RLS_VD_DC_WAIT_TIME;
float g_f4_aid_lq_rls_vq_sin_wait_time = AID_LQ_RLS_VQ_SIN_WAIT_TIME;
float g_f4_aid_lq_rls_measure_num = AID_LQ_RLS_MEASURE_NUM;
float g_f4_aid_lq_rls_reset_time = AID_LQ_RLS_RESET_TIME;
float g_f4_aid_ke_id_up_time = AID_KE_ID_UP_TIME;
float g_f4_aid_ke_id_ref_coef = AID_KE_ID_REF_COEF;
float g_f4_aid_ke_omega_up_time = AID_KE_OMEGA_UP_TIME;
float g_f4_aid_ke_measure_time = AID_KE_MEASURE_TIME;
float g_f4_aid_ke_measure_id_stab_wait = AID_KE_MEASURE_ID_STAB_WAIT;
float g_f4_aid_ke_freq = AID_KE_FREQ;
float g_f4_aid_ke_reset_time = AID_KE_RESET_TIME;
float g_f4_aid_ke_assumed_inertia = AID_KE_ASSUMED_INERTIA;
float g_f4_aid_jd_speedpi_omega = AID_JD_SPEEDPI_OMEGA;
float g_f4_aid_jd_id_ref_coef = AID_JD_ID_REF_COEF;
float g_f4_aid_jd_rls_freq = AID_JD_RLS_FREQ;
float g_f4_aid_jd_speed_amp_coef = AID_JD_SPEED_AMP_COEF;
float g_f4_aid_jd_speed_amp_coef_min = AID_JD_SPEED_AMP_COEF_MIN;
float g_f4_aid_jd_speed_offset_coef = AID_JD_SPEED_OFFSET_COEF;
float g_f4_aid_jd_rls_init = AID_JD_RLS_INIT;
float g_f4_aid_jd_rls_forget_k = AID_JD_RLS_FORGET_K;
float g_f4_aid_jd_limit_speed_change = AID_JD_LIMIT_SPEED_CHANGE;
float g_f4_aid_jd_dft_num = AID_JD_DFT_NUM;
float g_f4_aid_jd_measure_start_amp_rate = AID_JD_MEASURE_START_AMP_RATE;
float g_f4_aid_jd_stab_wait = AID_JD_STAB_WAIT;
float g_f4_aid_jd_stop_time = AID_JD_STOP_TIME;
float g_f4_aid_jd_assumed_inertia = AID_JD_ASSUMED_INERTIA;
uint8_t g_u1_aid_volterr_enable = AID_VOLTERR_ENABLE;
float g_f4_aid_volterr_rls_fgt_factor = AID_VOLTERR_RLS_FGT_FACTOR;
float g_f4_aid_volterr_avg_sample = AID_VOLTERR_AVG_SAMPLE;
float g_f4_aid_volterr_avg_sample_fast = AID_VOLTERR_AVG_SAMPLE_FAST;
float g_f4_aid_volterr_avg_wait_time = AID_VOLTERR_AVG_WAIT_TIME;
uint32_t g_u4_aid_volterr_point_num = AID_VOLTERR_POINT_NUM;
float g_f4_aid_volterr_min_current_step_lsb = AID_VOLTERR_MIN_CURRENT_STEP_LSB;
uint32_t g_u4_aid_volterr_iteration_num = AID_VOLTERR_ITERATION_NUM;
float g_f4_aid_volterr_rls_upper_current = AID_VOLTERR_RLS_UPPER_CURRENT;
float g_f4_aid_volterr_rls_lower_current = AID_VOLTERR_RLS_LOWER_CURRENT;
float g_f4_aid_volterr_init_r = AID_VOLTERR_INIT_R;
float g_f4_aid_current_omega = AID_CURRENT_OMEGA;
float g_f4_aid_current_zeta = AID_CURRENT_ZETA;
float g_f4_aid_e_obs_omega = AID_E_OBS_OMEGA;
float g_f4_aid_e_obs_zeta = AID_E_OBS_ZETA;
float g_f4_aid_pll_est_omega = AID_PLL_EST_OMEGA;
float g_f4_aid_pll_est_zeta = AID_PLL_EST_ZETA;
float g_f4_aid_offset_lpf_k = AID_OFFSET_LPF_K;
float g_f4_aid_offset_calc_time = AID_OFFSET_CALC_TIME;
float g_f4_aid_overcurrent_limit_mult = AID_OVERCURRENT_LIMIT_MULT;
float g_f4_aid_ol2cl_crnt_inc_mult = AID_OL2CL_CRNT_INC_MULT;
float g_f4_aid_ol2cl_inciq_phaseerr_dec_th = AID_OL2CL_INCIQ_PHASEERR_DEC_TH;
float g_f4_aid_ol2cl_phaseerr_max_dec = AID_OL2CL_PHASEERR_MAX_DEC;
float g_f4_aid_ol2cl_bemf_th_coef = AID_OL2CL_BEMF_TH_COEF;
float g_f4_aid_cl2ol_bemf_th_coef = AID_CL2OL_BEMF_TH_COEF;
float g_f4_aid_speed_change_rate_limit = AID_SPEED_CHANGE_RATE_LIMIT;
float g_f4_aid_iq_down_time = AID_IQ_DOWN_TIME;
float g_f4_aid_id_up_time = AID_ID_UP_TIME;
float g_f4_aid_id_down_time = AID_ID_DOWN_TIME;
float g_f4_aid_phaseerr_lpf_band_hz = AID_PHASEERR_LPF_BAND_HZ;
float g_f4_aid_r_min = AID_R_MIN;
float g_f4_aid_num_pole_pair_limit = AID_NUM_POLE_PAIR_LIMIT;

float g_f4_tune_current_setting;
uint16_t g_u2_tune_polepairs_setting;
float g_f4_tune_inertia_setting;
uint16_t g_u2_tune_volterr_setting;
float g_f4_tune_current_setting;
uint8_t g_u1_volterr_is_enabled;

VOID_FUNC g_fp_aid_internal_clear_oc_flag;
MTR_ID_FUNC g_fp_aid_internal_ctrl_start;
MTR_ID_FUNC g_fp_aid_internal_ctrl_stop;
MTR_GET_VDC_FUNC g_fp_aid_internal_get_vdc;
MTR_GET_CURRENT_IUIW_FUNC g_fp_aid_internal_get_current_iuiw;
MTR_INV_SET_UV_FUNC g_fp_aid_internal_inv_set_uvw;
float g_f4_aid_inv_info_duty_min;
float g_f4_aid_inv_info_duty_max;
float g_f4_ip_current_limit;
float g_f4_aid_inv_info_overvoltage_th;
float g_f4_aid_inv_info_undervoltage_th;
float g_f4_aid_inv_info_pwm_cycle_s;
float g_f4_aid_inv_info_pwm_deadtime_s;
float g_f4_mtr_carrier_set_base;
float g_f4_ip_current_range;
float g_f4_mtr_deadtime_set;
uint32_t *g_u4_u_count_reg;
uint32_t *g_u4_v_count_reg;
uint32_t *g_u4_w_count_reg;

/*******************************************************************************************************************//**
 * Implementation of on error event in CORE module
 * @param[in] u2_error_code  The error code @ref ERROR_CODE
 **********************************************************************************************************************/
void aid_core_on_error(uint16_t u2_error_code)
{
    if(u2_error_code)
    {
        //Dummy if for removing warning
    }
    aid_u2_tune_status = AID_STATUS_ERROR;
} /* End of function aid_core_on_error */

/*******************************************************************************************************************//**
 * @brief     Initialize Tuner with interrupt tick settings
 * @pre       Any initialization (such as initialization of peripherals) required by driver should be executed before
 *            calling this function.
 * @warning   Execute this function before calling any other API function
 * @param[in] pwm_tick_per_irq   The number of PWM cycle per interrupt tick
 * @param[in] speed_ctrl_period  The speed control period[s]
 **********************************************************************************************************************/
void R_AID_Init(uint8_t pwm_tick_per_irq, float speed_ctrl_period)
{
    if((pwm_tick_per_irq >= 1) && (speed_ctrl_period > 0.0f))
    {
        aid_core_init(pwm_tick_per_irq, speed_ctrl_period, aid_motorid_sequence, aid_motorid_sequence_post);

        gui_f4_tune_inertia_setting   = g_f4_tune_inertia_setting;
        gui_u2_tune_volterr_setting   = g_u2_tune_volterr_setting;
        gui_u1_volterr_is_enabled     = g_u1_volterr_is_enabled;
#if (AID_MOTOR_TYPE == AID_MOTOR_TYPE_BLDC)
        aid_f4_rated_current_max      = aid_f4_overcurrent_limit_hw / (AID_SQRT_2 * g_f4_aid_overcurrent_limit_mult);
#elif (AID_MOTOR_TYPE == AID_MOTOR_TYPE_STM)
        aid_f4_rated_current_max      = aid_f4_overcurrent_limit_hw / (g_f4_aid_overcurrent_limit_mult);
#endif
    }
    
    aid_volterr_init();
} /* End of function R_AID_Init */

/*******************************************************************************************************************//**
 * @brief Send start command
 **********************************************************************************************************************/
void R_AID_CmdStart(void)
{
    aid_state_machine(AID_COMMAND_START);
} /* End of function R_AID_CmdStart */

/*******************************************************************************************************************//**
 * @brief Send stop command
 **********************************************************************************************************************/
void R_AID_CmdStop(void)
{
    aid_state_machine(AID_COMMAND_STOP);
} /* End of function R_AID_CmdStop */

/*******************************************************************************************************************//**
 * @brief Send reset command
 **********************************************************************************************************************/
void R_AID_CmdReset(void)
{
    aid_state_machine(AID_COMMAND_RESET);
} /* End of function R_AID_CmdReset */

/*******************************************************************************************************************//**
 * @brief Send resume command
 **********************************************************************************************************************/
void R_AID_CmdResume(void)
{
    aid_state_machine(AID_COMMAND_RESUME);
} /* End of function R_AID_CmdResume */

/*******************************************************************************************************************//**
 * @brief This is a function for GUI
 * @param[in] cmd_code  The command code
 **********************************************************************************************************************/
void R_AID_CmdByCode(uint16_t cmd_code)
{
    aid_state_machine(cmd_code);
} /* End of function R_AID_CmdByCode */

/*******************************************************************************************************************//**
 * @brief     Generate user customizable error to stop the identification process with given error code
 * @param[in] u2_error_code  The error code
 **********************************************************************************************************************/
void R_AID_UserError(uint16_t u2_error_code)
{
    if (u2_error_code != 0)
    {
        aid_core_throw_error(u2_error_code);
    }
} /* End of function R_AID_UserError */

/*******************************************************************************************************************//**
 * @brief     Sets motor plate information includes rated current and number of pole pairs
 *
 * @param[in] f4_rated_current      The rated current
 * @param[in] u2_num_of_pole_pair   The number pole pair
 **********************************************************************************************************************/
void R_AID_ConfigMotorPlate(float f4_rated_current, uint16_t u2_num_of_pole_pair)
{
    /* These parameter will be checked in aid_tune_act_init */
    gui_f4_tune_current_setting = f4_rated_current;
    gui_u2_tune_polepairs_setting = u2_num_of_pole_pair;
} /* End of function R_AID_ConfigMotorPlate */

/*******************************************************************************************************************//**
 * @brief Enable the voltage error identification for next parameter identification
 **********************************************************************************************************************/
void R_AID_ConfigEnableVolterrID(void)
{
    gui_u1_volterr_is_enabled = true;
} /* End of function R_AID_ConfigEnableVolterrID */

/*******************************************************************************************************************//**
 * @brief Disable the voltage error identification for next parameter identification
 **********************************************************************************************************************/
void R_AID_ConfigDisableVolterrID(void)
{
    gui_u1_volterr_is_enabled = false;
} /* End of function R_AID_ConfigDisableVolterrID */

/*******************************************************************************************************************//**
 * @brief     Sets the voltage step in voltage error measurement
 * @details   Use this option to sets the current step of voltage error measurement, a small step size will improve
 *            accuracy of the measurement, however may be insufficient to cover the unsaturated region of voltage error
 *            @par The actual step size in [A] will be determined by multiply this option with the current resolution
 *            set in the driver.
 * @param[in] u1_num_of_crnt_step  The number of current step
 **********************************************************************************************************************/
void R_AID_ConfigSetVolterrCrntStep(uint8_t u1_num_of_crnt_step)
{
    /* These parameter will be checked in aid_tune_act_init */
    gui_u2_tune_volterr_setting = u1_num_of_crnt_step;
} /* End of function R_AID_ConfigSetVolterrCrntStep */

/*******************************************************************************************************************//**
 * @brief     Sets the inertia range in a relative value
 * @details   This option is for preventing over-voltage in inertia identification, in the case of 0,
 *            the tuner will assume that the inertia is light and identify it with large signal,
 *            in contract the case of 1, the tuner will assume that the inertia is very heavy and identify it with signal
 *            as small as possible to prevent over-voltage due to the BEMF.
 *            @par The default value 0 will be sufficient for most case.
 * @param[in] f4_inertia_range  The inertia range (0~1, default:0)
 **********************************************************************************************************************/
void R_AID_ConfigSetInertiaRange(float f4_inertia_range)
{
    /* These parameter will be checked in aid_tune_act_init */
    gui_f4_tune_inertia_setting = f4_inertia_range;
} /* End of function R_AID_ConfigSetInertiaRange */

/*******************************************************************************************************************//**
 * @brief     Sets the assumed inertia to ensure stability of speed control loop
 * @details   This assumed inertia is used to calculate the gains for speed control loop which is crucial for flux and
 *            inertia identification.
 *            Too large assumed inertia will increase the gain of speed controller so may cause over-voltage or
 *            even unstable speed control. Too small assumed inertia may cause FOC start-up failure.
 *            Increase this parameter if the FOC start-up fails with a motor with relatively big inertia.
 *            Decrease this parameter if the FOC start-up fails with a motor with relatively small inertia.
 *            @par The default value will be sufficient for small motors.
 * @param[in] f4_inertia The assumed inertia [kgm/s^2](default:1E-6);
 **********************************************************************************************************************/
void R_AID_ConfigSetAssumedInertia(float f4_inertia)
{
    aid_ke_config_assumed_inertia(f4_inertia);
    aid_jd_config_assumed_inertia(f4_inertia);
} /* End of function R_AID_ConfigSetAssumedInertia */

/*******************************************************************************************************************//**
 * @brief Sets lookup table for the voltage error compensation
 * @param st_lut  The data structure of lookup table for handling voltage error
 **********************************************************************************************************************/
void R_AID_ConfigSetVolterrLUT(st_aid_volterr_lut_t *st_lut)
{
    if (NULL != st_lut)
    {
        aid_core_set_volterr_table(st_lut->current_table,
                                   st_lut->volterr_table,
                                   st_lut->ref_voltage);
    }
} /* End of function R_AID_ConfigSetVolterrLUT */

/*******************************************************************************************************************//**
 * @brief     Sets known motor electrical parameters to skip related identification process, exact 0.0 will be treat as
 *            unknown motor parameter
 * @pre       All arguments must be finite positive or exact 0.0
 *
 * @param[in] f4_r   The known resistance[ohm]
 * @param[in] f4_ld  The known d-axis inductance [H]
 * @param[in] f4_lq  The known q-axis inductance [H]
 * @param[in] f4_ke  The known BEMF constant (rated flux) [Wb]
 *
 * @return    The fault return code @see FaultRet
 **********************************************************************************************************************/
int32_t R_AID_SetInitElecParams(float f4_r, float f4_ld, float f4_lq, float f4_ke)
{
    int32_t ret_err = 0;

    if ((false == isfinite(f4_r)) || (f4_r < 0.0f))
    {
        ret_err = AID_FAULT_PARAM_R;
    }
    if ((false == isfinite(f4_ld)) || (f4_ld < 0.0f))
    {
        ret_err = AID_FAULT_PARAM_LD;
    }
    if ((false == isfinite(f4_lq)) || (f4_lq < 0.0f))
    {
        ret_err = AID_FAULT_PARAM_LQ;
    }
    if ((false == isfinite(f4_ke)) || (f4_ke < 0.0f))
    {
        ret_err = AID_FAULT_PARAM_KE;
    }

    if (0 == ret_err)
    {
        gs_f4_cfg_init_r  = f4_r;
        gs_f4_cfg_init_ld = f4_ld;
        gs_f4_cfg_init_lq = f4_lq;
        gs_f4_cfg_init_ke = f4_ke;
    }

    return ret_err;
} /* End of function R_AID_SetInitElecParams */

/*******************************************************************************************************************//**
 * @brief Gets the version information of firmware
 * @param p_major_version  The pointer to the variable to store major version
 * @param p_minor_version  The pointer to the variable to store minor version
 **********************************************************************************************************************/
void R_AID_GetVersionInfo(uint16_t *p_major_version, uint16_t *p_minor_version)
{
    if ((NULL != p_major_version) && (NULL != p_minor_version))
    {
        *p_major_version = AID_API_MAJOR_VERSION;
        *p_minor_version = AID_API_MINOR_VERSION;
    }
} /* End of function R_AID_GetVersionInfo */

/*******************************************************************************************************************//**
 * @brief  Gets period of current control
 * @return Current control period [s]
 **********************************************************************************************************************/
float R_AID_GetCurrentCtrlPeriod(void)
{
    return (aid_f4_ctrl_period_ms * 0.001f);
} /* End of function R_AID_GetCurrentCtrlPeriod */

/*******************************************************************************************************************//**
 * @brief  Gets period of speed control
 * @return Speed control period [s]
 **********************************************************************************************************************/
float R_AID_GetSpeedCtrlPeriod(void)
{
    return (aid_f4_spd_ctrl_period_ms * 0.001f);
} /* End of function R_AID_GetSpeedCtrlPeriod */

/*******************************************************************************************************************//**
 * @brief  Gets period of PWM carrier
 * @return PWM carrier period [s]
 **********************************************************************************************************************/
float R_AID_GetPWMPeriod(void)
{
    return aid_f4_pwm_period_ms * 0.001f;
} /* End of function R_AID_GetPWMPeriod */

/*******************************************************************************************************************//**
 * @brief  Get the status of Tuner system
 * @retval 0 AID_STATUS_READY       Ready for starting parameter identification
 * @retval 1 AID_STATUS_MEASURE     Measuring (parameter identifying)
 * @retval 2 AID_STATUS_ERROR       Error occurred, see the error code by R_AID_GetErrorCode to identify the cause
 * @retval 3 AID_STATUS_RESET       Wait for system resetting
 * @retval 4 AID_STATUS_COMPLETED   The identification is completed
 **********************************************************************************************************************/
uint16_t R_AID_GetSystemStatus(void)
{
    return aid_u2_tune_status;
} /* End of function R_AID_GetSystemStatus */

/*******************************************************************************************************************//**
 * @brief  Gets the error status of Tuner
 * @return Error code, @see ERROR_CODE for detailed error code definitions
 **********************************************************************************************************************/
uint16_t R_AID_GetErrorStatus(void)
{
    return aid_u2_error_status;
} /* End of function R_AID_GetErrorStatus */

/*******************************************************************************************************************//**
 * @brief  Gets the progress of parameter identification
 * @return The progress of parameter identification, value returned will be in the range of 0~1 that means 0%~100%
 * @todo Check the progress value
 **********************************************************************************************************************/
float R_AID_GetProgress(void)
{
    float f4_progress = 0.0f;

    if (AID_STATUS_COMPLETED == aid_u2_tune_status)
    {
        f4_progress = 1.0f;
    }
    else
    {
        switch(aid_u2_para_mode)
        {
            case AID_PARAMODE_INIT:
            {
                f4_progress = 0.0f;
            }
            break;

            case AID_PARAMODE_VOLTERR:
            {
                f4_progress = 0.0f;
            }
            break;

            case AID_PARAMODE_R_DIFF:
            {
                f4_progress = 0.5f;
            }
            break;

            case AID_PARAMODE_RLD_RLS:
            {
                f4_progress = 0.60f;
            }
            break;

            case AID_PARAMODE_RLD_DFT:
            {
                f4_progress = 0.625f;
            }
            break;

            case AID_PARAMODE_LQ_RLS:
            {
                f4_progress = 0.65f;
            }
            break;

            case AID_PARAMODE_LQ_DFT:
            {
                f4_progress = 0.675f;
            }
            break;

            case AID_PARAMODE_KE:
            {
                f4_progress = 0.7f;
            }
            break;

            case AID_PARAMODE_JD:
            {
                f4_progress = 0.8f;
            }
            break;

            case AID_PARAMODE_END:
            {
                f4_progress = 1.0f;
            }
            break;

            default:
            {
                /* Do nothing */
                AID_ASSERT_FAIL();
            }
            break;
        }
    }

    return f4_progress;
} /* End of function R_AID_GetProgress */

/*******************************************************************************************************************//**
 * @brief Get identified resistance [ohm]
 * @return Motor resistance [ohm]
 **********************************************************************************************************************/
float R_AID_GetResistance(void)
{
    return aid_f4_r;
} /* End of function R_AID_GetResistance */

/*******************************************************************************************************************//**
 * @brief  Get identified d-axis inductance
 * @return Motor d-axis inductance
 **********************************************************************************************************************/
float R_AID_GetLd(void)
{
    return aid_f4_ld;
} /* End of function R_AID_GetLd */

/*******************************************************************************************************************//**
 * @brief  Get identified q-axis inductance
 * @return Motor q-axis inductance
 **********************************************************************************************************************/
float R_AID_GetLq(void)
{
    return aid_f4_lq;
} /* End of function R_AID_GetLq */

/*******************************************************************************************************************//**
 * @brief  Get identified BEMF constant
 * @return Motor BEMF constant (rated flux)[Wb]
 **********************************************************************************************************************/
float R_AID_GetKe(void)
{
    return aid_f4_ke;
} /* End of function R_AID_GetKe */

/*******************************************************************************************************************//**
 * @brief  Get identified moment of inertia
 * @return Moment of inertia [kgm^2]
 **********************************************************************************************************************/
float R_AID_GetInertia(void)
{
    return aid_f4_j;
} /* End of function R_AID_GetInertia */

/*******************************************************************************************************************//**
 * @brief  Get identified friction coefficient
 * @return Friction coefficient [Nm/(rad/s)]
 **********************************************************************************************************************/
float R_AID_GetFriction(void)
{
    return aid_f4_d;
} /* End of function R_AID_GetFriction */

/*******************************************************************************************************************//**
 * @brief Gets identified lookup table of voltage error identification
 * @pre   The result can be acquired only after successful identification
 * @param st_lut  Pointer to a structure to store measured voltage error LUT,the structure must be declared by user
 **********************************************************************************************************************/
void R_AID_GetVolterrLUT(st_aid_volterr_lut_t *st_lut)
{
    int32_t i;

    for(i = 0; i < AID_VOLTERR_TABLE_SIZE; i++)
    {
        st_lut->volterr_table[i] = aid_f4_volterr_voltage_tab[i];
        st_lut->current_table[i] = aid_f4_volterr_current_tab[i];
    }

    st_lut->ref_voltage = aid_f4_volterr_vdc_ref;
} /* End of function R_AID_GetVolterrLUT */

/*******************************************************************************************************************//**
 * @brief Gets current identification setting
 * @param st_id_setting  Pointer to the structure of identifier setting, see @ref st_aid_id_setting_t
 **********************************************************************************************************************/
void R_AID_GetIDSetting(st_aid_id_setting_t *st_id_setting)
{
    if (NULL != st_id_setting)
    {
        st_id_setting->f4_inertia_range         = gui_f4_tune_inertia_setting;
        st_id_setting->f4_rated_current         = gui_f4_tune_current_setting;
        st_id_setting->u2_volterr_crnt_step_lsb = gui_u2_tune_volterr_setting;
        st_id_setting->u1_volterr_is_enabled    = gui_u1_volterr_is_enabled;
        st_id_setting->u2_num_pole_pairs        = gui_u2_tune_polepairs_setting;
    }
} /* End of function R_AID_GetIDSetting */

/*******************************************************************************************************************//**
 * @brief Current control interrupt handler, must be called by user
 * @pre   This function must be called in an interrupt that synchronized with (or decimated) PWM.
 *        see parameters description in @ref R_AID_Init
 * @note  Wrapper of current control interrupt handler
 **********************************************************************************************************************/
void R_AID_CurrentCtrlISR(void)
{
    if (AID_STATUS_MEASURE == aid_u2_tune_status)
    {
        aid_core_crnt_ctrl_handler();
        aid_f4_total_time_elapsed += (aid_f4_ctrl_period_ms);
    }

    /* Run the state machine without event to trigger the delayed transition (such as RESET=>READY) */
    aid_state_machine(AID_COMMAND_NONE);
} /* End of function R_AID_CurrentCtrlISR */

/*******************************************************************************************************************//**
 * @brief Speed control interrupt handler, must be called by user
 * @pre   This function must be called in an interrupt that occurs by the period set in initialization (R_AID_Init)
 *        see parameters description in @ref R_AID_Init
 * @note  Wrapper of current control interrupt handler
 **********************************************************************************************************************/
void R_AID_SpeedCtrlISR(void)
{
    aid_core_spd_ctrl_handler();
} /* End of function R_AID_SpeedCtrlISR */

/*******************************************************************************************************************//**
 * @brief     State machine
 * @param[in] event  The event code AID_COMMAND_<CMD> group is available, in addition,
 *                   AID_PRV_EVENT_COMPLETED is also available for completed event
 **********************************************************************************************************************/
static void aid_state_machine(uint16_t event)
{
    switch (aid_u2_tune_status)
    {
        /* These case are intentionally combined */
        case AID_STATUS_READY:
        case AID_STATUS_COMPLETED:
        {
            if (AID_COMMAND_START == event)
            {
                aid_reset();
                aid_f4_total_time_elapsed = 0.0f;
                aid_u2_tune_status = AID_STATUS_MEASURE;
            }
            /* The RESUME command is available only when the last identification is not completed */
            else if ((AID_COMMAND_RESUME == event) && (AID_PARAMODE_INIT != aid_u2_para_mode))
            {
                aid_revert_to_lastest_paramode(&aid_u2_para_mode);
                aid_u2_tune_status = AID_STATUS_MEASURE;
            }
            else if (AID_PRV_EVENT_ERROR == event)
            {
                aid_u2_tune_status = AID_STATUS_ERROR;
            }
            else
            {
                /* Do nothing */
                __asm __volatile("nop\n");
            }
        }
        break;

        case AID_STATUS_MEASURE:
        {
            if (AID_COMMAND_STOP == event)
            {
                aid_u2_tune_status = AID_STATUS_RESET;
            }
            else if (AID_PRV_EVENT_COMPLETED == event)
            {
                aid_u2_tune_status = AID_STATUS_COMPLETED;
            }
            else if (AID_PRV_EVENT_ERROR == event)
            {
                aid_u2_tune_status = AID_STATUS_ERROR;
            }
            else
            {
                /* Do nothing */
                __asm __volatile("nop\n");
            }
        }
        break;

        case AID_STATUS_ERROR:
        {
            if (AID_COMMAND_RESET == event)
            {
                aid_u2_tune_status = AID_STATUS_RESET;
            }
            else if (AID_COMMAND_RESUME == event)
            {
                aid_core_clear_error();
                aid_revert_to_lastest_paramode(&aid_u2_para_mode);
                aid_u2_tune_status = AID_STATUS_MEASURE;
            }
            else
            {
                /* Do nothing */
                __asm __volatile("nop\n");
            }
        }
        break;

        case AID_STATUS_RESET:
        {
            aid_core_stop();
            aid_core_clear_error();
            aid_u2_tune_status = AID_STATUS_READY;
        }
        break;

        default:
        {
            AID_ASSERT_FAIL();
        }
        break;
    }
} /* End of function aid_state_machine */

/*******************************************************************************************************************//**
 * @brief Reset tuner for next identification
 * @details All submodule will be reset, derived parameter such as rated speed/torque will be cleared
 * @warning CORE module will also be reset so the identified parameter will be lost.
 **********************************************************************************************************************/
static void aid_reset(void)
{
    /* Sequence */
    aid_u2_para_mode = AID_PARAMODE_INIT;
    aid_core_reset();   /* This will also resets motor parameters */

    /* Input */
    aid_f4_rated_current = 0.0f;
    aid_f4_rated_power = 0.0f;
    aid_f4_pole_pairs = 0.0f;

    aid_volterr_reset();
    aid_rdiff_reset();
    aid_ldq_reset();
    aid_ke_reset();
    aid_jd_reset();

    /* Derived parameters */
    aid_f4_rated_speed_mech_rpm = 0.0f;
    aid_f4_rated_speed_elec_rad = 0.0f;
    aid_f4_rated_torque_nm = 0.0f;

} /* End of function aid_reset */

/*******************************************************************************************************************//**
 * @brief Revert identification to latest state that is able to restart
 * @param[in,out] p_paramode  Pointer to parameter identification mode variable
 **********************************************************************************************************************/
static void aid_revert_to_lastest_paramode(uint16_t *p_paramode)
{
    switch(*p_paramode)
    {
        case AID_PARAMODE_VOLTERR:
        {
            aid_volterr_reset();
        }
        break;

        case AID_PARAMODE_R_DIFF:
        {
            aid_rdiff_reset();
            aid_f4_r = 0.0f;
        }
        break;

        case AID_PARAMODE_RLD_RLS:
        case AID_PARAMODE_RLD_DFT:
        {
            aid_ldq_reset();
            aid_f4_ld = 0.0f;
            *p_paramode = AID_PARAMODE_RLD_RLS;
        }
        break;

        case AID_PARAMODE_LQ_RLS:
        case AID_PARAMODE_LQ_DFT:
        {
            aid_ldq_reset();
            aid_f4_lq = 0.0f;
            *p_paramode = AID_PARAMODE_LQ_RLS;
        }
        break;

        case AID_PARAMODE_KE:
        {
            aid_ke_reset();
            aid_f4_ke = 0.0f;
        }
        break;

        case AID_PARAMODE_JD:
        {
            aid_jd_reset();
            aid_f4_j = 0.0f;
            aid_f4_d = 0.0f;
        }
        break;

        default:
        {
            /* Do nothing */
            __asm __volatile("nop\n");
        }
        break;
    }
} /* End of function aid_revert_to_lastest_paramode */

/*******************************************************************************************************************//**
 * @brief Execute motor identification sequence, this sequence function should be execute before setting the current
 *        reference
 * @note  AID_PARAMODE_JD state is excluded because it is time consuming that will delay the PWM output update the
 *        process is put into aid_motorid_sequence_post function
 **********************************************************************************************************************/
static void aid_motorid_sequence(void)
{
    uint16_t is_completed = false;

    switch (aid_u2_para_mode)
    {
        case AID_PARAMODE_INIT:
        {
            /* Wait until the CORE module is ready to receive voltage/current command */
            if (AID_RUNMODE_READY == aid_u2_run_mode)
            {
                aid_act_tune_init();    /* This function is only executed once */
                if (true == gui_u1_volterr_is_enabled)
                {
                    aid_u2_para_mode = AID_PARAMODE_VOLTERR;
                }
                else
                {
                    aid_u2_para_mode = aid_get_next_paramode(); /* AID_PARAMODE_R_DIFF */
                }
            }
        }
        break;

        case AID_PARAMODE_VOLTERR:
        {
            if (g_u1_aid_volterr_enable == 1)
            {
                is_completed = aid_volterr_act();
                if (true == is_completed)
                {
                    aid_core_set_volterr_table(aid_f4_volterr_current_tab,
                                               aid_f4_volterr_voltage_tab,
                                               aid_f4_volterr_vdc_ref);
                    aid_u2_para_mode = aid_get_next_paramode();/* AID_PARAMODE_R_DIFF */
                }
            }
            else
            {
                AID_ASSERT_FAIL();
            }
        }
        break;

        case AID_PARAMODE_R_DIFF:
        {
            is_completed = aid_rdiff_act();
            if (true == is_completed)
            {
                if(0.0f == aid_f4_r)
                {
                    aid_f4_r = aid_f4_r_diff;
                    aid_f4_r_dc = aid_f4_r_diff;
                }
                aid_f4_v_err = aid_f4_volterr_est;
                aid_u2_para_mode = aid_get_next_paramode(); /* AID_PARAMODE_RLD_RLS */
            }
        }
        break;

        case AID_PARAMODE_RLD_RLS:
        {
            is_completed = aid_rld_rls_act(AID_SIGNAL_CONF_DEFAULT);
            if (true == is_completed)
            {
                aid_f4_ld = aid_f4_ld_rls;
                aid_u2_para_mode = AID_PARAMODE_RLD_DFT; /* Always perform DFT method identification after RLS */
            }
        }
        break;

        case AID_PARAMODE_RLD_DFT:
        {
            is_completed = aid_rld_dft_act(AID_SIGNAL_CONF_DEFAULT);
            if (true == is_completed)
            {
                aid_f4_ld = aid_f4_ld_dft;
                aid_u2_para_mode = aid_get_next_paramode(); /* AID_PARAMODE_LQ_RLS */
            }
        }
        break;

        case AID_PARAMODE_LQ_RLS:
        {
            is_completed = aid_lq_rls_act(AID_SIGNAL_CONF_DEFAULT);
            if (true == is_completed)
            {
                aid_f4_lq = (aid_f4_lq_rls >= aid_f4_ld) ? aid_f4_lq_rls : aid_f4_ld;
                aid_u2_para_mode = AID_PARAMODE_LQ_DFT;  /* Always perform DFT method identification after RLS */
            }
        }
        break;

        case AID_PARAMODE_LQ_DFT:
        {
            is_completed = aid_lq_dft_act(AID_SIGNAL_CONF_DEFAULT);
            if (true == is_completed)
            {
                aid_f4_lq = (aid_f4_lq_dft >= aid_f4_ld) ? aid_f4_lq_dft : aid_f4_ld;
                aid_u2_para_mode = aid_get_next_paramode(); /* AID_PARAMODE_KE */
            }
        }
        break;

        case AID_PARAMODE_KE:
        {
            is_completed = aid_ke_act();
            if (true == is_completed)
            {
                aid_f4_ke = aid_f4_ke_closed_loop;
                aid_u2_para_mode = aid_get_next_paramode(); /* AID_PARAMODE_JD */
            }
        }
        break;

        case AID_PARAMODE_JD:
        {
            /* The action function is invoked after PWM setting, see aid_motorid_seuence_post */
            __asm __volatile("nop\n");
        }
        break;

        case AID_PARAMODE_END:
        {
            aid_act_tune_end();
            aid_state_machine(AID_PRV_EVENT_COMPLETED);
            aid_u2_para_mode = AID_PARAMODE_INIT;
        }
        break;

        default:
        {
            AID_ASSERT_FAIL();
        }
        break;
    }
} /* End of function aid_motorid_sequence */

/*******************************************************************************************************************//**
 * @brief Process executed after update PWM duty cycles
 **********************************************************************************************************************/
static void aid_motorid_sequence_post(void)
{
    uint16_t is_completed = false;

    if (AID_PARAMODE_JD == aid_u2_para_mode)
    {
        is_completed = aid_jd_act();
        if (true == is_completed)
        {
            aid_u2_para_mode = AID_PARAMODE_END;
            aid_f4_j = aid_f4_j_dft;
            aid_f4_d = aid_f4_d_dft;
        }
    }
} /* End of function aid_motorid_sequence_post */

/*******************************************************************************************************************//**
 * Load parameters from user inputs
 * @note This function will be called when transitioning from AID_PARAMODE_INIT to next mode, since the DC voltage
 * knowledge is required
 **********************************************************************************************************************/
static void aid_act_tune_init(void)
{
    /* Input from GUI */
    aid_f4_rated_current     = gui_f4_tune_current_setting;
    aid_f4_pole_pairs        = gui_u2_tune_polepairs_setting;

    /* Check input parameters */
    if ((aid_f4_rated_current > aid_f4_rated_current_max) || (aid_f4_rated_current <= 0.0f))
    {
        aid_core_throw_error(AID_ERROR_INPUT_CURRENT);
        return;
    }

    /* Number of pole pairs should not be less than 1, too many pole pairs (more than 100) is also not make sense */
    if ((aid_f4_pole_pairs <= 0) || (aid_f4_pole_pairs > g_f4_aid_num_pole_pair_limit))
    {
        aid_core_throw_error(AID_ERROR_INPUT_POLEPAIR);
        return;
    }
    if ((gui_f4_tune_inertia_setting < 0.0f) || (gui_f4_tune_inertia_setting > 1.0f))
    {
        aid_core_throw_error(AID_ERROR_INPUT_INERTIA_RANGE);
        return;
    }
    aid_core_set_rated_current(aid_f4_rated_current);

    if (g_u1_aid_volterr_enable == 1)
    {
        if (gui_u2_tune_volterr_setting < 1)
        {
            aid_core_throw_error(AID_ERROR_INPUT_VOLTERR_STEP);
            return;
        }
        aid_volterr_config(g_f4_aid_volterr_init_r, aid_f4_current_lsb, AID_VOLTERR_RID_RLS);
        aid_volterr_config_current_step(gui_u2_tune_volterr_setting);
    }

    /* Assume that maximum copper loss (proportional to R) should not exceed a certain limit */
    aid_f4_r_max = aid_f4_vdc_ad / aid_f4_rated_current;
    aid_f4_r_min = g_f4_aid_r_min;
    aid_f4_ld_min = 0.0f;
    aid_f4_lq_min = 0.0f;

    aid_f4_r = gs_f4_cfg_init_r;
    aid_f4_r_dc = gs_f4_cfg_init_r;
    aid_f4_ld = gs_f4_cfg_init_ld;
    aid_f4_lq = gs_f4_cfg_init_lq;
    aid_f4_ke = gs_f4_cfg_init_ke;

    aid_jd_config_inertia_range(gui_f4_tune_inertia_setting);
} /* End of function aid_act_tune_init */

/******************************************************************************
* Function Name : aid_act_tune_end
* Description   :
* Arguments     : None
* Return Value  : none
******************************************************************************/
static void aid_act_tune_end(void)
{
    /* Rated speed (electrical,[rad/s] */
    aid_f4_rated_speed_elec_rad = aid_design_base_speed(aid_f4_ke_open,
                                                       aid_f4_r,
                                                       aid_f4_lq,
                                                       aid_f4_vmag_max,
                                                       aid_f4_imag_max);

    /* Rated speed [rpm] */
    aid_f4_rated_speed_mech_rpm = (aid_f4_rated_speed_elec_rad / aid_f4_pole_pairs) * (60.0f / AID_TWOPI);

    /* Rated torque */
    aid_f4_rated_torque_nm = aid_f4_ke * aid_f4_rated_current * aid_f4_pole_pairs;
    aid_core_stop();
} /* End of function aid_act_tune_end */

/*******************************************************************************************************************//**
 * @brief  Determine the next parameter identification mode. Skip the identification of known parameter.
 * @note   Implicit input:
 *              - aid_f4_r
 *              - aid_f4_ld
 *              - aid_f4_lq
 *              - aid_f4_ke
 *              - aid_f4_jd
 * @warning This function only handles electrical parameter identification modes, voltage error measurement mode is
 *          excluded if enabled
 * @return The next parameter identification mode. See @ref PARAMODE
 **********************************************************************************************************************/
static uint16_t aid_get_next_paramode(void)
{
    uint16_t ret = AID_PARAMODE_END;

    /* Note: it is intentionally comparing float and zero */
    if ((0.0f == aid_f4_r) || (0.0f == aid_f4_r_dc))
    {
        ret = AID_PARAMODE_R_DIFF;
    }
    else if (0.0f == aid_f4_ld)
    {
        ret = AID_PARAMODE_RLD_DFT;
    }
    else if (0.0f == aid_f4_lq)
    {
        ret = AID_PARAMODE_LQ_DFT;
    }
    else if (0.0f == aid_f4_ke)
    {
        if (0.0f == aid_f4_j)
        {
            aid_ke_config_no_stop_flag(true);
        }
        ret = AID_PARAMODE_KE;
    }
    else if (0.0f == aid_f4_j)
    {
        ret = AID_PARAMODE_JD;
    }
    else
    {
        AID_ASSERT_FAIL();
    }

    return ret;
} /* End of function aid_get_next_paramode */


/*******************************************************************************************************************//**
 * @brief     Calculate the base speed with given motor parameters, voltage and current limitations
 *
 * @param[in] f4_flux_wb  The flux, BEMF coefficient [Wb]
 * @param[in] f4_r        The resistance[ohm]
 * @param[in] f4_lq       The q-axis inductance[h]
 * @param[in] f4_va_max   The maximum magnitude of voltage vector [V]
 * @param[in] f4_ia_max   The maximum magnitude of current vector [A]
 *
 * @return    The base speed [rad/s], which is the maximum speed that can deliver maximum torque
 **********************************************************************************************************************/
static float aid_design_base_speed(float f4_flux_wb, float f4_r, float f4_lq, float f4_va_max, float f4_ia_max)
{
    float f4_totalflux_max;
    float f4_vflux_max;
    float f4_flux_q_wb;
    float f4_base_speed_rad;

    /* NOTE: this is a simplified method that can only be used on non-salient motor */
    /* Flux on q-axis is generated by q-axis current, assumes that all current is used on q-axis */
    f4_flux_q_wb = f4_lq * f4_ia_max;

    /* Calculate maximum total flux */
    f4_totalflux_max = sqrtf((f4_flux_wb * f4_flux_wb) + (f4_flux_q_wb * f4_flux_q_wb));

    /* Calculate maximum voltage can be generated by flux */
    f4_vflux_max = f4_va_max - (f4_r * f4_ia_max);

    /* Return the base speed */
    f4_base_speed_rad = f4_vflux_max / f4_totalflux_max;

    return (f4_base_speed_rad);
} /* End of function aid_design_base_speed */

/** @} */
/* End of file r_aid_auto_identify.c */
