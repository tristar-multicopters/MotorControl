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
* File Name    : r_aid_auto_identify_private.h
* Version      : 1.0
* Description  : Core definitions for identification modules
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 15.01.2007 1.00     First Release
***********************************************************************************************************************/
#include <stdint.h>
#include "r_aid_config.h"

#ifndef R_AID_AUTO_IDENTIFY_PRIVATE_H_
#define R_AID_AUTO_IDENTIFY_PRIVATE_H_

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#ifndef NULL
    #define NULL    (0)
#endif

#ifndef RELEASE
#define AID_ASSERT(expr)    if (!(expr)){aid_core_assert_fail(__LINE__, AID_PRV_FILE_CODE);}
#define AID_ASSERT_FAIL()   aid_core_assert_fail(__LINE__, AID_PRV_FILE_CODE)
#else
#define AID_ASSERT(expr)
#define AID_ASSERT_FAIL()   nop()
#endif

/* Motor ID */
#define AID_ID_A                        (0)                       /* Motor ID 0 */

#define AID_SIGNAL_CONF_DEFAULT (0)
/* value for calculation */
#define AID_TWOPI                       (2 * 3.14159265f)         /* 2 * pi */
#define AID_SQRT_2                      (1.41421356f)             /* sqrt(2) */
#define AID_SQRT_3                      (1.7320508f)              /* sqrt(3) */

#define AID_ID_UP                       (0)
#define AID_ID_CONST                    (1)
#define AID_ID_DOWN                     (2)
#define AID_ID_ZERO_CONST               (3)

#define AID_IQ_ZERO_CONST               (0)
#define AID_IQ_SPEED_PI_OUTPUT          (1)
#define AID_IQ_DOWN                     (2)
#define AID_IQ_CL_TRANS                 (3)

#define AID_SPEED_ZERO_CONST            (0)
#define AID_SPEED_CHANGE                (1)

/* control configuration */
#define AID_CONTROL_CURRENT             (0x01)
#define AID_CONTROL_SPEED               (0x02)

#define AID_CTRL_LEVEL_0                (0) /**< Initial state, identification has not started yet */
#define AID_CTRL_LEVEL_1                (1) /**< Only voltage output and current measurement */
#define AID_CTRL_LEVEL_2                (2) /**< Current control loop and BEMF observer is enabled above this level */
#define AID_CTRL_LEVEL_3                (3) /**< Angle and speed is estimated by PLL if possible */
#define AID_CTRL_LEVEL_4                (4) /**< Enables speed controller and startup sequence controller */

/* flag state*/
#define AID_FLG_CLR                     (0)                       /* for flag clear */
#define AID_FLG_SET                     (1)                       /* for flag set */

#define AID_RUNMODE_INIT                (0)
#define AID_RUNMODE_BOOT                (1)
#define AID_RUNMODE_READY               (2)

/** @addtogroup ERROR_CODE
 *  @{*/
#define AID_ERROR_NONE                  (0x0000)            /**< Defines non-error code */
#define AID_ERROR_OVER_CURRENT_HW       (0x0001)            /**< Defines error code, value 1 */
#define AID_ERROR_OVER_CURRENT_SW       (0x0002)            /**< Defines error code, value 2 */
#define AID_ERROR_OVER_VOLTAGE          (0x0003)            /**< Defines error code, value 3 */
#define AID_ERROR_UNDER_VOLTAGE         (0x0004)            /**< Defines error code, value 4 */
#define AID_ERROR_INPUT                 (0x0011)            /**< Defines error code, value 17 */
#define AID_ERROR_R_DIFF                (0x0012)            /**< Defines error code, value 18 */
#define AID_ERROR_R_DFT                 (0x0013)            /**< Defines error code, value 19 */
#define AID_ERROR_R_RLS                 (0x0014)            /**< Defines error code, value 20 */
#define AID_ERROR_LD_DFT                (0x0015)            /**< Defines error code, value 21 */
#define AID_ERROR_LD_RLS                (0x0016)            /**< Defines error code, value 22 */
#define AID_ERROR_LQ_DFT                (0x0017)            /**< Defines error code, value 23 */
#define AID_ERROR_LQ_RLS                (0x0018)            /**< Defines error code, value 24 */
#define AID_ERROR_KE                    (0x0019)            /**< Defines error code, value 25 */
#define AID_ERROR_J                     (0x001A)            /**< Defines error code, value 26 */
#define AID_ERROR_D                     (0x001B)            /**< Defines error code, value 27 */
#define AID_ERROR_J_STARTUP             (0x001C)            /**< Defines error code, value 28 */
#define AID_ERROR_ASSERT_FAIL           (0xFFFF)            /**< Defines error code, value 65535 */
/** @} */

#define AID_WARN_NONE                   (0x00000000U)
#define AID_WARN_VOLT_LIMIT             (0x00000001U)
#define AID_WARN_CURNT_LIMIT            (0x00000002U)
/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
typedef enum
{
    AID_ROTOR_ANGLE_MODE_OPENLOOP   = 0,
    AID_ROTOR_ANGLE_MODE_CLOSEDLOOP = 1,
} e_aid_rotor_angle_mode_t;

typedef struct
{
    float f4_signal_amp;
    float f4_signal_offset;
    float f4_signal_freg_hz;
} st_aid_signal_conf_t;

typedef void (*core_delegate_t)(void);
/***********************************************************************************************************************
Exported global variables
***********************************************************************************************************************/
/* Identification parameters */
extern float aid_f4_rated_current;
extern float aid_f4_rated_power;
extern float aid_f4_pole_pairs;

/* Shared identified parameters */
extern float  aid_f4_v_err;
extern float  aid_f4_r_dc;
extern float  aid_f4_r;
extern float  aid_f4_ld;
extern float  aid_f4_lq;
extern float  aid_f4_ke;
extern float  aid_f4_j;
extern float  aid_f4_d;

/* Basic motor drive signals */
extern float aid_f4_vdc_ad;
extern float aid_f4_angle_rad;
extern float aid_f4_speed_rad;
extern float aid_f4_speed_lpf_rad;
extern float aid_f4_ia_ref;
extern float aid_f4_ib_ref;
extern float aid_f4_ia_ad;
extern float aid_f4_ib_ad;
extern float aid_f4_id_ref;
extern float aid_f4_iq_ref;
extern float aid_f4_id_ad;
extern float aid_f4_iq_ad;
extern float aid_f4_vd_ref;
extern float aid_f4_vq_ref;
extern float aid_f4_va_ref_pwm;
extern float aid_f4_vb_ref_pwm;
extern float aid_f4_ref_speed_rad;
extern float aid_f4_ref_speed_rad_ctrl;
extern float aid_f4_limit_speed_change;
extern float aid_f4_e;
extern float aid_f4_ed;
extern float aid_f4_eq;
extern float aid_f4_e_lpf;
extern float aid_f4_vmag_max;
extern float aid_f4_imag_max;

/* Motor drive control mode selection */
extern uint16_t                 aid_u2_run_mode;            /* Tuner error code, RO */
extern uint16_t                 aid_u2_error_status;        /* Tuner error code, RO */
extern uint16_t                 aid_u2_ctrl_level;          /* Determine control process is enabled or disabled, RW */
extern e_aid_rotor_angle_mode_t aid_s4_rotor_angle_mode;    /* Determine rotor angle by PLL or specified value, RO */

/* Shared counter, to save RAM */
extern uint32_t  aid_u4_sample_cnt;
extern uint16_t  aid_u2_1ms_cnt;
extern uint16_t  aid_u2_sum_cnt;

/* Shared variables */
extern float aid_f4_r_max;      /* The upper limit of a acceptable resistance identification result [ohm] */
extern float aid_f4_r_min;      /* The lower limit of a acceptable resistance identification result [ohm] */
extern float aid_f4_ld_min;     /* The lower limit of a acceptable Ld identification result [H] */
extern float aid_f4_lq_min;     /* The lower limit of a acceptable Lq identification result [H] */

/* Timing variables */
extern float aid_f4_pwm_period_ms;
extern float aid_f4_ctrl_period_ms;
extern float aid_f4_spd_ctrl_period_ms;
extern float aid_f4_ctrl_freq_hz;
extern float aid_f4_spd_ctrl_freq_hz;

extern float aid_f4_overcurrent_limit_hw;
extern float aid_f4_current_lsb;
extern float aid_f4_pwm_duty_lsb;

/* PI gains */
extern float aid_f4_kp_id;
extern float aid_f4_ki_id;
extern float aid_f4_kp_iq;
extern float aid_f4_ki_iq;

/***********************************************************************************************************************
Exported global functions (to be accessed by other files)
***********************************************************************************************************************/
void aid_core_assert_fail(uint32_t line, uint32_t file);
void aid_core_throw_error(uint16_t u2_error_code);
void aid_core_config_bemf_obsv(float f4_omega, float f4_zeta);
void aid_core_config_current_pi_gains(float f4_omega, float f4_zeta);
void aid_core_set_startup_params(float f4_ref_id, float f4_ol2cl_speed_rad, float f4_cl2ol_speed_rad);
void aid_core_set_speed_pi(float f4_kp, float f4_kidt);
void aid_core_set_rated_current(float f4_rated_curret);
void aid_core_set_volterr_table(const float *f4_current_tab, const float *f4_voltage_tab, float f4_vdc_ref);
void aid_core_set_ctrl_level(uint16_t ctrl_level);
void aid_core_clear_error(void);
void aid_core_init(uint8_t pwm_tick_per_irq,
                   float speed_ctrl_period,
                   core_delegate_t before_current_ctrl,
                   core_delegate_t after_pwmoutput);
void aid_core_reset(void);
void aid_core_stop(void);
void aid_core_crnt_ctrl_handler(void);
void aid_core_spd_ctrl_handler(void);
void aid_core_on_error(uint16_t u2_error_code);
void aid_core_reset_startup_seq(void);
#endif /* R_AID_AUTO_IDENTIFY_PRIVATE_H_ */
