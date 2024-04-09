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
* File Name    : r_aid_auto_identify.h
* Version      : 0.1
* Description  : API of Tuner for Stepping motor
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : XX.06.2018 1.00     First Release
***********************************************************************************************************************/
#include <stdint.h>
#ifndef R_AID_AUTO_IDENTIFY_H_
#define R_AID_AUTO_IDENTIFY_H_

/*******************************************************************************************************************//**
 * @addtogroup AUTO_IDENTIFY
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define AID_API_MAJOR_VERSION           (0)         /**< Defines the major version of API */
#define AID_API_MINOR_VERSION           (1)         /**< Defines the minor version of API */
#define AID_VOLTERR_TABLE_SIZE          (5)
/**
 * @defgroup SYS_STATUS System Status
 * @brief Indicates the status of Tuner
 *  \dot
 *  digraph example {
 *      nodesep = 0.8
 *      node [ shape=record, fontname=Helvetica, fontsize=10];
 *      rd   [ label="READY" URL="\ref AID_STATUS_READY"];
 *      m    [ label="MEASURE" URL="\ref AID_STATUS_MEASURE"];
 *      err  [ label="ERROR" URL="\ref AID_STATUS_ERROR"];
 *      re   [ label="RESET" URL="\ref AID_STATUS_RESET"];
 *      cp   [ label="COMPLETE" URL="\ref AID_STATUS_COMPLETED"];
 *      st   [ label = "Start", shape = circle];
 *      st -> rd;
 *      rd -> m     [fontsize = 8, label="START or RESUME\ncommand" ];
 *      m -> err    [fontsize = 8, label="Error occurred" ];
 *      m -> cp     [fontsize = 8, label="ID completed" ];
 *      m -> rd     [fontsize = 8, label="STOP command" ];
 *      m -> m      [fontsize = 8, label="Identifying motor..." ];
 *      re -> rd    [fontsize = 8, label="Always" ];
 *      err -> re   [fontsize = 8, label="RESET command" ];
 *      err -> m    [fontsize = 8, label="RESUME command" ];
 *      cp -> m     [fontsize = 8, label="START or RESUME\ncommand" ];
 *  }
 *  \enddot
 * @{*/
#define AID_STATUS_READY                (0)         /**< Defines READY status code */
#define AID_STATUS_MEASURE              (1)         /**< Defines MEASURE status code */
#define AID_STATUS_ERROR                (2)         /**< Defines ERROR status code */
#define AID_STATUS_RESET                (3)         /**< Defines RESET status code */
#define AID_STATUS_COMPLETED            (4)         /**< Defines COMPLETED status code */
/** @}*/

/**
 * @defgroup PARAMODE Parameter Identification Mode
 * @brief Indicates which stage is the identification process in
 *  \dot
 * digraph example {
 *     nodesep = 0.8
 *     graph   [ rankdir = TB ]
 *     node    [ shape=record, fontname=Helvetica, fontsize=10];
 *
 *     st      [ group = A2 label = "Start", shape = "circle"]
 *     i       [ group = A2 label = "Initialize"];
 *     verr    [ group = A1 label = "Voltage error ID\n(Optional)"];
 *     rdiff   [ group = A1 label = "Resistance ID\nDifferential Method"];
 *     ldr     [ group = A1  label = "Ld ID (RLS method)"];
 *     ldd     [ group = A1 label = "Ld ID (DFT method)"];
 *     lqr     [ group = A1 label = "Lq ID (RLS method)"];
 *     lqd     [ group = A1 label = "Lq ID (DFT method)"];
 *     ke      [ group = A1 label = "Ke ID (Open-loop method)"];
 *     jd      [ group = A1 label = "Mechanical Parameters ID"];
 *     end     [ group = A2 label = "ID End"];
 *     st -> i;
 *     i -> verr [ label = "ID Started &\nOffset Calibration ended"];
 *     verr -> rdiff -> ldr -> ldd -> lqr -> lqd -> ke -> jd -> end -> i;
 * }
 * \enddot
 * @addtogroup PARAMODE
 * @{*/
#define AID_PARAMODE_INIT               (0)         /**< Defines initial identification mode code */
#define AID_PARAMODE_R_DIFF             (1)         /**< Defines RDIFF identification mode code */
#define AID_PARAMODE_RLD_RLS            (2)         /**< Defines RLS method Ld identification mode code */
#define AID_PARAMODE_RLD_DFT            (3)         /**< Defines DFT method Ld identification mode code */
#define AID_PARAMODE_LQ_RLS             (4)         /**< Defines RLS method Lq identification mode code */
#define AID_PARAMODE_LQ_DFT             (5)         /**< Defines DFT method Lq identification mode code */
#define AID_PARAMODE_KE                 (6)         /**< Defines rated flux identification mode code */
#define AID_PARAMODE_JD                 (7)         /**< Defines inertia identification mode code */
#define AID_PARAMODE_END                (8)         /**< Defines end identification mode code */
#define AID_PARAMODE_VOLTERR            (9)         /**< Defines voltage error measurement mode code */
/** @}*/

/**
 * @defgroup ERROR_CODE Error Code
 * @brief Error code that displayed in GUI, can be get by R_AID_GetErrorCode
 * @{*/
#define AID_ERROR_INPUT_CURRENT         (0x1001)    /**< (4097) The rated current input value is invalid */
#define AID_ERROR_INPUT_POLEPAIR        (0x1002)    /**< (4098) The number of pole pairs input value is invalid */
#define AID_ERROR_INPUT_VOLTERR_STEP    (0x1003)    /**< (4099) The voltage error current step input value is invalid */
#define AID_ERROR_INPUT_INERTIA_RANGE   (0x1004)    /**< (4100) The inertia range input value is invalid */
/** @}*/

/**
 * @defgroup Command Command Code
 * @brief Command code
 * @{*/
#define AID_COMMAND_NONE                (0U)        /**< Defines status that no command is issued */
#define AID_COMMAND_START               (1U)        /**< Defines command code that starts tuning */
#define AID_COMMAND_STOP                (2U)        /**< Defines command code that stops tuning */
#define AID_COMMAND_RESET               (3U)        /**< Defines reset command code */
#define AID_COMMAND_RESUME              (4U)        /**< Defines resume command code */
/** @}*/

/**
 * @defgroup FaultRet Fault return code
 * @brief Fault codes that returned by some API functions, integer
 */
#define AID_FAULT_PARAM_R               (-1)        /**< Defines fault code that parameter R is invalid */
#define AID_FAULT_PARAM_LD              (-2)        /**< Defines fault code that parameter Ld is invalid  */
#define AID_FAULT_PARAM_LQ              (-3)        /**< Defines fault code that parameter Lq is invalid  */
#define AID_FAULT_PARAM_KE              (-4)        /**< Defines fault code that parameter Ke is invalid  */
/** @}*/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
/*******************************************************************************************************************//**
 * Voltage error lookup table structure, this data structure is used for setting and getting the lookup table
 **********************************************************************************************************************/
typedef struct
{
    float current_table[AID_VOLTERR_TABLE_SIZE];                 /**< Current table */
    float volterr_table[AID_VOLTERR_TABLE_SIZE];                 /**< Voltage error table */
    float ref_voltage;                      /**< Reference voltage, DC voltage that used in the measurement [V] */
} st_aid_volterr_lut_t;

/*******************************************************************************************************************//**
 * Identification setting structure, this data structure is used for obtaining current identification setting
 **********************************************************************************************************************/
typedef struct
{
    float       f4_rated_current;           /**< The rated current [A] */
    uint16_t    u2_num_pole_pairs;          /**< Number of pole pairs */
    uint8_t     u1_volterr_is_enabled ;     /**< Is the voltage error measurement enabled 1=Enabled, 0=Disabled*/
    uint16_t    u2_volterr_crnt_step_lsb;   /**< Current step of voltage error measurement */
    float       f4_inertia_range;           /**< Range of inertia, 0(No load)~1(Heavy inertia) */
    float       f4_assumed_inertia;         /**< Inertia used to design gains of speed controller */
} st_aid_id_setting_t;

/***********************************************************************************************************************
Exported global functions (to be accessed by other files)
***********************************************************************************************************************/
/*******************************************************************************************************************//**
 * @brief     Initialize Tuner with interrupt tick settings
 * @pre       Any initialization (such as initialization of peripherals) required by driver should be executed before
 *            calling this function.
 * @warning   Execute this function before calling any other API function
 * @param[in] pwm_tick_per_irq   The number of PWM cycle per interrupt tick
 * @param[in] speed_ctrl_period  The speed control period[s]
 **********************************************************************************************************************/
void R_AID_Init(uint8_t pwm_tick_per_irq, float speed_ctrl_period);

/*******************************************************************************************************************//**
 * @brief Send start command
 **********************************************************************************************************************/
void R_AID_CmdStart(void);

/*******************************************************************************************************************//**
 * @brief Send stop command
 **********************************************************************************************************************/
void R_AID_CmdStop(void);

/*******************************************************************************************************************//**
 * @brief Send reset command
 **********************************************************************************************************************/
void R_AID_CmdReset(void);

/*******************************************************************************************************************//**
 * @brief Send resume command
 **********************************************************************************************************************/
void R_AID_CmdResume(void);

/*******************************************************************************************************************//**
 * @brief This is a function for GUI
 * @param[in] cmd_code  The command code
 **********************************************************************************************************************/
void R_AID_CmdByCode(uint16_t cmd_code);

/*******************************************************************************************************************//**
 * @brief     Generate user customizable error to stop the identification process with given error code
 * @param[in] u2_error_code  The error code
 **********************************************************************************************************************/
void R_AID_UserError(uint16_t u2_error_code);

/*******************************************************************************************************************//**
 * @brief     Sets motor plate information includes rated current and number of pole pairs
 *
 * @param[in] f4_rated_current      The rated current
 * @param[in] u2_num_of_pole_pair   The number pole pair
 **********************************************************************************************************************/
void R_AID_ConfigMotorPlate(float f4_rated_current, uint16_t u2_num_of_pole_pair);

/*******************************************************************************************************************//**
 * @brief Enable the voltage error identification for next parameter identification
 **********************************************************************************************************************/
void R_AID_ConfigEnableVolterrID(void);

/*******************************************************************************************************************//**
 * @brief Disable the voltage error identification for next parameter identification
 **********************************************************************************************************************/
void R_AID_ConfigDisableVolterrID(void);

/*******************************************************************************************************************//**
 * @brief     Sets the voltage step in voltage error measurement
 * @details   Use this option to sets the current step of voltage error measurement, a small step size will improve
 *            accuracy of the measurement, however may be insufficient to cover the unsaturated region of voltage error
 *            @par The actual step size in [A] will be determined by multiply this option with the current resolution
 *            set in the driver.
 * @param[in] u1_num_of_crnt_step  The number of current step
 **********************************************************************************************************************/
void R_AID_ConfigSetVolterrCrntStep(uint8_t u1_num_of_crnt_step);

/*******************************************************************************************************************//**
 * @brief     Sets the inertia range in a relative value
 * @details   This option is for preventing over-voltage in inertia identification, in the case of 0,
 *            the tuner will assume that the inertia is light and identify it with large signal,
 *            in contract the case of 1, the tuner will assume that the inertia is very heavy and identify it with signal
 *            as small as possible to prevent over-voltage due to the BEMF.
 *            @par The default value 0 will be sufficient for most case.
 * @param[in] f4_inertia_range  The inertia range (0~1, default:0);
 **********************************************************************************************************************/
void R_AID_ConfigSetInertiaRange(float f4_inertia_range);

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
void R_AID_ConfigSetAssumedInertia(float f4_inertia);

/*******************************************************************************************************************//**
 * @brief Sets lookup table for the voltage error compensation
 * @param st_lut  The data structure of lookup table for handling voltage error
 **********************************************************************************************************************/
void R_AID_ConfigSetVolterrLUT(st_aid_volterr_lut_t *st_lut);

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
int32_t R_AID_SetInitElecParams(float f4_r, float f4_ld, float f4_lq, float f4_ke);

/*******************************************************************************************************************//**
 * @brief Gets the version information of firmware
 * @param p_major_version  The pointer to the variable to store major version
 * @param p_minor_version  The pointer to the variable to store minor version
 **********************************************************************************************************************/
void R_AID_GetVersionInfo(uint16_t *p_major_version, uint16_t *p_minor_version);

/*******************************************************************************************************************//**
 * @brief  Gets period of current control
 * @return Current control period [s]
 **********************************************************************************************************************/
float R_AID_GetCurrentCtrlPeriod(void);

/*******************************************************************************************************************//**
 * @brief  Gets period of speed control
 * @return Speed control period [s]
 **********************************************************************************************************************/
float R_AID_GetSpeedCtrlPeriod(void);

/*******************************************************************************************************************//**
 * @brief  Gets period of PWM carrier
 * @return PWM carrier period [s]
 **********************************************************************************************************************/
float R_AID_GetPWMPeriod(void);

/*******************************************************************************************************************//**
 * @brief  Get the status of Tuner system
 * @retval 0 AID_STATUS_READY       Ready for starting parameter identification
 * @retval 1 AID_STATUS_MEASURE     Measuring (parameter identifying);
 * @retval 2 AID_STATUS_ERROR       Error occurred, see the error code by R_AID_GetErrorCode to identify the cause
 * @retval 3 AID_STATUS_RESET       Wait for system resetting
 * @retval 4 AID_STATUS_COMPLETED   The identification is completed
 **********************************************************************************************************************/
uint16_t R_AID_GetSystemStatus(void);


/*******************************************************************************************************************//**
 * @brief  Gets the error status of Tuner
 * @return Error code, @see ERROR_CODE for detailed error code definitions
 **********************************************************************************************************************/
uint16_t R_AID_GetErrorStatus(void);

/*******************************************************************************************************************//**
 * @brief  Gets the progress of parameter identification
 * @return The progress of parameter identification, value returned will be in the range of 0~1 that means 0%~100%
 * @todo Check the progress value
 **********************************************************************************************************************/
float R_AID_GetProgress(void);

/*******************************************************************************************************************//**
 * @brief Get identified resistance [ohm]
 * @return Motor resistance [ohm]
 **********************************************************************************************************************/
float R_AID_GetResistance(void);

/*******************************************************************************************************************//**
 * @brief  Get identified d-axis inductance
 * @return Motor d-axis inductance
 **********************************************************************************************************************/
float R_AID_GetLd(void);

/*******************************************************************************************************************//**
 * @brief  Get identified q-axis inductance
 * @return Motor q-axis inductance
 **********************************************************************************************************************/
float R_AID_GetLq(void);

/*******************************************************************************************************************//**
 * @brief  Get identified BEMF constant
 * @return Motor BEMF constant (rated flux)[Wb]
 **********************************************************************************************************************/
float R_AID_GetKe(void);

/*******************************************************************************************************************//**
 * @brief  Get identified moment of inertia
 * @return Moment of inertia [kgm^2]
 **********************************************************************************************************************/
float R_AID_GetInertia(void);

/*******************************************************************************************************************//**
 * @brief  Get identified friction coefficient
 * @return Friction coefficient [Nm/(rad/s)]
 **********************************************************************************************************************/
float R_AID_GetFriction(void);

/*******************************************************************************************************************//**
 * @brief Gets identified lookup table of voltage error identification
 * @pre   The result can be acquired only after successful identification
 * @param st_lut  Pointer to a structure to store measured voltage error LUT,the structure must be declared by user
 **********************************************************************************************************************/
void R_AID_GetVolterrLUT(st_aid_volterr_lut_t *st_lut);

/*******************************************************************************************************************//**
 * @brief Gets current identification setting
 * @param st_id_setting  Pointer to the structure of identifier setting, see @ref st_aid_id_setting_t
 **********************************************************************************************************************/
void R_AID_GetIDSetting(st_aid_id_setting_t *st_id_setting);

/*******************************************************************************************************************//**
 * @brief Current control interrupt handler, must be called by user
 * @pre   This function must be called in an interrupt that synchronized with (or decimated) PWM.
 *        see parameters description in @ref R_AID_Init
 * @note  Wrapper of current control interrupt handler
 **********************************************************************************************************************/
void R_AID_CurrentCtrlISR(void);

/*******************************************************************************************************************//**
 * @brief Speed control interrupt handler, must be called by user
 * @pre   This function must be called in an interrupt that occurs by the period set in initialization (R_AID_Init);
 *        see parameters description in @ref R_AID_Init
 * @note  Wrapper of current control interrupt handler
 **********************************************************************************************************************/
void R_AID_SpeedCtrlISR(void);

/** @} */
#endif /* R_AID_AUTO_IDENTIFY_H_ */
