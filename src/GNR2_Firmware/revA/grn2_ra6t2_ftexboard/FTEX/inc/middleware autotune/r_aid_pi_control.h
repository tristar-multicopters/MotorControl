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
* File Name   : r_aid_pi_control.h
* Description : Definitions of general purpose PI control modules
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version
*         : 05.04.2017 1.00
*         : 07.07.2017 1.01
***********************************************************************************************************************/

/* Guard against multiple inclusion */
#ifndef R_AID_PI_CONTROL_H
#define R_AID_PI_CONTROL_H

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
* Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Structure definitions
***********************************************************************************************************************/
typedef struct
{
    float f4_err;         /* Error */
    float f4_kp;          /* Gain of proportional term */
    float f4_ki;          /* Gain of integral term */
    float f4_refi;        /* Buffer of integral term */
    float f4_ilimit;      /* Limit of integral term */
} st_aid_pi_ctrl_t;

/***********************************************************************************************************************
* Global function definitions
***********************************************************************************************************************/
/***********************************************************************************************************************
* Function Name : aid_pi_ctrl
* Description   : PI control
* Arguments     : pi_ctrl - The pointer to the PI control structure
* Return Value  : PI control output value
***********************************************************************************************************************/
float aid_pi_ctrl(st_aid_pi_ctrl_t *pi_ctrl, float f4_err);

/***********************************************************************************************************************
* Function Name : aid_pi_ctrl_reset
* Description   : Resets PI control
* Arguments     : p_pi_ctrl - The pointer to the PI control structure
* Return Value  : None
***********************************************************************************************************************/
void aid_pi_ctrl_reset(st_aid_pi_ctrl_t *p_pi_ctrl);

/***********************************************************************************************************************
* Function Name : aid_pi_ctrl_set_kp
* Description   : Sets proportional gain of PI control
* Arguments     : p_pi_ctrl - The pointer to the PI control structure
*               : f4_kp - The proportional gain to set
* Return Value  : None
***********************************************************************************************************************/
void aid_pi_ctrl_set_kp(st_aid_pi_ctrl_t *p_pi_ctrl, float f4_kp);

/***********************************************************************************************************************
* Function Name : aid_pi_ctrl_set_ki
* Description   : Sets integral gain of PI control
* Arguments     : p_pi_ctrl - The pointer to the PI control structure
*               : f4_ki - The integral gain to set
* Return Value  : None
***********************************************************************************************************************/
void aid_pi_ctrl_set_ki(st_aid_pi_ctrl_t *p_pi_ctrl, float f4_ki);

/***********************************************************************************************************************
* Function Name : aid_pi_ctrl_set_integrator
* Description   : Sets value integrator of PI control directly
* Arguments     : p_pi_ctrl - The pointer to the PI control structure
*               : f4_integrator - The new value for integrator
* Return Value  : None
***********************************************************************************************************************/
void aid_pi_ctrl_set_integrator(st_aid_pi_ctrl_t *p_pi_ctrl, float f4_integrator);

/***********************************************************************************************************************
* Function Name : aid_pi_ctrl_set_integral_limit
* Description   : Sets integral limit of PI control
* Arguments     : p_pi_ctrl - The pointer to the PI control structure
*               : f4_ilimit - The integral limit value to set
* Return Value  : None
***********************************************************************************************************************/
void aid_pi_ctrl_set_integral_limit(st_aid_pi_ctrl_t *p_pi_ctrl, float f4_ilimit);

#endif /* R_MTR_PI_CONTROL_H */
