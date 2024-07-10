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
* Copyright (C) 2017 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name   : r_aid_transform.c
* Description : Normal and inverse Clarke Park transformations and related rotor angle calculations
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version
*         : 05.04.2017 1.00
*         : 07.07.2017 1.01
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <math.h>
#include "r_aid_math_consts.h"
#include "r_aid_transform.h"

/***********************************************************************************************************************
* Function Name : aid_rotor_angle_init
* Description   : Initializes and resets rotor angle structure
* Arguments     : p_angle - The pointer to the rotor angle structure
* Return Value  : None
***********************************************************************************************************************/
void aid_rotor_angle_init(aid_rotor_angle_t *p_angle)
{
    aid_rotor_angle_reset(p_angle);
} /* End of function aid_rotor_angle_init */

/***********************************************************************************************************************
* Function Name : aid_rotor_angle_reset
* Description   : Resets the angle to 0 degree and updates related trigonometric values
* Arguments     : p_angle - The pointer to the rotor angle structure
* Return Value  : None
***********************************************************************************************************************/
void aid_rotor_angle_reset(aid_rotor_angle_t *p_angle)
{
    p_angle->f4_rotor_angle_rad = 0.0f;
    p_angle->f4_cos = 1.0f;
    p_angle->f4_sin = 0.0f;
} /* End of function aid_rotor_angle_reset */

/***********************************************************************************************************************
* Function Name : aid_rotor_angle_update
* Description   : Updates the angle and related trigonometric values
* Arguments     : p_angle - The pointer to the rotor angle structure
*               : f4_angle_rad - The angle[rad] used to update the structure
* Return Value  : None
***********************************************************************************************************************/
void aid_rotor_angle_update(aid_rotor_angle_t *p_angle, float f4_angle_rad)
{
    p_angle->f4_rotor_angle_rad = f4_angle_rad;
    p_angle->f4_cos = cosf(f4_angle_rad);
    p_angle->f4_sin = sinf(f4_angle_rad);
} /* End of function aid_rotor_angle_update */

/***********************************************************************************************************************
* Function Name : aid_rotor_angle_get_angle
* Description   : Gets the electrical angle of rotor [rad]
* Arguments     : p_angle - The pointer to the rotor angle structure
* Return Value  : The electrical angle of rotor [rad]
***********************************************************************************************************************/
float aid_rotor_angle_get_angle(aid_rotor_angle_t *p_angle)
{
    return (p_angle->f4_rotor_angle_rad);
} /* End of function aid_rotor_angle_get_angle */

/***********************************************************************************************************************
* Function Name : aid_transform_uvw_dq_abs
* Description   : Coordinate transform UVW to dq (absolute transform)
* Arguments     : p_angle   - the pointer to rotor angle structure
*                 f4_uvw    - the pointer to the UVW-phase array in [U,V,W] format
*                 f4_dq     - where to store the [D,Q] formated array on dq coordinates
* Return Value  : None
***********************************************************************************************************************/
void aid_transform_uvw_dq_abs(const aid_rotor_angle_t *p_angle, const float *f4_uvw, float *f4_dq)
{
    float f4_temp0;
    float f4_temp1;
    float f4_temp2;
    float f4_temp3;
    float f4_u = f4_uvw[0];
    float f4_v_sub_w = f4_uvw[1] - f4_uvw[2];

    f4_temp0 = p_angle->f4_cos * (1.0f / AID_SQRT_2);
    f4_temp1 = p_angle->f4_sin * (1.0f / AID_SQRT_2);
    f4_temp2 = AID_SQRT_3 * f4_temp0;
    f4_temp3 = AID_SQRT_3 * f4_temp1;
    f4_dq[0] = (f4_temp2 * f4_u) + (f4_temp1 * f4_v_sub_w);
    f4_dq[1] = ((-f4_temp3) * f4_u) + (f4_temp0 * f4_v_sub_w);
} /* End of function aid_transform_uvw_dq_abs */

/***********************************************************************************************************************
* Function Name : aid_transform_dq_uvw_abs
* Description   : Coordinate transform dq to UVW 3-phase (absolute transform)
* Arguments     : p_angle   - the pointer to rotor angle structure
*                 f4_dq     - the pointer to the dq-axis value array in [D,Q] format
*                 f4_uvw    - where to store the [U,V,W] formated 3-phase quantities array
* Return Value  : None
***********************************************************************************************************************/
void aid_transform_dq_uvw_abs(const aid_rotor_angle_t *p_angle, const float *f4_dq, float *f4_uvw)
{
    float f4_cos;
    float f4_sin;
    float f4_cos_div_sqrt3;
    float f4_sin_div_sqrt3;
    float f4_output_d;
    float f4_output_q;
    float f4_input_d = f4_dq[0];
    float f4_input_q = f4_dq[1];

    f4_cos = p_angle->f4_cos;
    f4_sin = p_angle->f4_sin;
    f4_cos_div_sqrt3 = f4_cos * (1.0f / AID_SQRT_3);
    f4_sin_div_sqrt3 = f4_sin * (1.0f / AID_SQRT_3);

    f4_uvw[0] = AID_SQRT_2 * ((f4_cos_div_sqrt3 * f4_input_d) - (f4_sin_div_sqrt3 * f4_input_q));

    f4_output_d = ((-f4_cos_div_sqrt3) + f4_sin) * f4_input_d;
    f4_output_q = (f4_sin_div_sqrt3 + f4_cos) * f4_input_q;
    f4_uvw[1] = (f4_output_d + f4_output_q) * (1.0f / AID_SQRT_2);

    f4_output_d = ((-f4_cos_div_sqrt3) - f4_sin) * f4_input_d;
    f4_output_q = (f4_sin_div_sqrt3 - f4_cos) * f4_input_q;
    f4_uvw[2] = (f4_output_d + f4_output_q) * (1.0f / AID_SQRT_2);
} /* End of function aid_transform_dq_uvw_abs */
