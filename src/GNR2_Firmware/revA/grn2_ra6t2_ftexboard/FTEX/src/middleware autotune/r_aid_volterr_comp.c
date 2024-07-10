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
* File Name   : r_aid_volterr_comp.c
* Description : Compensation of inverter output voltage error
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
#include "r_aid_volterr_comp.h"

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_init
* Description   : Initializes voltage error compensation module
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_init(st_aid_volterr_comp_t *st_volt_comp,
                           const float *f4_current_table,
                           const float *f4_volterr_table,
                           float f4_ref_vdc)
{
    /* Enabled by default */
    st_volt_comp->u1_volt_err_comp_enable = 1;
    aid_volterr_comp_set_table(st_volt_comp,
                               f4_current_table,
                               f4_volterr_table,
                               f4_ref_vdc);
} /* End of function aid_volterr_comp_init */

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_reset
* Description   : Resets compensation voltage error value
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_reset(st_aid_volterr_comp_t *st_volt_comp)
{
    uint8_t i;

    for(i = 0; i < 3; i++)
    {
        st_volt_comp->f4_volt_comp_array[i] = 0.0f;
    }
} /* End of function aid_volterr_comp_reset */

/***********************************************************************************************************************
* Function Name : volterr_comp_calc
* Description   : Calculates compensation voltage form current
* Arguments     : p_f4_i_array - The pointer to the motor three phase current
*                 st_volt_comp - The pointer to voltage error compensation structure
* Return Value  : none
***********************************************************************************************************************/
static inline void volterr_comp_calc(float *p_f4_i_array, st_aid_volterr_comp_t *st_volt_comp)
{
    float f4_i_sign[3];
    float f4_i_vec[3];
    float f4_temp[3];
    uint8_t index;

    /* Calculate absolute value and direction of the u phase current */
    if (p_f4_i_array[0] >= 0)
    {
        f4_i_sign[0] = 1.0f;
        f4_i_vec[0] = p_f4_i_array[0];
    }
    else
    {
        f4_i_sign[0] = -1.0f;
        f4_i_vec[0] = -p_f4_i_array[0];
    }

    /* Calculate absolute value and direction of the v phase current */
    if (p_f4_i_array[1] >= 0)
    {
        f4_i_sign[1] = 1.0f;
        f4_i_vec[1] = p_f4_i_array[1];
    }
    else
    {
        f4_i_sign[1] = -1.0f;
        f4_i_vec[1] = -p_f4_i_array[1];
    }

    /* Calculate absolute value and direction of the w phase current */
    if (p_f4_i_array[2] >= 0)
    {
        f4_i_sign[2] = 1.0f;
        f4_i_vec[2] = p_f4_i_array[2];
    }
    else
    {
        f4_i_sign[2] = -1.0f;
        f4_i_vec[2] = -p_f4_i_array[2];
    }

    /*============================================*/
    /*  U phase compensation voltage calculation  */
    /*============================================*/
    if (f4_i_vec[0] < st_volt_comp->f4_comp_i[2])
    {
        if (f4_i_vec[0] >= st_volt_comp->f4_comp_i[1])
        {
            index = 2;
        }
        else if (f4_i_vec[0] >= st_volt_comp->f4_comp_i[0])
        {
            index = 1;
        }
        else
        {
            index = 0;
        }
    }
    else
    {
        if (f4_i_vec[0] <= st_volt_comp->f4_comp_i[3])
        {
            index = 3;
        }
        else if (f4_i_vec[0] <= st_volt_comp->f4_comp_i[4])
        {
            index = 4;
        }
        else
        {
            index = 5;
        }
    }
    f4_temp[0] = ((f4_i_vec[0] * st_volt_comp->f4_slope[index]) + st_volt_comp->f4_intcept[index]);

    /* Scale the table by DC bus voltage  */
    f4_temp[0] = (st_volt_comp->f4_vdc * f4_temp[0]);

    if (f4_temp[0] > (st_volt_comp->f4_volt_comp_limit))
    {
        f4_temp[0] = st_volt_comp->f4_volt_comp_limit;
    }
    st_volt_comp->f4_volt_comp_array[0] = f4_i_sign[0] * f4_temp[0];

    /*============================================*/
    /*  V phase compensation voltage calculation  */
    /*============================================*/
    if (f4_i_vec[1] < st_volt_comp->f4_comp_i[2])
    {
        if (f4_i_vec[1] >= st_volt_comp->f4_comp_i[1])
        {
            index = 2;
        }
        else if (f4_i_vec[1] >= st_volt_comp->f4_comp_i[0])
        {
            index = 1;
        }
        else
        {
            index = 0;
        }
    }
    else
    {
        if (f4_i_vec[1] <= st_volt_comp->f4_comp_i[3])
        {
            index = 3;
        }
        else if (f4_i_vec[1] <= st_volt_comp->f4_comp_i[4])
        {
            index = 4;
        }
        else
        {
            index = 5;
        }
    }
    f4_temp[1] = ((f4_i_vec[1] * st_volt_comp->f4_slope[index]) + st_volt_comp->f4_intcept[index]);

    /* Scale the table by DC bus voltage  */
    f4_temp[1] = (st_volt_comp->f4_vdc * f4_temp[1]);

    if (f4_temp[1] > (st_volt_comp->f4_volt_comp_limit))
    {
        f4_temp[1] = st_volt_comp->f4_volt_comp_limit;
    }
    st_volt_comp->f4_volt_comp_array[1] = f4_i_sign[1] * f4_temp[1];

    /*============================================*/
    /*  W phase compensation voltage calculation  */
    /*============================================*/
    if (f4_i_vec[2] < st_volt_comp->f4_comp_i[2])
    {
        if (f4_i_vec[2] >= st_volt_comp->f4_comp_i[1])
        {
            index = 2;
        }
        else if (f4_i_vec[2] >= st_volt_comp->f4_comp_i[0])
        {
            index = 1;
        }
        else
        {
            index = 0;
        }
    }
    else
    {
        if (f4_i_vec[2] <= st_volt_comp->f4_comp_i[3])
        {
            index = 3;
        }
        else if (f4_i_vec[2] <= st_volt_comp->f4_comp_i[4])
        {
            index = 4;
        }
        else
        {
            index = 5;
        }
    }
    f4_temp[2] = ((f4_i_vec[2] * st_volt_comp->f4_slope[index]) + st_volt_comp->f4_intcept[index]);

    /*  Scale the table by DC bus voltage  */
    f4_temp[2] = (st_volt_comp->f4_vdc * f4_temp[2]);

    if (f4_temp[2] > (st_volt_comp->f4_volt_comp_limit))
    {
        f4_temp[2] = st_volt_comp->f4_volt_comp_limit;
    }
    st_volt_comp->f4_volt_comp_array[2] = f4_i_sign[2] * f4_temp[2];
} /* End of function mtr_volt_comp_calc */

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_main
* Description   : Compensates voltage error
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
*                 p_f4_v_array - The pointer to the compensation amount of the motor three phase voltage
*                 p_f4_i_array - The pointer to the motor three phase current
*                 f4_vdc       - Inverter bus voltage
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_main(st_aid_volterr_comp_t *st_volt_comp, float *p_f4_v_array, float *p_f4_i_array, float f4_vdc)
{

    if (1 == st_volt_comp->u1_volt_err_comp_enable)
    {
        st_volt_comp->f4_vdc = f4_vdc;
        /*===============================================*/
        /* Calculate compensation voltage limit          */
        /*===============================================*/
        st_volt_comp->f4_volt_comp_limit = st_volt_comp->f4_comp_v[AID_VOLTERR_COMP_LUT_SIZE - 1] * st_volt_comp->f4_vdc;

        /*===============================================*/
        /*      Calculate compensation voltage           */
        /*===============================================*/
        volterr_comp_calc(p_f4_i_array, st_volt_comp);

        p_f4_v_array[0] += st_volt_comp->f4_volt_comp_array[0];
        p_f4_v_array[1] += st_volt_comp->f4_volt_comp_array[1];
        p_f4_v_array[2] += st_volt_comp->f4_volt_comp_array[2];
    }
    else
    {
        st_volt_comp->f4_volt_comp_array[0] = 0.0f;
        st_volt_comp->f4_volt_comp_array[1] = 0.0f;
        st_volt_comp->f4_volt_comp_array[2] = 0.0f;
    }

} /* End of function aid_volterr_comp_main */

/***********************************************************************************************************************
* Function Name : volterr_comp_calc_ab
* Description   : Calculates compensation voltage form current
* Arguments     : p_f4_i_array - The pointer to the motor three phase current
*                 st_volt_comp - The pointer to voltage error compensation structure
* Return Value  : none
***********************************************************************************************************************/
static inline void volterr_comp_calc_ab(float *p_f4_i_array, st_aid_volterr_comp_t *st_volt_comp)
{
    float f4_i_sign[2];
    float f4_i_vec[2];
    float f4_temp[2];
    uint8_t index;

    /* Calculate absolute value and direction of the a phase current */
    if (p_f4_i_array[0] >= 0)
    {
        f4_i_sign[0] = 1.0f;
        f4_i_vec[0] = p_f4_i_array[0];
    }
    else
    {
        f4_i_sign[0] = -1.0f;
        f4_i_vec[0] = -p_f4_i_array[0];
    }

    /* Calculate absolute value and direction of the b phase current */
    if (p_f4_i_array[1] >= 0)
    {
        f4_i_sign[1] = 1.0f;
        f4_i_vec[1] = p_f4_i_array[1];
    }
    else
    {
        f4_i_sign[1] = -1.0f;
        f4_i_vec[1] = -p_f4_i_array[1];
    }

    /*============================================*/
    /*  a phase compensation voltage calculation  */
    /*============================================*/
    if (f4_i_vec[0] < st_volt_comp->f4_comp_i[2])
    {
        if (f4_i_vec[0] >= st_volt_comp->f4_comp_i[1])
        {
            index = 2;
        }
        else if (f4_i_vec[0] >= st_volt_comp->f4_comp_i[0])
        {
            index = 1;
        }
        else
        {
            index = 0;
        }
    }
    else
    {
        if (f4_i_vec[0] <= st_volt_comp->f4_comp_i[3])
        {
            index = 3;
        }
        else if (f4_i_vec[0] <= st_volt_comp->f4_comp_i[4])
        {
            index = 4;
        }
        else
        {
            index = 5;
        }
    }
    f4_temp[0] = ((f4_i_vec[0] * st_volt_comp->f4_slope[index]) + st_volt_comp->f4_intcept[index]);

    /* Scale the table by DC bus voltage  */
    f4_temp[0] = (st_volt_comp->f4_vdc * f4_temp[0]);

    if (f4_temp[0] > (st_volt_comp->f4_volt_comp_limit))
    {
        f4_temp[0] = st_volt_comp->f4_volt_comp_limit;
    }
    st_volt_comp->f4_volt_comp_array[0] = f4_i_sign[0] * f4_temp[0];

    /*============================================*/
    /*  b phase compensation voltage calculation  */
    /*============================================*/
    if (f4_i_vec[1] < st_volt_comp->f4_comp_i[2])
    {
        if (f4_i_vec[1] >= st_volt_comp->f4_comp_i[1])
        {
            index = 2;
        }
        else if (f4_i_vec[1] >= st_volt_comp->f4_comp_i[0])
        {
            index = 1;
        }
        else
        {
            index = 0;
        }
    }
    else
    {
        if (f4_i_vec[1] <= st_volt_comp->f4_comp_i[3])
        {
            index = 3;
        }
        else if (f4_i_vec[1] <= st_volt_comp->f4_comp_i[4])
        {
            index = 4;
        }
        else
        {
            index = 5;
        }
    }
    f4_temp[1] = ((f4_i_vec[1] * st_volt_comp->f4_slope[index]) + st_volt_comp->f4_intcept[index]);

    /* Scale the table by DC bus voltage  */
    f4_temp[1] = (st_volt_comp->f4_vdc * f4_temp[1]);

    if (f4_temp[1] > (st_volt_comp->f4_volt_comp_limit))
    {
        f4_temp[1] = st_volt_comp->f4_volt_comp_limit;
    }
    st_volt_comp->f4_volt_comp_array[1] = f4_i_sign[1] * f4_temp[1];
} /* End of function mtr_volt_comp_calc */

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_main_ab
* Description   : Compensates voltage error
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
*                 p_f4_v_array - The pointer to the compensation amount of the motor three phase voltage
*                 p_f4_i_array - The pointer to the motor three phase current
*                 f4_vdc       - Inverter bus voltage
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_main_ab(st_aid_volterr_comp_t *st_volt_comp, float *p_f4_v_array, float *p_f4_i_array, float f4_vdc)
{

    if (1 == st_volt_comp->u1_volt_err_comp_enable)
    {
        st_volt_comp->f4_vdc = f4_vdc;
        /*===============================================*/
        /* Calculate compensation voltage limit          */
        /*===============================================*/
        st_volt_comp->f4_volt_comp_limit = st_volt_comp->f4_comp_v[AID_VOLTERR_COMP_LUT_SIZE - 1] * st_volt_comp->f4_vdc;

        /*===============================================*/
        /*      Calculate compensation voltage           */
        /*===============================================*/
        volterr_comp_calc_ab(p_f4_i_array, st_volt_comp);

        p_f4_v_array[0] += st_volt_comp->f4_volt_comp_array[0];
        p_f4_v_array[1] += st_volt_comp->f4_volt_comp_array[1];
    }
    else
    {
        st_volt_comp->f4_volt_comp_array[0] = 0.0f;
        st_volt_comp->f4_volt_comp_array[1] = 0.0f;
    }

} /* End of function aid_volterr_comp_main_ab */

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_set_table
* Description   : Set up voltage error compensation module with given voltage error table
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_set_table(st_aid_volterr_comp_t *st_volt_comp,
                                 const float *f4_current_table,
                                 const float *f4_volterr_table,
                                 float f4_ref_vdc)
{
    uint8_t i;
    float f4_temp0;
    float f4_temp1;

    for(i = 0; i < AID_VOLTERR_COMP_LUT_SIZE; i++)
    {
        st_volt_comp->f4_comp_v[i] = f4_volterr_table[i] / f4_ref_vdc;
        st_volt_comp->f4_comp_i[i] = f4_current_table[i];
    }

    for(i = 0; i < 3; i++)
    {
        st_volt_comp->f4_volt_comp_array[i] = 0.0f;
    }

    /* linear interpolation table */
    for(i = 0; i < (AID_VOLTERR_COMP_LUT_SIZE + 1); i++)
    {
        if (0 == i)
        {
            f4_temp0 = st_volt_comp->f4_comp_v[i];
            f4_temp1 = st_volt_comp->f4_comp_i[i];
            st_volt_comp->f4_slope[i] = f4_temp0 / f4_temp1;
            st_volt_comp->f4_intcept[i] =  0.0f;
        }
        else if (AID_VOLTERR_COMP_LUT_SIZE == i)
        {
            st_volt_comp->f4_slope[i] = 0.0f;
            st_volt_comp->f4_intcept[i] = st_volt_comp->f4_comp_v[AID_VOLTERR_COMP_LUT_SIZE-1];
        }
        else
        {
            f4_temp0 = st_volt_comp->f4_comp_v[i] - st_volt_comp->f4_comp_v[i-1];
            f4_temp1 = st_volt_comp->f4_comp_i[i] - st_volt_comp->f4_comp_i[i-1];
            st_volt_comp->f4_slope[i] = f4_temp0 / f4_temp1;
            st_volt_comp->f4_intcept[i]
                     = st_volt_comp->f4_comp_v[i] - (st_volt_comp->f4_slope[i] * st_volt_comp->f4_comp_i[i]);
        }
    }
}
