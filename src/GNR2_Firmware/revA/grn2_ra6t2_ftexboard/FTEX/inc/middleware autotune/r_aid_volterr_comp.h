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
* File Name   : r_aid_volterr_comp.h
* Description : Definitions of the inverter voltage error compensation
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version
*         : 05.04.2017 1.00
*         : 07.07.2017 1.01
***********************************************************************************************************************/

/* guard against multiple inclusion */
#ifndef AID_VOLTERR_COMP_H
#define AID_VOLTERR_COMP_H

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>

/***********************************************************************************************************************
* Macro definitions
***********************************************************************************************************************/
#define AID_VOLTERR_COMP_LUT_SIZE                (5)

/***********************************************************************************************************************
* Structure definitions
***********************************************************************************************************************/
typedef struct
{
    float f4_comp_v [AID_VOLTERR_COMP_LUT_SIZE];
    float f4_comp_i [AID_VOLTERR_COMP_LUT_SIZE];
    float f4_slope [AID_VOLTERR_COMP_LUT_SIZE +1];
    float f4_intcept[AID_VOLTERR_COMP_LUT_SIZE +1];
    float f4_volt_comp_array[3];
    float f4_vdc;
    float f4_volt_comp_limit;
    uint8_t   u1_volt_err_comp_enable;
} st_aid_volterr_comp_t;

/***********************************************************************************************************************
* Global function definitions
***********************************************************************************************************************/
/***********************************************************************************************************************
* Function Name : aid_volterr_comp_init
* Description   : Initializes voltage error compensation module
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_init(st_aid_volterr_comp_t *st_volt_comp,
                           const float *f4_current_table,
                           const float *f4_volterr_table,
                           float f4_ref_vdc);

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_reset
* Description   : Resets compensation voltage error value
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_reset(st_aid_volterr_comp_t *st_volt_comp);

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_main
* Description   : Compensates voltage error
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
*                 p_f4_v_array - The pointer to the compensation amount of the motor three phase voltage
*                 p_f4_i_array - The pointer to the motor three phase current
*                 f4_vdc       - Inverter bus voltage
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_main(st_aid_volterr_comp_t *st_volt_comp, float *p_f4_v_array,float *p_f4_i_array, float f4_vdc);

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_main_ab
* Description   : Compensates voltage error
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
*                 p_f4_v_array - The pointer to the compensation amount of the motor three phase voltage
*                 p_f4_i_array - The pointer to the motor three phase current
*                 f4_vdc       - Inverter bus voltage
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_main_ab(st_aid_volterr_comp_t *st_volt_comp, float *p_f4_v_array, float *p_f4_i_array, float f4_vdc);

/***********************************************************************************************************************
* Function Name : aid_volterr_comp_set_table
* Description   : Set up voltage error compensation module with given voltage error table
* Arguments     : st_volt_comp - The pointer to voltage error compensation structure
* Return Value  : none
***********************************************************************************************************************/
void aid_volterr_comp_set_table(st_aid_volterr_comp_t *st_volt_comp,
                                const float *f4_current_table,
                                const float *f4_volterr_table,
                                float f4_ref_vdc);
#endif /* MTR_VOLT_ERR_COMP_H */
