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
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : r_aid_volterr.h
 * Version      : 1.0
 * Description  : Definitions for voltage error identification module
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdint.h>
#include "r_aid_config.h"
#ifndef MIDDLE_IDENTIFICATION_R_AID_VOLTERR_H_
    #define MIDDLE_IDENTIFICATION_R_AID_VOLTERR_H_
/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#define AID_VOLTERR_OUTPUT_TAB_SIZE         (5)     /* Table size of the output IV table */

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
typedef enum {
    AID_VOLTERR_SEQ_INIT,           /* The INITIALIZATION stage, to determine parameters for measurement */
    AID_VOLTERR_SEQ_ID_R,           /* The READY stage equivalent, to retrieve resistance by RLS method */
    AID_VOLTERR_SEQ_ID_V_OVER_I,    /* The MEASURE stage equivalent, to acquire the IV characteristics */
    AID_VOLTERR_SEQ_CALC,           /* The CALCULATION stage, process bulk data */
    AID_VOLTERR_SEQ_CHECK,          /* The CHECK stage, performs sanity check on the results */
    AID_VOLTERR_SEQ_RESET,          /* The RESET ramp down the system */
    AID_VOLTERR_SEQ_COMPLETED       /* The completed stage, sequence function returns 1,
                                       sequence will remain on this stage until being reset */
} e_aid_volterr_status_t;

typedef struct
{
    e_aid_volterr_status_t e_status;/* The main status for voltage error measurement sequence */
    float f4_current_table[AID_VOLTERR_POINT_NUM];  /* Current column identified voltage error lookup table */
    float f4_voltage_table[AID_VOLTERR_POINT_NUM];  /* Voltage column identified voltage error lookup table */
    float f4_slope_table[AID_VOLTERR_POINT_NUM];    /* The slope of linear regression of the nearest 5 samples */
    float f4_intercept_table[AID_VOLTERR_POINT_NUM];/* The intercept of linear regression of the nearest 5 samples */
    float f4_sat_current;                           /* Identified current that the voltage error dose not increase,
                                                     the current table will refer to this value */
    float f4_sat_voltage;           /* Identified maximum voltage error */
    float f4_r_dc;                  /* The R in DC condition used to calculate current step and
                                                       store final result */
    float f4_meas_volt_sum;         /* The sum of sampled voltage in order to calculate mean voltage */
    float f4_meas_current_sum;      /* The sum of sampled current in order to calculate mean voltage */
    float f4_current_target;        /* The target current [A] that should be achieved by last voltage command*/
    float f4_voltage_step;          /* The voltage step should be increased to achieve the next target current */

    uint32_t u4_iteration_cnt;      /* The counter of resistance measurement iteration */
    uint32_t u4_count;              /* The general counter */
    uint32_t u4_meas_index;         /* The index of voltage error point that is under measurement */
} st_aid_volterr_t;

typedef enum
{
    AID_VOLTERR_RID_RLS,                            /* Identify R before VI measurement */
    AID_VOLTERR_RID_IGNORE                          /* Use initial R */
} e_volterr_cfg_rid_t;

/***********************************************************************************************************************
 Exported global variables
 **********************************************************************************************************************/
extern float aid_f4_volterr_current_tab[AID_VOLTERR_OUTPUT_TAB_SIZE];
extern float aid_f4_volterr_voltage_tab[AID_VOLTERR_OUTPUT_TAB_SIZE];
extern float aid_f4_volterr_vdc_ref;
extern float aid_f4_volterr_rdc;

/***********************************************************************************************************************
 Exported global functions (to be accessed by other files)
 **********************************************************************************************************************/
void aid_volterr_init(void);
uint16_t aid_volterr_act(void);
void aid_volterr_config(float f4_r_initval, float f4_i_lsb, e_volterr_cfg_rid_t e_rid_cfg);
void aid_volterr_config_current_step(uint16_t u2_ilsb);
void aid_volterr_reset(void);

#endif /* MIDDLE_IDENTIFICATION_R_AID_VOLTERR_H_ */
