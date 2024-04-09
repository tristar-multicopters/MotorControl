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
 * File Name    : r_aid_jd.h
 * Version      : 1.0
 * Description  : Definitions for inertia & friction identification module
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdint.h>

#ifndef MIDDLE_IDENTIFICATION_R_AID_JD_H_
    #define MIDDLE_IDENTIFICATION_R_AID_JD_H_
/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
typedef enum {
    AID_SEQ_J_INIT,
    AID_SEQ_J_READY,
    AID_SEQ_J_MEASURE,
    AID_SEQ_J_CALC,
    AID_SEQ_J_CHECK,
    AID_SEQ_J_RESET,
    AID_SEQ_J_COMPLETED
} e_aid_seq_j_t;

/***********************************************************************************************************************
 Exported global variables
 **********************************************************************************************************************/
extern float aid_f4_j_dft;
extern float aid_f4_d_dft;
extern float aid_f4_j_rls;
extern float aid_f4_d_rls;

/***********************************************************************************************************************
 Exported global functions (to be accessed by other files)
 **********************************************************************************************************************/
uint16_t    aid_jd_act(void);
void        aid_jd_reset(void);
void        aid_jd_config_inertia_range(float f4_inertia_range);
void        aid_jd_config_assumed_inertia(float f4_assumed_inertia);
#endif /* MIDDLE_IDENTIFICATION_R_AID_JD_H_ */
