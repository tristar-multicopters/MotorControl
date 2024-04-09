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
 * File Name    : r_aid_ldq.h
 * Version      : 1.0
 * Description  : Definitions for inductance identification module
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdint.h>
#include "r_aid_core.h"
#ifndef MIDDLE_IDENTIFICATION_R_AID_LDQ_H_
    #define MIDDLE_IDENTIFICATION_R_AID_LDQ_H_

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
typedef enum {
    AID_SEQ_RLD_DFT_INIT,
    AID_SEQ_RLD_DFT_READY,
    AID_SEQ_RLD_DFT_MEASURE,
    AID_SEQ_RLD_DFT_CALC,
    AID_SEQ_RLD_DFT_CHECK,
    AID_SEQ_RLD_DFT_RESET,
    AID_SEQ_RLD_DFT_COMPLETED,
} e_aid_seq_rld_dft_t;

typedef enum {
    AID_SEQ_RLD_RLS_INIT,
    AID_SEQ_RLD_RLS_READY,
    AID_SEQ_RLD_RLS_MEASURE,
    AID_SEQ_RLD_RLS_CALC,
    AID_SEQ_RLD_RLS_CHECK,
    AID_SEQ_RLD_RLS_RESET,
    AID_SEQ_RLD_RLS_COMPLETED,
} e_aid_seq_rld_rls_t;

typedef enum {
    AID_SEQ_RLQ_DFT_INIT,
    AID_SEQ_RLQ_DFT_READY,
    AID_SEQ_RLQ_DFT_MEASURE,
    AID_SEQ_RLQ_DFT_CALC,
    AID_SEQ_RLQ_DFT_CHECK,
    AID_SEQ_RLQ_DFT_RESET,
    AID_SEQ_RLQ_DFT_COMPLETED
} e_aid_seq_rlq_dft_t;

typedef enum {
    AID_SEQ_RLQ_RLS_INIT,
    AID_SEQ_RLQ_RLS_READY,
    AID_SEQ_RLQ_RLS_MEASURE,
    AID_SEQ_RLQ_RLS_CALC,
    AID_SEQ_RLQ_RLS_CHECK,
    AID_SEQ_RLQ_RLS_RESET,
    AID_SEQ_RLQ_RLS_COMPLETED
} e_aid_seq_rlq_rls_t;

/***********************************************************************************************************************
 Exported global variables
 **********************************************************************************************************************/
extern float  aid_f4_r_dft;
extern float  aid_f4_r_rls;
extern float  aid_f4_ld_dft;
extern float  aid_f4_ld_rls;
extern float  aid_f4_lq_dft;
extern float  aid_f4_lq_rls;

/***********************************************************************************************************************
 Exported global functions (to be accessed by other files)
 **********************************************************************************************************************/
uint16_t aid_rld_dft_act(st_aid_signal_conf_t *st_signal_conf);
uint16_t aid_rld_rls_act(st_aid_signal_conf_t *st_signal_conf);
uint16_t aid_lq_dft_act(st_aid_signal_conf_t *st_signal_conf);
uint16_t aid_lq_rls_act(st_aid_signal_conf_t *st_signal_conf);
void aid_ldq_reset(void);
#endif /* MIDDLE_IDENTIFICATION_R_AID_LDQ_H_ */
