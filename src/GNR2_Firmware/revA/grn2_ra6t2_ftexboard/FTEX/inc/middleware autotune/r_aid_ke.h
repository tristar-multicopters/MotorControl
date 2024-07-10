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
 * File Name    : r_aid_ke.h
 * Version      : 1.0
 * Description  : Definitions for open-loop flux identification module
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 15.01.2007 1.00     First Release
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdint.h>

#ifndef MIDDLE_IDENTIFICATION_R_AID_KE_H_
    #define MIDDLE_IDENTIFICATION_R_AID_KE_H_

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
typedef enum {
    AID_SEQ_KE_INIT,
    AID_SEQ_KE_READY,
    AID_SEQ_KE_MEASURE,
    AID_SEQ_KE_CALC,
    AID_SEQ_KE_CHECK,
    AID_SEQ_KE_RESET,
    AID_SEQ_KE_COMPLETED
} e_aid_seq_ke_t;

/***********************************************************************************************************************
 Exported global variables
 **********************************************************************************************************************/
extern float  aid_f4_ke_open;
extern float  aid_f4_ke_closed_loop;
/***********************************************************************************************************************
 Exported global functions (to be accessed by other files)
 **********************************************************************************************************************/
uint16_t aid_ke_act(void);
void aid_ke_reset(void);
void aid_ke_config_assumed_inertia(float f4_assumed_inertia);
void aid_ke_config_no_stop_flag(uint8_t no_stop);

#endif /* MIDDLE_IDENTIFICATION_R_AID_KE_H_ */
