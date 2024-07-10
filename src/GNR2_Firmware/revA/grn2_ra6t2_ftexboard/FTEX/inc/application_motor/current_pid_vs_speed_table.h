/**
    * @file  current_pid_vs_speed_table.h
    * @author Sami Bouzid, FTEX inc.
    * @brief This file instantiate the lookup tables needed to update dynamically the PID gains used for current control.
    *
*/

#ifndef __CURRENT_PID_VS_SPEED_TABLE_H
#define __CURRENT_PID_VS_SPEED_TABLE_H

#include "stdint.h"
#include "lookup_table.h"
#include "drive_parameters.h"

#define CURRENT_PID_VS_SPEED_LUT_SIZE                 2

extern LookupTableHandle_t LookupTableM1IqKp;

extern LookupTableHandle_t LookupTableM1IqKi;

extern LookupTableHandle_t LookupTableM1IdKp;

extern LookupTableHandle_t LookupTableM1IdKi;

#endif

