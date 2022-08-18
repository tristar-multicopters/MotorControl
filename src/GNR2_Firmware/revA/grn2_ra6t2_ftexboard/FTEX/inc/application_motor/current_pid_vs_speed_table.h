/**
    * @file  current_pid_vs_speed_table.h
    * @author Sami Bouzid, FTEX inc.
    * @brief This file instantiate the lookup tables needed to update dynamically the PID gains used for current control.
    *
*/

#include "stdint.h"
#include "lookup_table.h"


extern LookupTableHandle_t LookupTableM1IqKp;

extern LookupTableHandle_t LookupTableM1IqKi;

extern LookupTableHandle_t LookupTableM1IdKp;

extern LookupTableHandle_t LookupTableM1IdKi;

