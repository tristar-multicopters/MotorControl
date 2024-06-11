/**
    * @file  current_pid_vs_speed_table.c
    * @author Sami Bouzid, FTEX inc.
    * @brief This file instantiate the lookup tables needed to update dynamically the PID gains used for current control.
    *
*/

#include "current_pid_vs_speed_table.h"
LookupTableHandle_t LookupTableM1IqKp =
{
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
};

LookupTableHandle_t LookupTableM1IqKi =
{
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
};

LookupTableHandle_t LookupTableM1IdKp =
{
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
};

LookupTableHandle_t LookupTableM1IdKi =
{
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
};
