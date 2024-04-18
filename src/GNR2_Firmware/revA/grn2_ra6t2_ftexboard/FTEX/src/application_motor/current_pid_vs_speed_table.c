/**
    * @file  current_pid_vs_speed_table.c
    * @author Sami Bouzid, FTEX inc.
    * @brief This file instantiate the lookup tables needed to update dynamically the PID gains used for current control.
    *
*/

#include "current_pid_vs_speed_table.h"
LookupTableHandle_t LookupTableM1IqKp =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
};

LookupTableHandle_t LookupTableM1IqKi =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
};

LookupTableHandle_t LookupTableM1IdKp =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
};

LookupTableHandle_t LookupTableM1IdKi =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
};
