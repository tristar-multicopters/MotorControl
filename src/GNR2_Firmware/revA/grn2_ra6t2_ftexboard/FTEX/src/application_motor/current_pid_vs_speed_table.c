/**
    * @file  current_pid_vs_speed_table.c
    * @author Sami Bouzid, FTEX inc.
    * @brief This file instantiate the lookup tables needed to update dynamically the PID gains used for current control.
    *
*/

#include "gnr_parameters.h"
#include "current_pid_vs_speed_table.h"



#define CURRENT_PID_VS_SPEED_LUT_SIZE                 2


#if VEHICLE_SELECTION == VEHICLE_GRIZZLY

#define CURRENT_PID_LUT_SPEED_STEP                    100
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             400

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    600
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    50,
    200,
};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    600,
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    50,
    200,
};

#elif VEHICLE_SELECTION == VEHICLE_EBGO

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    300
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    50,
    50,
};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    300,
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    50,
    50,
};

#elif VEHICLE_SELECTION == VEHICLE_E_CELLS

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    800,
    600
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    600,
    1000,
};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    800,
    600,
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    2000,
    3000,
};

#endif

LookupTableHandle_t LookupTableM1IqKp =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
  .pOutputTable = IqKpVsSpeedTable,
};

LookupTableHandle_t LookupTableM1IqKi =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
  .pOutputTable = IqKiVsSpeedTable,
};

LookupTableHandle_t LookupTableM1IdKp =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
  .pOutputTable = IdKpVsSpeedTable,
};

LookupTableHandle_t LookupTableM1IdKi =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
  .pOutputTable = IdKiVsSpeedTable,
};

LookupTableHandle_t LookupTableM1RotorPosObsKp =
{
  .hXDataStep = CURRENT_PID_LUT_SPEED_STEP,
  .wXDataFirstValue = CURRENT_PID_LUT_SPEED_FIRST_VALUE,
  .hTableLength = CURRENT_PID_VS_SPEED_LUT_SIZE,
  .pOutputTable = IdKiVsSpeedTable,
};
