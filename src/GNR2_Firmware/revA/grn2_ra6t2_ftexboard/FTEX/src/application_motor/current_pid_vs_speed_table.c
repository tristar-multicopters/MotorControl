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

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {  /* all PID gains are computed by Autotune */
    216,   
    600
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    60,
    200,
};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    216,
    600,
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    60,
    200,
};

#elif VEHICLE_SELECTION == VEHICLE_E_CELLS

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             150

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    90,
    300
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    27,
    200,
};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    87,
    300,
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    26,
    200,
};

#elif VEHICLE_SELECTION == VEHICLE_R48_750W

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    300,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    20,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    300,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    20,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_WHEEL

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    300,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    200,   /* old PI = 50 parameter tunning for Vibration */ 
    200,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    300,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    20,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
};
#elif VEHICLE_SELECTION == VEHICLE_TSUGAWA

#define CURRENT_PID_LUT_SPEED_STEP                    300
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             100

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    3500,
    3500,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    500,   /* old PI = 50 parameter tunning for Vibration */ 
    500,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    2500,
    2500,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    500,	 /* old PI = 50 parameter tunning for Vibration */
    500,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    150,
    150,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    10,   /* old PI = 50 parameter tunning for Vibration */ 
    10,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    150,
    150,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    10,	 /* old PI = 50 parameter tunning for Vibration */
    10,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_QUIETKAT

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    90,
    300
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    27,
    200,
};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    87,
    300,
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    26,
    200,
};

#elif VEHICLE_SELECTION == VEHICLE_RBS_MB

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    272,
    272,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    20,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    183,
    183,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    20,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_URBAN

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    171,
    171,

};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    48,   /* old PI = 50 parameter tunning for Vibration */ 
    48,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    154,
    154,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    43,	 /* old PI = 50 parameter tunning for Vibration */
    43,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_VELEC_CITI_500W

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    171,
    171,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    20,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    154,
    154,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    20,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_A2_350W

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    225,
    225,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    20,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    163,
    163,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    20,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_UTK_350W 

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    262,
    262,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    40,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    171,
    171,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    40,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_A2_500W

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    171,
    171,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    20,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    154,
    154,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    20,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
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
