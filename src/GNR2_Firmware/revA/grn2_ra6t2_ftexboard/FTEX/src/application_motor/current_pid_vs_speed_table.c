/**
    * @file  current_pid_vs_speed_table.c
    * @author Sami Bouzid, FTEX inc.
    * @brief This file instantiate the lookup tables needed to update dynamically the PID gains used for current control.
    *
*/

#include "gnr_parameters.h"
#include "current_pid_vs_speed_table.h"



#define CURRENT_PID_VS_SPEED_LUT_SIZE                 2


#if VEHICLE_SELECTION == VEHICLE_A2_350W

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
    20,     /* old PI = 50 parameter tunning for Vibration */
    20,     /* old PI = 50 parameter tunning for Vibration */
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
    20,     /* old PI = 50 parameter tunning for Vibration */
    20,     /* old PI = 50 parameter tunning for Vibration */
}; 

#elif VEHICLE_SELECTION == VEHICLE_E_CELLS

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             250

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    300
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    15,
    200,
};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    300,
    300,
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    10,
    200,
};

#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    900,
    900,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    10,   /* old PI = 50 parameter tunning for Vibration */ 
    10,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    80,
    80,    
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    10,     /* old PI = 50 parameter tunning for Vibration */
    10,     /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_PEGATRON

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    900,
    900,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    10,   /* old PI = 50 parameter tunning for Vibration */ 
    10,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    80,
    80,    
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    10,     /* old PI = 50 parameter tunning for Vibration */
    10,     /* old PI = 50 parameter tunning for Vibration */
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
    20,     /* old PI = 50 parameter tunning for Vibration */
    20,     /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_SUPER73

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    72,
    72,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    20,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    54,
    54,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    15,	 /* old PI = 50 parameter tunning for Vibration */
    15,	 /* old PI = 50 parameter tunning for Vibration */
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
    500,     /* old PI = 50 parameter tunning for Vibration */
    500,     /* old PI = 50 parameter tunning for Vibration */
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
    40,     /* old PI = 50 parameter tunning for Vibration */
    20,     /* old PI = 50 parameter tunning for Vibration */
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
    20,     /* old PI = 50 parameter tunning for Vibration */
    20,     /* old PI = 50 parameter tunning for Vibration */
};

#elif VEHICLE_SELECTION == VEHICLE_MAHLE

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    119,
    119,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    31,   /* old PI = 50 parameter tunning for Vibration */ 
    31,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    64,
    64,    
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    18,     /* old PI = 50 parameter tunning for Vibration */
    18,     /* old PI = 50 parameter tunning for Vibration */
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
