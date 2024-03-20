/**
    * @file  current_pid_vs_speed_table.c
    * @author Sami Bouzid, FTEX inc.
    * @brief This file instantiate the lookup tables needed to update dynamically the PID gains used for current control.
    *
*/

#include "vc_parameters.h"
#include "current_pid_vs_speed_table.h"



#define CURRENT_PID_VS_SPEED_LUT_SIZE                 2


#if MOTOR_SELECTION == MOTOR_AKM_128SX_350W

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

#elif MOTOR_SELECTION == MOTOR_AKM_128SX_500W

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

#elif MOTOR_SELECTION == MOTOR_AKM_128SX_750W

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

#elif MOTOR_SELECTION == MOTOR_BAFANG_G020_500W

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
    43,  /* old PI = 50 parameter tunning for Vibration */
    43,  /* old PI = 50 parameter tunning for Vibration */
};

#elif MOTOR_SELECTION == MOTOR_BAFANG_G60_750W

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

#elif MOTOR_SELECTION == MOTOR_BAFANG_G040_500W

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    132,
    132,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    20,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    124,
    124,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    20,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif MOTOR_SELECTION == MOTOR_BAFANG_G0900_750W

#define CURRENT_PID_LUT_SPEED_STEP                    700
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             300

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    132,
    132,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    20,   /* old PI = 50 parameter tunning for Vibration */ 
    20,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    124,
    124,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    20,	 /* old PI = 50 parameter tunning for Vibration */
    20,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif MOTOR_SELECTION == MOTOR_NIDEC_B900_V3

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

#elif MOTOR_SELECTION == MOTOR_BAFANG_G062_750W

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

#elif MOTOR_SELECTION == MOTOR_RS2_1200W

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

#elif MOTOR_SELECTION == MOTOR_TSUGAWA_L13S5_350W

#define CURRENT_PID_LUT_SPEED_STEP                    300
#define CURRENT_PID_LUT_SPEED_FIRST_VALUE             100

const int32_t IqKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    2500,
    2500,
};

const int32_t IqKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {

    10,   /* old PI = 50 parameter tunning for Vibration */ 
    10,   /* old PI = 50 parameter tunning for Vibration */

};

const int32_t IdKpVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    2500,
    2500,	
};

const int32_t IdKiVsSpeedTable[CURRENT_PID_VS_SPEED_LUT_SIZE] = {
    10,	 /* old PI = 50 parameter tunning for Vibration */
    10,	 /* old PI = 50 parameter tunning for Vibration */
};

#elif MOTOR_SELECTION == MOTOR_UTK_G250R_CA11_350W 

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
