/**

* @file  ntc_table.c
* @author Luiz Martins, FTEX inc.
* @brief This file declares the NTC (negative temperature coefficient) thermistor lookup table and its parameters to use with LookupTable component.
* Added practical drift after thermal camera feedback
*/

#include "ntc_table.h"
#include "gnr_parameters.h"

#define NTC_CONTROLLER_LUT_SIZE                        31           // the number of correlation elements on this table
#define NTC_CONTROLLER_LUT_DIGITAL_STEP                1600         //100 is the tics steps for a 12bits ADC conversion
#define NTC_CONTROLLER_LUT_DIGITAL_FIRST_VALUE         16000        //1000 is the first value considered in the table that has a temperature correlation (7 degree C).
                                                         //*16 is the factor on tics conversion due variable casting

#if VEHICLE_SELECTION == VEHICLE_R48_750W

#define NTC_MOTOR_LUT_SIZE                        38           // the number of correlation elements on this table
#define NTC_MOTOR_LUT_DIGITAL_STEP                1600         //100 is the tics steps for a 12bits ADC conversion
#define NTC_MOTOR_LUT_DIGITAL_FIRST_VALUE         4000        //250 is the first value considered in the table that has a temperature correlation (144 degree C).
                                                         //*16 is the factor on tics conversion due variable casting

#elif VEHICLE_SELECTION == VEHICLE_NIDEC || VEHICLE_SELECTION == VEHICLE_PEGATRON

#define NTC_MOTOR_LUT_SIZE                        38           // the number of correlation elements on this table
#define NTC_MOTOR_LUT_DIGITAL_STEP                1600         //100 is the tics steps for a 12bits ADC conversion
#define NTC_MOTOR_LUT_DIGITAL_FIRST_VALUE         3200        //250 is the first value considered in the table that has a temperature correlation (144 degree C).
                 
                                        //*16 is the factor on tics conversion due variable casting

#else

#define NTC_MOTOR_LUT_SIZE                        0           // when there is no temp sensor
#define NTC_MOTOR_LUT_DIGITAL_STEP                0         
#define NTC_MOTOR_LUT_DIGITAL_FIRST_VALUE         0        


#endif


const int32_t NTCControllerTemperatureTable[NTC_CONTROLLER_LUT_SIZE] =        //revised the function to calculate temperature values of the controller after the NTC test.
{
    7,                                                   //Added few values to check the behaviour for low temperature values.
    9,            
    11,
    13,
    15,
    17,
    19,
    21,
    23,
    25,
    27,
    29,
    31,
    33,
    35,
    37,
    39,
    41,
    44,
    46,
    49,
    52,
    55,
    59,
    63,
    68,
    73,
    80,
    89,
    103,
    129,
};

#if VEHICLE_SELECTION == VEHICLE_R48_750W

const int32_t NTCMotorTemperatureTable[NTC_MOTOR_LUT_SIZE] =        //function to calculate temperature values of the motor
{                                                                   //table calculated here: https://docs.google.com/spreadsheets/d/1fEv8Z7ZyeggrdsOoPRL44zLIx-ng7lT2qx_m4sxkFug/edit#gid=0
    144,
    104,
    90,
    81,
    73,
    67,
    62,
    58,
    54,
    50,
    47,
    44,
    41,
    38,
    35,
    33,
    30,
    28,
    26,
    23,
    21,
    19,
    17,
    14,
    12,
    10,
    7,
    5,
    2,
    0,
    -3,
    -6,
    -9,
    -13,
    -17,
    -22,
    -28,
    -34,
};

#elif VEHICLE_SELECTION == VEHICLE_NIDEC || VEHICLE_SELECTION == VEHICLE_PEGATRON

const int32_t NTCMotorTemperatureTable[NTC_MOTOR_LUT_SIZE] =        //function to calculate temperature values of the motor
{                                                                                                                                     //table calculated here: https://docs.google.com/spreadsheets/d/1fEv8Z7ZyeggrdsOoPRL44zLIx-ng7lT2qx_m4sxkFug/edit#gid=0
    150,
    113,
    96,
    85,
    77,
    70,
    65,
    60,
    56,
    52,
    48,
    45,
    42,
    39,
    37,
    34,
    32,
    29,
    27,
    25,
    22,
    20,
    18,
    15,
    13,
    11,
    9,
    6,
    4,
    1,
    -2,
    -5,
    -8,
    -11,
    -15,
    -19,
    -25,
    -32,
};

#else 

const int32_t NTCMotorTemperatureTable[NTC_MOTOR_LUT_SIZE] = {}; //Empty array when no temp sensor

#endif


LookupTableHandle_t ControllerNTCLookupTable =
 {
    .hXDataStep = NTC_CONTROLLER_LUT_DIGITAL_STEP,
    .wXDataFirstValue = NTC_CONTROLLER_LUT_DIGITAL_FIRST_VALUE,
    .hTableLength = NTC_CONTROLLER_LUT_SIZE,
    .pOutputTable = NTCControllerTemperatureTable,
};
 
LookupTableHandle_t MotorNTCLookupTable =
 {
    .hXDataStep = NTC_MOTOR_LUT_DIGITAL_STEP,
    .wXDataFirstValue = NTC_MOTOR_LUT_DIGITAL_FIRST_VALUE,
    .hTableLength = NTC_MOTOR_LUT_SIZE,
    .pOutputTable = NTCMotorTemperatureTable,
};
 