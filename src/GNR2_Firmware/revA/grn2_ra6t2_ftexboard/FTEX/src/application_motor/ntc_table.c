/**

* @file  ntc_table.c
* @author Luiz Martins, FTEX inc.
* @brief This file declares the NTC (negative temperature coefficient) thermistor lookup table and its parameters to use with LookupTable component.
* Added practical drift after thermal camera feedback
*/

#include "ntc_table.h"

#define NTC_LUT_SIZE                        31           // the number of correlation elements on this table
#define NTC_LUT_DIGITAL_STEP                1600         //100 is the tics steps for a 12bits ADC conversion
#define NTC_LUT_DIGITAL_FIRST_VALUE         16000        //1000 is the first value considered in the table that has a temperature correlation (7 degree C).
                                                         //*16 is the factor on tics conversion due variable casting


const int32_t NTCInverterTemperatureTable[NTC_LUT_SIZE] =        //revised the function to calculate temperature values of the inverter after the NTC test.
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

const int32_t NTCMotorTemperatureTable[NTC_LUT_SIZE] =        //function to calculate temperature values of the motor, values are currently inaccuarte
{
    129,                                                 
    103,
    89,
    80,
    73,
    68,
    63,
    59,
    55,
    52,
    49,
    46,
    44,
    41,
    39,
    37,
    35,
    33,
    31,
    29,
    27,
    25,
    23,
    21,
    19,
    17,
    15,
    13,
    11,
    9,
    7,
};


LookupTableHandle_t InverterNTCLookupTable =
 {
    .hXDataStep = NTC_LUT_DIGITAL_STEP,
    .wXDataFirstValue = NTC_LUT_DIGITAL_FIRST_VALUE,
    .hTableLength = NTC_LUT_SIZE,
    .pOutputTable = NTCInverterTemperatureTable,
};
 
LookupTableHandle_t MotorNTCLookupTable =
 {
    .hXDataStep = NTC_LUT_DIGITAL_STEP,
    .wXDataFirstValue = NTC_LUT_DIGITAL_FIRST_VALUE,
    .hTableLength = NTC_LUT_SIZE,
    .pOutputTable = NTCMotorTemperatureTable,
};
 