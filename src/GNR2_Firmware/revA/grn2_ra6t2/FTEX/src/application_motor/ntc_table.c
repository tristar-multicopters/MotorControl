/**
    * @file  ntc_table.c
    * @author Sami Bouzid, FTEX inc.
    * @brief This file declares the NTC (negative temperature coefficient) thermistor lookup table and its parameters to use with LookupTable component.
    *
*/

#include "ntc_table.h"

#define NTC_LUT_SIZE                        19
#define NTC_LUT_DIGITAL_STEP                2000
#define NTC_LUT_DIGITAL_FIRST_VALUE         25000

const int32_t NTCTemperatureTable[NTC_LUT_SIZE] = {
    20,
    22,
    24,
    26,
    28,
    31,
    34,
    37,
    40,
    44,
    48,
    52,
    57,
    62,
    67,
    73,
    80,
    87,
    95,
};

LookupTableHandle_t NTCLookupTable = 
{
  .hXDataStep = NTC_LUT_DIGITAL_STEP,
  .wXDataFirstValue = NTC_LUT_DIGITAL_FIRST_VALUE,
  .hTableLength = NTC_LUT_SIZE,
  .pOutputTable = NTCTemperatureTable,
};

