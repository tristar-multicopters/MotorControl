/**
    * @file  ntc_table.h
    * @author Sami Bouzid, FTEX inc.
    * @brief This file declares the NTC lookup table and its parameters to use with LookupTable component.
    *
*/

#include "stdint.h"

#define NTC_LUT_SIZE                        5
#define NTC_LUT_DIGITAL_STEP                1000
#define NTC_LUT_DIGITAL_FIRST_VALUE         5000
extern const int32_t NTCLookupTable[NTC_LUT_SIZE];
