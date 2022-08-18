/**
    * @file  ntc_table.h
    * @author Sami Bouzid, FTEX inc.
    * @brief This file declares the NTC lookup table and its parameters to use with LookupTable component.
    *
*/

#include "stdint.h"

#define NTC_LUT_SIZE                        19
#define NTC_LUT_DIGITAL_STEP                2000
#define NTC_LUT_DIGITAL_FIRST_VALUE         25000
extern const int32_t NTCLookupTable[NTC_LUT_SIZE];
