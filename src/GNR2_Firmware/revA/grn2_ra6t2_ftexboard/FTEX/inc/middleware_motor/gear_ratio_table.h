/**
    * @file  gear_ratio_table.h
    * @author Natasha Yang, FTEX inc.
    * @brief This file declares the gear ratio lookup tables.
    *
*/

#ifndef __GEAR_RATIO_TABLE_H
#define __GEAR_RATIO_TABLE_H

#include "stdint.h"

#if VEHICLE_SELECTION == VEHICLE_NIDEC || VEHICLE_SELECTION == VEHICLE_PEGATRON

#define NUM_GEARS                                           9           // the number of gears on the bike

const uint8_t gearRatioTable[NUM_GEARS] =                 //gear ratios for each gear on the bike
{
    9,
    11,
    13,
    15,
    18,
    20,
    23,
    27,
    32
};

//else there is 1 gear and gear ratio is 1, for hub drives (these values are not being used for calculations)

#else

#define NUM_GEARS                                           1           // the number of gears on the bike

const uint8_t gearRatioTable[NUM_GEARS] =                 //gear ratios for each gear on the bike
{
    1,
};

#endif

#endif	