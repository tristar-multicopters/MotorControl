/**
  * @file    lookup_table.c
  * @brief   This file provides firmware functions to get data from lookup table
  *          and interpolate between values.
  *
*/

/* Includes ------------------------------------------------------------------*/

#include "lookup_table.h"

/*
    Initializes lookup table module.
*/
void LookupTable_Init(LookupTableHandle_t *pHandle)
{

}

/*
    Calculate output of lookup table.
*/
int32_t LookupTable_CalcOutput(LookupTableHandle_t *pHandle, int32_t wInputdata)
{

    
    uint16_t hIndex;
    uint16_t hIndexRemainder;
    int32_t wOutput;
    
    if (wInputdata < pHandle->wXDataFirstValue)
    {
        wOutput = (int32_t) pHandle->pOutputTable[0];
    }
    else
    {
        int32_t wTemp = wInputdata - pHandle->wXDataFirstValue;
        hIndex = (uint16_t) (wTemp / pHandle->hXDataStep); // Calculate index of lookup table (floored division)
        if (hIndex >= pHandle->hTableLength-1)
        {
            wOutput = (int32_t) pHandle->pOutputTable[pHandle->hTableLength-1];
        }
        else
        {
            hIndexRemainder = (uint16_t) (wTemp % pHandle->hXDataStep); // Calculate remainder of index division above
            wOutput = pHandle->pOutputTable[hIndex] + (hIndexRemainder*
                            (pHandle->pOutputTable[hIndex+1]-pHandle->pOutputTable[hIndex]))
                             /pHandle->hXDataStep; // Compute output by interpolating between two values of the table
            
            if (wOutput > INT16_MAX)
            {
                wOutput = INT16_MAX;
            }
            if (wOutput < INT16_MIN)
            {
                wOutput = INT16_MIN;
            }
        }
    }
    
    return (int16_t) wOutput;
}

