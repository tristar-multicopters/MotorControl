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
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pOutputTable != NULL);
    ASSERT(pHandle->hXDataStep != 0);
}

/*
    Calculate output of lookup table.
*/
int32_t LookupTable_CalcOutput(LookupTableHandle_t *pHandle, int32_t wInputdata)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pOutputTable != NULL);
    ASSERT(pHandle->hXDataStep != 0);
    
    uint16_t hIndex;
    uint16_t hIndexRemainder;
    int32_t wOutput;
    
    if (wInputdata < pHandle->wXDataFirstValue) // Is the value smaller than the table input minimum ?
    {
        wOutput = (int32_t) pHandle->pOutputTable[0];
        pHandle->OutsideTable = true; // Set the flag to show we are outside of the table
    }
    else
    {
        int32_t wTemp = wInputdata - pHandle->wXDataFirstValue;
        hIndex = (uint16_t) (wTemp / pHandle->hXDataStep); // Calculate index of lookup table (floored division)
        
        if (hIndex > pHandle->hTableLength-1) // Is the value bigger then the table input maximum
        {
            wOutput = (int32_t) pHandle->pOutputTable[pHandle->hTableLength-1];
            pHandle->OutsideTable = true; // Set the flag to show we are outside of the table
        }
        else if (hIndex == pHandle->hTableLength-1)
        {
            wOutput = (int32_t) pHandle->pOutputTable[pHandle->hTableLength-1];
            pHandle->OutsideTable = false;
        }
        else
        {
            hIndexRemainder = (uint16_t) (wTemp % pHandle->hXDataStep); // Calculate remainder of index division above
            wOutput = pHandle->pOutputTable[hIndex] + (hIndexRemainder*
                      (pHandle->pOutputTable[hIndex+1]-pHandle->pOutputTable[hIndex]))
                      /pHandle->hXDataStep; // Compute output by interpolating between two values of the table
            
            pHandle->OutsideTable = false;
            
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

