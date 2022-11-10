/**
  * @file    lookup_table.h
  * @brief   This file provides firmware functions to get data from lookup table
  *          and interpolate between values.
*/

#ifndef __LOOKUP_TABLE_H
#define __LOOKUP_TABLE_H

#include "stdint.h"
#include "ASSERT_FTEX.h"

typedef struct
{
    const uint16_t hXDataStep;             /* Step size between values of x-data */
    const int32_t wXDataFirstValue;        /* Value of first element of x-data */
    const uint16_t hTableLength;           /* Length of lookup table */
    const int32_t * pOutputTable;          /* Pointer to instance of y-data table */
    bool  OutsideTable;                    /* Flag that indicates if the received value is outside of the expected table input range*/
} LookupTableHandle_t;

/**
 * 	@brief Initializes lookup table module.
 * 	@param pHandle : Pointer on Handle structure of LookupTable component
 */
void LookupTable_Init(LookupTableHandle_t *pHandle);

/**
 * 	@brief Calculate output of lookup table by using interpolation between values if needed.
            If input is out of range of x-data, keep first or last value of the lookup table.
 * 	@param pHandle : Pointer on Handle structure of LookupTable component
 * 	@param wInputdata : Input data to feed lookup table
 */
int32_t LookupTable_CalcOutput(LookupTableHandle_t *pHandle, int32_t wInputdata);


#endif
