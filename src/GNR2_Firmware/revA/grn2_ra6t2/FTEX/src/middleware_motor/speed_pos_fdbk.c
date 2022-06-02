/**
  ******************************************************************************
  * @file    speed_pos_fdbk.c
  * @author  FTEX inc
  * @brief   This file provides firmware functions that implement the  features
  *          of the Speed & Position Feedback component of the Motor Control SDK.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/**
* Function to calculate and return electrical angle from electrical speed dpp.
*/
int16_t SPD_GetElAngle( SpeednPosFdbk_Handle_t * pHandle )
{
    return ( pHandle->hElAngle );
}

/**
* Funtion to return mechanical angle which is part of speed position feedback handle
*/
int32_t SPD_GetMecAngle( SpeednPosFdbk_Handle_t * pHandle )
{
    return ( pHandle->wMecAngle );
}

/**
* Function used to return average mechanical speed which is a part of speed position feedback handle
*/
int16_t SPD_GetAvrgMecSpeedUnit( SpeednPosFdbk_Handle_t * pHandle )
{
    return ( pHandle->hAvrMecSpeedUnit );
}

/**
* Function used to return electrical speed expressed in digital pulse per control period which is a part of speed position feedback handle.
*/
int16_t SPD_GetElSpeedDpp( SpeednPosFdbk_Handle_t * pHandle )
{
    return ( pHandle->hElSpeedDpp );
}

/**
* Function used to return electrical speed expressed in digital pulse per control period which is a part of speed position feedback handle. 
*/
int16_t SPD_GetInstElSpeedDpp( SpeednPosFdbk_Handle_t * pHandle )
{
    return ( pHandle->InstantaneousElSpeedDpp );
}

/**
* Function returns the result of the last reliability check performed.
*/
bool SPD_Check( SpeednPosFdbk_Handle_t * pHandle )
{
    bool SpeedSensorReliability = true;
    if ( pHandle->bSpeedErrorNumber ==
    pHandle->bMaximumSpeedErrorsNumber )
    {
        SpeedSensorReliability = false;
    }
    return ( SpeedSensorReliability );
}

/**
  * @brief  It computes and returns the reliability state of the sensor; reliability is measured with
  *         reference to parameters hMaxReliableElSpeedUnit, hMinReliableElSpeedUnit, bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */
bool SPD_IsMecSpeedReliable( SpeednPosFdbk_Handle_t * pHandle, int16_t * pMecSpeedUnit )
{
    bool SpeedSensorReliability = true;
    uint8_t bSpeedErrorNumber;
    uint8_t bMaximumSpeedErrorsNumber = pHandle->bMaximumSpeedErrorsNumber;
    bool SpeedError = false;
    uint16_t hAbsMecSpeedUnit, hAbsMecAccelUnitP;
    int16_t hAux;
    bSpeedErrorNumber = pHandle->bSpeedErrorNumber;
    if ( *pMecSpeedUnit < 0 )  // Compute absoulte value of mechanical speed
    {
        hAux = -( *pMecSpeedUnit );
        hAbsMecSpeedUnit = ( uint16_t )( hAux );
    }
    else
    {
        hAbsMecSpeedUnit = ( uint16_t )( *pMecSpeedUnit );
    }
    if ( hAbsMecSpeedUnit > pHandle->hMaxReliableMecSpeedUnit )  // Check if absolute mechanical speed is below maximum reliable mechanical speed
    {
        SpeedError = true;
    }
    if ( hAbsMecSpeedUnit < pHandle->hMinReliableMecSpeedUnit )  // Check if absolute mechanical speed is above minimum reliable mechanical speed
    {
        SpeedError = true;
    }
    if ( pHandle->hMecAccelUnitP < 0 )  // Compute absoulte value of mechanical acceleration
    {
        hAux = -( pHandle->hMecAccelUnitP );
        hAbsMecAccelUnitP = ( uint16_t )( hAux );
    }
    else
    {
        hAbsMecAccelUnitP = ( uint16_t )( pHandle->hMecAccelUnitP );  
    }
    if ( hAbsMecAccelUnitP > pHandle->hMaxReliableMecAccelUnitP )  // Check if absolute mechanical acceleration is below maximum reliable mechanical acceleration
    {
        SpeedError = true;
    }
    if ( SpeedError == true )
    {
        if ( bSpeedErrorNumber < bMaximumSpeedErrorsNumber )
        {
            bSpeedErrorNumber++;
        }
    }
    else
    {
        if ( bSpeedErrorNumber < bMaximumSpeedErrorsNumber )
        {
            bSpeedErrorNumber = 0u;
        }
    }
    if ( bSpeedErrorNumber == bMaximumSpeedErrorsNumber )
    {
        SpeedSensorReliability = false;
    }
    pHandle->bSpeedErrorNumber = bSpeedErrorNumber;  // Update number of speed errors in speed position feedback handle.
    return ( SpeedSensorReliability );
}

/**
* This method returns the average mechanical rotor speed expressed in "S16Speed"
*/
int16_t SPD_GetS16Speed( SpeednPosFdbk_Handle_t * pHandle )
{
    int32_t wAux = ( int32_t ) pHandle->hAvrMecSpeedUnit;
    wAux *= INT16_MAX;
    wAux /= ( int16_t ) pHandle->hMaxReliableMecSpeedUnit;
    return ( int16_t )wAux;
}

/**
* This method returns the coefficient used to transform electrical to mechanical quantities and viceversa.
*/
uint8_t SPD_GetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle )
{
    return ( pHandle->bElToMecRatio );
}

/**
* This method sets the coefficient used to transform electrical to mechanical quantities and viceversa.
*/
void SPD_SetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle, uint8_t bPP )
{
    pHandle->bElToMecRatio = bPP;
}
