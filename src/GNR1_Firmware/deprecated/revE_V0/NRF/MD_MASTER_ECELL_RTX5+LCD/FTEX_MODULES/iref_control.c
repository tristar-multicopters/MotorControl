/**
  ******************************************************************************
  * @file    iref_control.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles computation of current reference to motor
  *
	******************************************************************************
*/

#include "iref_control.h"


/* Functions ---------------------------------------------------- */

/**
  * @brief  It calculates the square root of a non-negative int32_t. It returns 0
  *         for negative int32_t.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
static int32_t MCM_Sqrt( int32_t wInput )
{
  int32_t wtemprootnew;

  if ( wInput > 0 )
  {
  uint8_t biter = 0u;
  int32_t wtemproot;

    if ( wInput <= ( int32_t )2097152 )
    {
      wtemproot = ( int32_t )128;
    }
    else
    {
      wtemproot = ( int32_t )8192;
    }

    do
    {
      wtemprootnew = ( wtemproot + wInput / wtemproot ) / ( int32_t )2;
      if ( wtemprootnew == wtemproot )
      {
        biter = 6u;
      }
      else
      {
        biter ++;
        wtemproot = wtemprootnew;
      }
    }
    while ( biter < 6u );

  }
  else
  {
    wtemprootnew = ( int32_t )0;
  }

  return ( wtemprootnew );
}

/**
 * @brief Initializes flux reference control module
 *
 *  @p pHandle : Pointer on Handle structure of IREF_Handle_t component
 */
void IREF_Init( IREF_Handle_t * pHandle )
{
}

/**
  * @brief Performs the flux control algorithm
  *  @p pHandle : Pointer on Handle structure of IREF_Handle_t component
  */
qd_t IREF_ComputeIref( IREF_Handle_t * pHandle, int16_t hIqref, int32_t wSpeed )
{
	qd_t iqdref = {0, 0};
	int32_t wAux, wIqSatSq, wIqSat;
	
	if ( wSpeed < pHandle->wLowSpeedThreshold )
	{
		wAux = hIqref * abs(pHandle->hLowSpeedFluxTorqueRatio);
		iqdref.d = (int16_t) wAux / INT16_MAX;
	}
	
  /* Saturation for Iqref */
  wIqSatSq =  pHandle->hMaxSqCurrModule - iqdref.d * iqdref.d;
  wIqSat = MCM_Sqrt( wIqSatSq );
	
	if ( hIqref > wIqSat )
  {
    iqdref.q = ( int16_t )wIqSat;
  }
  else if ( iqdref.q < -wIqSat )
  {
    iqdref.q = -( int16_t )wIqSat;
  }
  else
  {
  }
	
	return iqdref;
}


