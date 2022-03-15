/* Includes ------------------------------------------------------------------*/
#include "derate.h"


/* Functions ---------------------------------------------------- */

static void UpdateDerateLevel( Derate_Handle_t * pHandle )
{
	int16_t wTemp;
	//wTemp = NTC_GetAvTemp_C( pHandle->pThermistor1 );
	
	uint16_t i = 0;
	while (i < pHandle->hTableSize)
	{
		if (wTemp < pHandle->pTemperatureTable[i])
		{
			break;
		}
		else
		{
			i++;
		}
	}
	pHandle->hDeratingLevel = i;
}


/**
 * @brief Initializes derate management
 *
 *  @p pHandle : Pointer on Handle structure of derate component
 */
__weak void Derate_Init( Derate_Handle_t * pHandle )
{	
	UpdateDerateLevel( pHandle );
}

/**
  * @brief  Returns torque value limited by derating
  *
  * @p pHandle : Pointer on Handle structure of derate component
	* @p torque : Initial torque value 
  */
int16_t Derate_CalcTorque( Derate_Handle_t * pHandle, int16_t torque)
{
	int32_t wAux;
	int16_t lTorque;
	
	UpdateDerateLevel( pHandle );
	
	if (pHandle->hDeratingLevel == 0)
	{
		lTorque = torque;
	}
	else
	{
		wAux = pHandle->pTorquePercentageTable[pHandle->hDeratingLevel - 1] * torque;
		lTorque = (int16_t) wAux / 100;
	}
	
	return lTorque;
}

