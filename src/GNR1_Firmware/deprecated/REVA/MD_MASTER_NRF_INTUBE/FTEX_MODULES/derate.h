/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DERATE_H
#define __DERATE_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#define DERATE_TABLE_SIZE 2

/**
  * @brief Derate_Handle_t structure used for derate management
  *
  */
typedef struct
{
  //NTC_Handle_t * pThermistor1;   			/**< Thermistor1 handle structure */

  uint16_t hDeratingLevel;      			/**< Current derating level */
	
	int16_t pTemperatureTable[DERATE_TABLE_SIZE];				/**< Ascending temperature table in degree C */
	uint16_t pTorquePercentageTable[DERATE_TABLE_SIZE];		/**< Torque percentage table associated with pTemperatureTable */
	uint16_t hTableSize;
	
	int16_t hTorque;									/**< Last computed torque value */
	
} Derate_Handle_t;

/* Initialize derating parameters */
void Derate_Init( Derate_Handle_t * pHandle );

/* Compute derated torque */
int16_t Derate_CalcTorque( Derate_Handle_t * pHandle, int16_t torque);


#endif /* __DERATE_H */
