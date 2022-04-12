/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWMNCURRFDBK_H
#define PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* Includes ------------------------------------------------------------------*/
#include "mc_types.h"
#include "hal_data.h"

/* Exported defines ------------------------------------------------------------*/

#define SECTOR_1  0u
#define SECTOR_2  1u
#define SECTOR_3  2u
#define SECTOR_4  3u
#define SECTOR_5  4u
#define SECTOR_6  5u
#define SQRT3FACTOR (uint16_t) 0xDDB4 /* = (16384 * 1.732051 * 2)*/

/* Exported types ------------------------------------------------------------*/

#define PWM_PERIOD_CYCLES  (uint16_t) 0x1770



/* Function Prototypes */

/*  Converts input voltage components Valfa, beta into duty cycles and feed it to the inverter */
uint16_t PWMC_SetPhaseVoltage(alphabeta_t Valfa_beta );



#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PWMNCURRFDBK_H */