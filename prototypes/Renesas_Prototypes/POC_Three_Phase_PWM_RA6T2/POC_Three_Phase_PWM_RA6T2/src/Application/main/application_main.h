/* Guard against multiple inclusion */
#ifndef APPLICATION_MAIN_H
#define APPLICATION_MAIN_H

#include <stdint.h>

typedef struct
{
  int16_t q;
  int16_t d;
} qd_t;

typedef struct
{
  int16_t alpha;
  int16_t beta;
} alphabeta_t;

#define APP_MAIN 0
#define ADC_EOC_ISR 1	

#define SQRT3FACTOR (uint16_t) 0xDDB4 /* = (16384 * 1.732051 * 2)*/
#define PWM_PERIOD_CYCLES  (uint16_t) 0x1770
#define MAX(a,b) (((a)>(b))?(a):(b)) 
#define UPDATE_LOCATION ADC_EOC_ISR

/***********************************************************************************************************************
* global functions
***********************************************************************************************************************/
/***********************************************************************************************************************
* Function Name : application_init
* Description   : Initialization 
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void application_init( void );

/***********************************************************************************************************************
* Function Name : application_main
* Description   : Initialization
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void application_main(void);




#endif /* APPLICATION_MAIN_H */