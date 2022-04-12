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