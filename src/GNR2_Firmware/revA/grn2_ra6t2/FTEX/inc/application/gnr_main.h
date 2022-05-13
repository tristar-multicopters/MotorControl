#ifndef __GNR_MAIN_H
#define __GNR_MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

#include "hal_data.h"
//#include "../../src/bsp/mcu/all/bsp_clocks.h"
#include "RTE_Components.h"
#include <cmsis_os2.h>
#include <rtx_os.h>
//#include "EventRecorder.h"



/* Defines --------------------------------------------------------------------*/

//#define CMSIS_device_header "RTE_Patch.h"

#define UNUSED_VARIABLE(X)  ((void)(X))
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)
#define UNUSED_RETURN_VALUE(X) UNUSED_VARIABLE(X)

/* Structures ------------------------------------------------------------------*/



/* Variables --------------------------------------------------------------------*/



/* Function Prototypes ------------------------------------------------------------*/

void gnr_main(void);

__NO_RETURN void TSK_0 (void * pvParameter);

__NO_RETURN void TSK_1 (void * pvParameter);

__NO_RETURN void TSK_2 (void * pvParameter);

__NO_RETURN void TSK_3 (void * pvParameter);



#endif /* __GNR_MAIN_H */