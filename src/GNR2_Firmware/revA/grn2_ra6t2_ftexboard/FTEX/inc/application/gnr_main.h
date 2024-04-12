/**
  * @file    gnr_main.h
  * @brief   This file is the main application of the ganrunner motor controller firmware
    */

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

#include "motor_parameters.h"



/* Defines --------------------------------------------------------------------*/

//#define CMSIS_device_header "RTE_Patch.h"

/* Structures ------------------------------------------------------------------*/



/* Variables --------------------------------------------------------------------*/



/* Function Prototypes ------------------------------------------------------------*/

void gnr_main(void);



#endif /* __GNR_MAIN_H */