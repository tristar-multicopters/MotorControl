/**
  * @file    power_stage_parameters.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a power stage.
  *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWER_STAGE_PARAMETERS_H
#define __POWER_STAGE_PARAMETERS_H


/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR      0.0354 /*!< It expresses how
                                                       much the Vbus is attenuated
                                                       before being converted into
                                                       digital value */

#define AMPLIFICATION_GAIN            0.01   /* V/A of current sensors */


#endif /*__POWER_STAGE_PARAMETERS_H*/
