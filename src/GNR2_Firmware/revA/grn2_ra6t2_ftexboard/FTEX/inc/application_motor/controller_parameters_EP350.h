/**
  * @file    controller_parameters_EP350.h
  * @brief   This file contains the parameters needed for EP350.
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __CONTROLLER_PARAMETERS_EP350_H
#define __CONTROLLER_PARAMETERS_EP350_H


/************************** Current Sensing and Protection Parameters ************************/

#define AMPLIFICATION_GAIN              0.01        // V/A of current sensors, for scaling                                      //NEEDS TO BE VERIFIED
                                                    // Values are on p.15 of this doc (MEASmg): https://www.infineon.com/dgdl/Infineon-TLI4971_ProgGuide-ApplicationNotes-v01_30-EN.pdf?fileId=5546d4626bfb5124016c1a294ad5286e
                           
#define PEAK_CURRENT_CONTROLLER_amps    55          //peak current of controller in amps                                        //NEEDS TO BE VERIFIED                           
#define OCSP_SAFETY_MARGIN_amps         85         // Measured current amplitude can be until SOCP_SAFETY_MARGIN higher        //NEEDS TO BE VERIFIED
#define OCSP_MAX_CURRENT_amps           85         // Max current that can be reached before triggering software overcurrent   //NEEDS TO BE VERIFIED

#define HARDWARE_OCD2    OCD2_ENABLED     // OCD2_ENABLED to completley disable PWM using PEOG in timer0                        //NEEDS TO BE VERIFIED
                                          // OCD2_DISABLED to disable any Hardware OverCurrent handling


/************* Temperature and Overcurrent Protection Parameters *************/

#define OV_TEMP_CONTROLLER_THRESHOLD_C  75         // Heatsink overtemperature threshold before thermal shutdown. Celsius degrees               //NEEDS TO BE VERIFIED
#define OV_TEMP_CONTROLLER_HYSTERESIS_C 15         // Heatsink overtemperature hysteresis after a thermal shutdown occurred. Celsius degrees    //NEEDS TO BE VERIFIED


/************* Transistor Switching Parameters *************/

#define DEADTIME_NS                  250         // Dead-time to be inserted by FW                                                              //NEEDS TO BE VERIFIED


/************* Bus Voltage Sensing ****************/

#define VBUS_PARTITIONING_FACTOR      0.03985 /*!< It expresses how much the Vbus is attenuated before being converted into digital value */

#endif
