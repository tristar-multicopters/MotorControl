/**
    * @file  ntc_table.h
    * @author Sami Bouzid, FTEX inc.
    * @brief This file declares the NTC (negative temperature coefficient) thermistor lookup table and its parameters to use with LookupTable component.
    *
*/

#include "stdint.h"
#include "lookup_table.h"
#include "motor_parameters.h"

#define NTC_CONTROLLER_LUT_SIZE                        31           // the number of correlation elements on this table
#define NTC_CONTROLLER_LUT_DIGITAL_STEP                1600         //100 is the tics steps for a 12bits ADC conversion
#define NTC_CONTROLLER_LUT_DIGITAL_FIRST_VALUE         16000        //1000 is the first value considered in the table that has a temperature correlation (7 degree C).
                                                         //*16 is the factor on tics conversion due variable casting

#if MOTOR_SELECTION == MOTOR_AKM_128SX_750W

#define NTC_MOTOR_LUT_SIZE                        38           // the number of correlation elements on this table
#define NTC_MOTOR_LUT_DIGITAL_STEP                1600         //100 is the tics steps for a 12bits ADC conversion
#define NTC_MOTOR_LUT_DIGITAL_FIRST_VALUE         4000        //250 is the first value considered in the table that has a temperature correlation (144 degree C).
                                                         //*16 is the factor on tics conversion due variable casting

#elif MOTOR_SELECTION == MOTOR_NIDEC_B900_V3

#define NTC_MOTOR_LUT_SIZE                        38           // the number of correlation elements on this table
#define NTC_MOTOR_LUT_DIGITAL_STEP                1600         //100 is the tics steps for a 12bits ADC conversion
#define NTC_MOTOR_LUT_DIGITAL_FIRST_VALUE         3200        //250 is the first value considered in the table that has a temperature correlation (144 degree C).
                 
                                        //*16 is the factor on tics conversion due variable casting
#elif MOTOR_SELECTION == MOTOR_AKM_128SX_500W

#define NTC_MOTOR_LUT_SIZE                        38           // the number of correlation elements on this table
#define NTC_MOTOR_LUT_DIGITAL_STEP                1600         //100 is the tics steps for a 12bits ADC conversion
#define NTC_MOTOR_LUT_DIGITAL_FIRST_VALUE         4000        //250 is the first value considered in the table that has a temperature correlation (144 degree C).
                                                         //*16 is the factor on tics conversion due variable casting
#else

#define NTC_MOTOR_LUT_SIZE                        0           // when there is no temp sensor
#define NTC_MOTOR_LUT_DIGITAL_STEP                0         
#define NTC_MOTOR_LUT_DIGITAL_FIRST_VALUE         0      


#endif

extern LookupTableHandle_t ControllerNTCLookupTable;
extern LookupTableHandle_t MotorNTCLookupTable;


