#ifndef __P3_D70008_W
#define __P3_D70008_W

// Currently choosen motor for Nidec

/************** WHEEL SPEED SENSOR PARAMETERS  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION     1       // Wheel speed sensor cycle number for one wheel rotation
#define WHEEL_SPEED_SLOW_LOOP_COUNT             3       // Wheel speed sensor slow detect counter to get 750ms per function call
#define WHEEL_SPEED_SENSOR_CORRECTION_FACTOR    2       // Wheel speed sensor slow detect correction for a signal after two wheel spin detection 
#define WHEEL_SPEED_SLOW_LOOP_DETECT            true    // Wheel speed sensor slow detect counter flag activation

#endif