#ifndef __BAFANG_G60_750W
#define __BAFANG_G60_750W

// Currently choosen motor for e cells

/************** WHEEL SPEED SENSOR PARAMETERS  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION     4       // Wheel speed sensor cycle number for one wheel rotation
#define WHEEL_SPEED_SLOW_LOOP_COUNT             1       // Wheel speed sensor slow detect counter to get 750ms per function call
#define WHEEL_SPEED_SENSOR_CORRECTION_FACTOR    1       // Wheel speed sensor slow detect correction for a signal after two wheel spin detection 
#define WHEEL_SPEED_SLOW_LOOP_DETECT            false   // Wheel speed sensor slow detect counter flag activation

#endif