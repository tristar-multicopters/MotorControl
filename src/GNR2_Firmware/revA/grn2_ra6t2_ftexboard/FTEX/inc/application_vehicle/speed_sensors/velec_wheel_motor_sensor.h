#ifndef __VELEC_WHEEL_SENSOR_H
#define __VELEC_WHEEL_SENSOR_H

// Currently choosen motor speed signals for Velec

/************** WHEEL SPEED SENSOR PARAMETERS  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION     12       // Wheel speed sensor cycle number for one wheel rotation
#define WHEEL_SPEED_SLOW_LOOP_COUNT             3       // Wheel speed sensor slow detect counter to get 750ms per function call
#define WHEEL_SPEED_SENSOR_CORRECTION_FACTOR    2       // Wheel speed sensor slow detect correction for a signal after two wheel spin detection
#define WHEEL_SPEED_SLOW_LOOP_DETECT            true    // Wheel speed sensor slow detect counter flag activation

#endif