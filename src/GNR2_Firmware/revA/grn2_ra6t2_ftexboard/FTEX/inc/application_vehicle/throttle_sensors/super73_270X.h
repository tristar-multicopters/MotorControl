#ifndef __SUPER73_270X_H
#define __SUPER73_270X_H

// Currently choosen throttle for Super 73

/***************** THROTTLE PARAMETERS  ******************************/

#define THROTTLE_FILTER_ALPHA               2.27F   // Butterworth alpha coefficient for throttle filtering
#define THROTTLE_FILTER_BETA                -0.27F  // Butterworth beta coefficient for throttle filtering

#define THROTTLE_OFFSET_ADC2THROTTLE        10800   // Offset for ADC to throttle linear transformation
#define THROTTLE_MAX_ADC2THROTTLE           53000   // Maximum value reachable by the throttle adc value

#endif