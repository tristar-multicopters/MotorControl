#ifndef __VELEC_THROTTLE_R48_H
#define __VELEC_THROTTLE_R48_H

// Currently choosen throttle for Velec R48

/***************** THROTTLE PARAMETERS  ******************************/

#define THROTTLE_FILTER_ALPHA               2.27F   // Butterworth alpha coefficient for throttle filtering
#define THROTTLE_FILTER_BETA                -0.27F  // Butterworth beta coefficient for throttle filtering

#define THROTTLE_OFFSET_ADC2THROTTLE        11800   // Offset for ADC to throttle linear transformation
#define THROTTLE_MAX_ADC2THROTTLE           55800   // Maximum value reachable by the throttle adc value

#endif