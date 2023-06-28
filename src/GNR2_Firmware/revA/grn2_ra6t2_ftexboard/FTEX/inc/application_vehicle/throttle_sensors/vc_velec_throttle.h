#ifndef __VELEC_THROTTLE_H
#define __VELEC_THROTTLE_H

// Currently choosen throttle for Velec A2 and CITI

/***************** THROTTLE PARAMETERS  ******************************/

#define THROTTLE_FILTER_ALPHA               2.27F   // Butterworth alpha coefficient for throttle filtering
#define THROTTLE_FILTER_BETA                -0.27F  // Butterworth beta coefficient for throttle filtering

#define THROTTLE_OFFSET_ADC2THROTTLE        12500   // Offset for ADC to throttle linear transformation
#define THROTTLE_MAX_ADC2THROTTLE           53200   // Maximum value reachable by the throttle adc value 

#endif