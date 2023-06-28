#ifndef __WUXING_130X_H
#define __WUXING_130X_H

// Chosen throttle for Quietkat

/***************** THROTTLE PARAMETERS  ******************************/

#define THROTTLE_FILTER_ALPHA               2.27F   // Butterworth alpha coefficient for throttle filtering
#define THROTTLE_FILTER_BETA                -0.27F  // Butterworth beta coefficient for throttle filtering

#define THROTTLE_OFFSET_ADC2THROTTLE        12500   // Offset for ADC to throttle linear transformation
#define THROTTLE_MAX_ADC2THROTTLE           56000   // Maximum value reachable by the throttle adc value 

#endif