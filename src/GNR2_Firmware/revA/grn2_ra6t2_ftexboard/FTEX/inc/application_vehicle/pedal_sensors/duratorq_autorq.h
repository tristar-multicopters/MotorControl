#ifndef __DURATORQ_AUTORQ_H
#define __DURATORQ_AUTORQ_H

// Currently choosen torque sensor for Velec

/***************** PEDAL TORQUE SENSOR PARAMETERS  ******************************/

#define PTS_FILTER_ALPHA                2.27F   // Butterworth alpha coefficient pedal torque sensor filtering
#define PTS_FILTER_BETA                 -0.27F  // Butterworth beta coefficient pedal torque sensor filtering
#define PTS_MAX_PTSVALUE                61000   // Maximum analog value to reach

#define PTS_OFFSET_ADC2PTS              16000   // Offset for ADC to pedal torque sensor linear transformation

/***************** PEDAL TORQUE SENSOR CADENCE PARAMETERS  ******************************/

#define PAS_SLOW_PEDAL_COUNT            0       // Loop wait counter to update the PAS detection function




#endif