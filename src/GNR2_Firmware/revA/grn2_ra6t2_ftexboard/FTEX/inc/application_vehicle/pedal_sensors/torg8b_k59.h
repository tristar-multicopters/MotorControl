#ifndef __TORG8B_K59_H
#define __TORG8B_K59_H

// Currently choosen torque sensor for Nidec

/***************** PEDAL TORQUE SENSOR PARAMETERS  ******************************/

#define PTS_FILTER_ALPHA                2.27F   // Butterworth alpha coefficient pedal torque sensor filtering
#define PTS_FILTER_BETA                 -0.27F  // Butterworth beta coefficient pedal torque sensor filtering
#define PTS_MAX_PTSVALUE                40000   // Maximum analog value to reach

#define PTS_OFFSET_ADC2PTS              12000   // Offset for ADC to pedal torque sensor linear transformation

/***************** PEDAL TORQUE SENSOR CADENCE PARAMETERS  ******************************/

#define PAS_SLOW_PEDAL_COUNT            1       // Loop wait counter to update the PAS detection function

#endif