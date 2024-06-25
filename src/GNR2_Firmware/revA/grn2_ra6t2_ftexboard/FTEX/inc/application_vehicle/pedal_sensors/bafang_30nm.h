#ifndef __BAFANG_30NM
#define __BAFANG_30NM

// Currently choosen torque sensor for e cells

/***************** PEDAL TORQUE SENSOR PARAMETERS  ******************************/

#define PTS_FILTER_ALPHA                2.27F   // Butterworth alpha coefficient pedal torque sensor filtering
#define PTS_FILTER_BETA                 -0.27F  // Butterworth beta coefficient pedal torque sensor filtering
#define PTS_MAX_PTSVALUE                42000   // Maximum analog value to reach

#define PTS_OFFSET_ADC2PTS              20000   // Offset for ADC to pedal torque sensor linear transformation

/***************** PEDAL ASSIST CADENCE PARAMETERS  ******************************/

#define PAS_NB_MAGNETS_PER_TURN                  0       //Number of magnets / pulses per turn / Poles

#endif