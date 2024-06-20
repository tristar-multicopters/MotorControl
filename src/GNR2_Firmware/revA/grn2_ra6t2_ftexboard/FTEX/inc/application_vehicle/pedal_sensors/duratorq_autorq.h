#ifndef __DURATORQ_AUTORQ_H
#define __DURATORQ_AUTORQ_H

// Currently choosen torque sensor for Velec
//https://www.autorq.com/duratorq-torque-sensor-ebikes

/***************** PEDAL TORQUE SENSOR PARAMETERS  ******************************/

#define PTS_FILTER_ALPHA                2.27F   // Butterworth alpha coefficient pedal torque sensor filtering
#define PTS_FILTER_BETA                 -0.27F  // Butterworth beta coefficient pedal torque sensor filtering
#define PTS_MAX_PTSVALUE                61000   // Maximum analog value to reach

#define PTS_OFFSET_ADC2PTS              16000   // Offset for ADC to pedal torque sensor linear transformation

/***************** PEDAL ASSIST CADENCE PARAMETERS  ******************************/

#define PAS_NB_MAGNETS_PER_TURN                  36       //Number of magnets / pulses per rev / Poles


#endif