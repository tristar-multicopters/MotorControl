/**
  * @file    ntc_temperature_sensor.c
  * @brief   This file provides firmware functions that implement the features
  *          of the Temperature Sensor component of the Motor Control application.
*/

#include "ntc_temperature_sensor.h"
#include "drive_parameters.h"
#include "motor_signal_processing.h"


/* Defines --------------------------------------------------------------------*/

#define TIMER_STEP          1       /* Timer value increases by 1 */
#define INIT_IGNORE_TIMER   1000    /* Timer for ignoring first NTC values in initialization */
#define DISC_DIGITAL        60000   /* NTC digital value at which temp sensor disconnects at */

/* the minimum acceptable value for NTC - any lower value means 
sensor is disconnected or temeprature is very low
outuput voltage of 10K (resisotr on PCB) || 200K (NTC at -30degrees) on 3.3V = 0.157
So, 200 assumed with a safeguard as the minimum acceptable digital value for NTC, multiplied by 16
Link to calculations: https://docs.google.com/spreadsheets/d/1Hm1R5MGJHDdYDkZ9kOQ2_IeC9xUJphte0QKUo2zZfmU/edit#gid=155823673 */
#define MIN_NTC_FREEZE      3200

/* Number of NTCs in NTC struct. Should be increased if another NTC is added */
#define NTC_NUMBER          2
// ========================================================================= //

/* Global Variables -----------------------------------------------------------*/

/* Struct that holds all of the NTCs */
NTCTempSensorHandle_t NTCTempSensors[NTC_NUMBER];

// ========================================================================= //

/* Private function prototypes -----------------------------------------------*/
NTCTempFaultStates_t NTC_SetFaultState(uint8_t chooseNTC);


// ========================================================================= //

/**
  * @brief Returns fault when temperature exceeds the over voltage protection threshold
  *
  *  @r Fault status : Updated internal fault status
  */
NTCTempFaultStates_t NTC_SetFaultState(uint8_t chooseNTC)
{
    NTCTempFaultStates_t hFault;

    if (NTCTempSensors[chooseNTC].hAvTempCelcius >= NTCTempSensors[chooseNTC].hOverTempThreshold)
    {
        hFault = NTC_OT;
    }
    else if (NTCTempSensors[chooseNTC].hAvTempCelcius >= NTCTempSensors[chooseNTC].hFoldbackStartTemp)
    {
        hFault = NTC_FOLDBACK;
    }
    else if (NTCTempSensors[chooseNTC].hAvTempDigital >= DISC_DIGITAL) //disconnection when temp is at lowest value in NTC table
    {
        hFault = NTC_DISC;
    }
    else if (NTCTempSensors[chooseNTC].hAvTempDigital <= MIN_NTC_FREEZE)
    {
        hFault = NTC_FREEZE;
    }
    else if (NTCTempSensors[chooseNTC].hAvTempCelcius < NTCTempSensors[chooseNTC].hOverTempDeactThreshold)
    {
        hFault = NTC_NO_ERRORS;
    }
    else
    {
        hFault = NTCTempSensors[chooseNTC].hFaultState;
    }
    return hFault;
}

/* Functions ---------------------------------------------------- */



void NTCTempSensor_Init(uint8_t chooseNTC, NTCTempSensorHandle_t NTCInit, uint16_t defaultTemp)
{
    NTCTempSensors[chooseNTC].bSensorType = NTCInit.bSensorType;
    NTCTempSensors[chooseNTC].bSensorMixed = NTCInit.bSensorMixed;
    NTCTempSensors[chooseNTC].hOverTempThreshold = NTCInit.hOverTempDeactThreshold;
    NTCTempSensors[chooseNTC].hOverTempDeactThreshold = NTCInit.hOverTempThreshold;
    NTCTempSensors[chooseNTC].hFoldbackStartTemp = NTCInit.hFoldbackStartTemp;
    NTCTempSensors[chooseNTC].hNTCBetaCoef = NTCInit.hNTCBetaCoef;
    NTCTempSensors[chooseNTC].hNTCResCoef = NTCInit.hNTCResCoef;
    NTCTempSensors[chooseNTC].TempRegConv.hChannel = NTCInit.TempRegConv.hChannel;
    NTCTempSensors[chooseNTC].hLowPassFilterBw = NTCInit.hLowPassFilterBw;
    
    if (NTCTempSensors[chooseNTC].bSensorType == REAL_SENSOR)
    {
        NTCTempSensors[chooseNTC].bConvHandle = RegConvMng_RegisterRegConv(&NTCTempSensors[chooseNTC].TempRegConv);  // Need to be register with RegularConvManager
        NTCTempSensors[chooseNTC].hTimer = INIT_IGNORE_TIMER;
        NTCTempSensor_Clear(chooseNTC, defaultTemp);
    }
    else  // NO_SENSOR
    {
        NTCTempSensors[chooseNTC].hFaultState = NTC_NO_ERRORS;
        NTCTempSensors[chooseNTC].hAvTempDigital = NTCTempSensors[chooseNTC].hExpectedTempDigital;
    }
}


void NTCTempSensor_Clear(uint8_t chooseNTC, uint16_t defaultTemp)
{
    NTCTempSensors[chooseNTC].hAvTempDigital = defaultTemp;
}

uint16_t NTCTempSensor_CalcAvTemp(uint8_t chooseNTC)
{
    uint32_t wtemp;  // temporary 32 bit variable for calculation
    uint16_t hAux;   // temporary 16 bit variable for calculation
    if (NTCTempSensors[chooseNTC].bSensorType == REAL_SENSOR)  // Checks if the sensor is real or virtual
    {
        //does have the current motor mixed signal?
        if ((isMotorMixedSignal() == true) && (getMixedSignalRegConvIndex() == NTCTempSensors[chooseNTC].bConvHandle))
        { 
            hAux = getExtractedMotorTemperature();
        }
        else
        {
            hAux = RegConvMng_ReadConv(NTCTempSensors[chooseNTC].bConvHandle);   // Reads raw value of converted ADC value.
        }
        
        // Checks for max reading, if yes, no point of averaging
        // Performs first order averaging:
        // new_average = (instantenous_measurment + (previous_average * number_of_smaples - 1)) / number_of_smaples
        wtemp =  (uint32_t)(NTCTempSensors[chooseNTC].hLowPassFilterBw) - 1u;
        wtemp *= (uint32_t) (NTCTempSensors[chooseNTC].hAvTempDigital);
        wtemp += hAux;
        wtemp /= (uint32_t)(NTCTempSensors[chooseNTC].hLowPassFilterBw);
        NTCTempSensors[chooseNTC].hAvTempDigital = (uint16_t) wtemp;
    }
    else  // NO_SENSOR
    {
        NTCTempSensors[chooseNTC].hFaultState = NTC_NO_ERRORS;
    }
    
    int32_t wTemp;  // temporary 32 bit variable for calculation
    if((NTCTempSensors[chooseNTC].bSensorType == REAL_SENSOR) && (chooseNTC == NTC_MOTOR)) // Checks for sensor type and sensor channel
    {
        wTemp = NTCTempSensor_CalcMotorTemp(NTCTempSensors[NTC_MOTOR].hAvTempDigital);
    }
    else if ((NTCTempSensors[chooseNTC].bSensorType == REAL_SENSOR) && (chooseNTC == NTC_CONTROLLER))  // Checks for sensor type and sensor channel
    {
        wTemp = NTCTempSensor_CalcControllerTemp (NTCTempSensors[NTC_CONTROLLER].hAvTempDigital);
    }
    else
    {
        wTemp = NTCTempSensors[chooseNTC].hExpectedTempCelcius;
    }
    
    NTCTempSensors[chooseNTC].hAvTempCelcius = (int16_t) wTemp;
    
    NTCTempSensors[chooseNTC].hFaultState = NTC_SetFaultState(chooseNTC);  // Retain state
    
    //ignore the first few values after initialization to avoid triggering the error at the beginning
    if (NTCTempSensors[chooseNTC].hTimer > 0)
    {
        NTCTempSensors[chooseNTC].hTimer -= TIMER_STEP;
        NTCTempSensors[chooseNTC].hFaultState = NTC_NO_ERRORS;
    }
    
    return NTCTempSensors[chooseNTC].hFaultState;
}


uint16_t NTCTempSensor_GetAvTempDigital(uint8_t chooseNTC)
{
    return NTCTempSensors[chooseNTC].hAvTempDigital;
}

int16_t NTCTempSensor_GetAvTempCelcius(uint8_t chooseNTC)
{
    return NTCTempSensors[chooseNTC].hAvTempCelcius;
}


uint16_t NTCTempSensor_GetFaultState(uint8_t chooseNTC)
{
    return NTCTempSensors[chooseNTC].hFaultState;
}

//Calculate the motor temperature from the input ADC data.
uint16_t NTCTempSensor_CalcMotorTemp(int32_t wInputdata)
{
    double wtemp = 0;

    // Convert ADC input data to voltage
    wtemp = (double)wInputdata / ADC_BYTE_TO_TICS;
    wtemp *= ADC_REFERENCE_VOLTAGE;
    wtemp /= ADC_MAXIMUM_VALUE;

    // Calculate the resistance of the NTC thermistor
    wtemp = (MOTOR_NTC_PULLUP_RESISTOR * wtemp) / (ADC_REFERENCE_VOLTAGE - wtemp);
    wtemp -= MOTOR_NTC_SERIES_PULLDOWN_RESISTOR;

    // Apply Beta coefficient method to calculate temperature
    wtemp *= (NTCTempSensors[NTC_MOTOR].hNTCResCoef  / 100);
    wtemp = log(wtemp);
    wtemp = NTCTempSensors[NTC_MOTOR].hNTCBetaCoef / wtemp;

    // Convert temperature from Kelvin to Celsius
    wtemp -= CELSIUS_TO_KELVIN;

    // Return the calculated temperature as an unsigned 16-bit integer
    return (uint16_t)wtemp;
}

//Calculate the heat sink temperature from the input ADC data.
uint16_t NTCTempSensor_CalcControllerTemp(int32_t wInputdata)
{
    double wtemp = 0;

    // Convert ADC input data to voltage
    wtemp = (double)wInputdata / ADC_BYTE_TO_TICS;

    // Calculate the resistance of the NTC thermistor
    wtemp = (HEATSINK_NTC_PULLDOWN_RESISTOR * ADC_MAXIMUM_VALUE) / wtemp;
    wtemp -= HEATSINK_NTC_PULLDOWN_RESISTOR;

    // Apply Beta coefficient method to calculate temperature
    wtemp *= NTCTempSensors[NTC_CONTROLLER].hNTCResCoef;
    wtemp = log(wtemp);
    wtemp = NTCTempSensors[NTC_CONTROLLER].hNTCBetaCoef / wtemp;

    // Convert temperature from Kelvin to Celsius
    wtemp -= CELSIUS_TO_KELVIN;

    // Adjust temperature for NTC drift characteristics
    wtemp = wtemp - (wtemp * HEATSINK_NTC_DRIFT_SLOPE - HEATSINK_NTC_DRIFT_INTERCEPT);

    // Return the calculated temperature as an unsigned 16-bit integer
    return (uint16_t)wtemp;
}

//Get sensor type (REAL_SENSOR or NO_SENSOR)
uint8_t NTCTempSensor_GetSensorType(uint8_t chooseNTC)
{
    return NTCTempSensors[chooseNTC].bSensorType;
}

//Set sensor type (REAL_SENSOR or NO_SENSOR)
void NTCTempSensor_SetSensorType(uint8_t chooseNTC, SensorType_t sensorType)
{
    NTCTempSensors[chooseNTC].bSensorType = sensorType;
}

//Set sensor beta coefficient
void NTCTempSensor_SetBetaCoef(uint8_t chooseNTC, uint16_t betaCoef)
{
    NTCTempSensors[chooseNTC].hNTCBetaCoef = betaCoef;
}

//Set sensor resistance coefficient
void NTCTempSensor_SetResistanceCoef(uint8_t chooseNTC, float resCoef)
{
    NTCTempSensors[chooseNTC].hNTCResCoef = resCoef;
}

