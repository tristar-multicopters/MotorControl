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
NTCTempFaultStates_t NTC_SetFaultState(NTCTempTypes_t chosenNTC);


// ========================================================================= //

/**
  * @brief Returns fault when temperature exceeds the over voltage protection threshold
  *
  *  @r Fault status : Updated internal fault status
  */
NTCTempFaultStates_t NTC_SetFaultState(NTCTempTypes_t chosenNTC)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    NTCTempFaultStates_t hFault;

    if (NTCTempSensors[chosenNTC].hAvTempCelcius >= NTCTempSensors[chosenNTC].hOverTempThreshold)
    {
        hFault = NTC_OT;
    }
    else if (NTCTempSensors[chosenNTC].hAvTempCelcius >= NTCTempSensors[chosenNTC].hFoldbackStartTemp)
    {
        hFault = NTC_FOLDBACK;
    }
    else if (NTCTempSensors[chosenNTC].hAvTempDigital >= DISC_DIGITAL) //disconnection when temp is at lowest value in NTC table
    {
        hFault = NTC_DISC;
    }
    else if (NTCTempSensors[chosenNTC].hAvTempDigital <= MIN_NTC_FREEZE)
    {
        hFault = NTC_FREEZE;
    }
    else if (NTCTempSensors[chosenNTC].hAvTempCelcius < NTCTempSensors[chosenNTC].hOverTempDeactThreshold)
    {
        hFault = NTC_NO_ERRORS;
    }
    else
    {
        hFault = NTCTempSensors[chosenNTC].hFaultState;
    }
    return hFault;
}

/* Functions ---------------------------------------------------- */



void NTCTempSensor_Init(NTCTempTypes_t chosenNTC, NTCTempSensorHandle_t NTCInit, uint16_t defaultTemp)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    NTCTempSensors[chosenNTC].bSensorType = NTCInit.bSensorType;
    NTCTempSensors[chosenNTC].bSensorMixed = NTCInit.bSensorMixed;
    NTCTempSensors[chosenNTC].hOverTempThreshold = NTCInit.hOverTempDeactThreshold;
    NTCTempSensors[chosenNTC].hOverTempDeactThreshold = NTCInit.hOverTempThreshold;
    NTCTempSensors[chosenNTC].hFoldbackStartTemp = NTCInit.hFoldbackStartTemp;
    NTCTempSensors[chosenNTC].hNTCBetaCoef = NTCInit.hNTCBetaCoef;
    NTCTempSensors[chosenNTC].hNTCResCoef = NTCInit.hNTCResCoef;
    NTCTempSensors[chosenNTC].TempRegConv.hChannel = NTCInit.TempRegConv.hChannel;
    NTCTempSensors[chosenNTC].hLowPassFilterBw = NTCInit.hLowPassFilterBw;
    
    if (NTCTempSensors[chosenNTC].bSensorType == REAL_SENSOR)
    {
        NTCTempSensors[chosenNTC].bConvHandle = RegConvMng_RegisterRegConv(&NTCTempSensors[chosenNTC].TempRegConv);  // Need to be register with RegularConvManager
        NTCTempSensors[chosenNTC].hTimer = INIT_IGNORE_TIMER;
        NTCTempSensor_Clear(chosenNTC, defaultTemp);
    }
    else  // NO_SENSOR
    {
        NTCTempSensors[chosenNTC].hFaultState = NTC_NO_ERRORS;
        NTCTempSensors[chosenNTC].hAvTempDigital = NTCTempSensors[chosenNTC].hExpectedTempDigital;
    }
}


void NTCTempSensor_Clear(NTCTempTypes_t chosenNTC, uint16_t defaultTemp)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    NTCTempSensors[chosenNTC].hAvTempDigital = defaultTemp;
}

uint16_t NTCTempSensor_CalcAvTemp(NTCTempTypes_t chosenNTC)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    uint32_t wtemp;  // temporary 32 bit variable for calculation
    uint16_t hAux;   // temporary 16 bit variable for calculation
    if (NTCTempSensors[chosenNTC].bSensorType == REAL_SENSOR)  // Checks if the sensor is real or virtual
    {
        //does have the current motor mixed signal?
        if ((isMotorMixedSignal() == true) && (getMixedSignalRegConvIndex() == NTCTempSensors[chosenNTC].bConvHandle))
        { 
            hAux = getExtractedMotorTemperature();
        }
        else
        {
            hAux = RegConvMng_ReadConv(NTCTempSensors[chosenNTC].bConvHandle);   // Reads raw value of converted ADC value.
        }
        
        // Checks for max reading, if yes, no point of averaging
        // Performs first order averaging:
        // new_average = (instantenous_measurment + (previous_average * number_of_smaples - 1)) / number_of_smaples
        wtemp =  (uint32_t)(NTCTempSensors[chosenNTC].hLowPassFilterBw) - 1u;
        wtemp *= (uint32_t) (NTCTempSensors[chosenNTC].hAvTempDigital);
        wtemp += hAux;
        wtemp /= (uint32_t)(NTCTempSensors[chosenNTC].hLowPassFilterBw);
        NTCTempSensors[chosenNTC].hAvTempDigital = (uint16_t) wtemp;
    }
    else  // NO_SENSOR
    {
        NTCTempSensors[chosenNTC].hFaultState = NTC_NO_ERRORS;
    }
    
    int32_t wTemp;  // temporary 32 bit variable for calculation
    if((NTCTempSensors[chosenNTC].bSensorType == REAL_SENSOR) && (chosenNTC == NTC_MOTOR)) // Checks for sensor type and sensor channel
    {
        wTemp = NTCTempSensor_CalcMotorTemp(NTCTempSensors[NTC_MOTOR].hAvTempDigital);
    }
    else if ((NTCTempSensors[chosenNTC].bSensorType == REAL_SENSOR) && (chosenNTC == NTC_CONTROLLER))  // Checks for sensor type and sensor channel
    {
        wTemp = NTCTempSensor_CalcControllerTemp (NTCTempSensors[NTC_CONTROLLER].hAvTempDigital);
    }
    else
    {
        wTemp = NTCTempSensors[chosenNTC].hExpectedTempCelcius;
    }
    
    NTCTempSensors[chosenNTC].hAvTempCelcius = (int16_t) wTemp;
    
    NTCTempSensors[chosenNTC].hFaultState = NTC_SetFaultState(chosenNTC);  // Retain state
    
    //ignore the first few values after initialization to avoid triggering the error at the beginning
    if (NTCTempSensors[chosenNTC].hTimer > 0)
    {
        NTCTempSensors[chosenNTC].hTimer -= TIMER_STEP;
        NTCTempSensors[chosenNTC].hFaultState = NTC_NO_ERRORS;
    }
    
    return NTCTempSensors[chosenNTC].hFaultState;
}


uint16_t NTCTempSensor_GetAvTempDigital(NTCTempTypes_t chosenNTC)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    return NTCTempSensors[chosenNTC].hAvTempDigital;
}

int16_t NTCTempSensor_GetAvTempCelcius(NTCTempTypes_t chosenNTC)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    return NTCTempSensors[chosenNTC].hAvTempCelcius;
}


uint16_t NTCTempSensor_GetFaultState(NTCTempTypes_t chosenNTC)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    return NTCTempSensors[chosenNTC].hFaultState;
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
uint8_t NTCTempSensor_GetSensorType(NTCTempTypes_t chosenNTC)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    return NTCTempSensors[chosenNTC].bSensorType;
}

//Set sensor type (REAL_SENSOR or NO_SENSOR)
void NTCTempSensor_SetSensorType(NTCTempTypes_t chosenNTC, SensorType_t sensorType)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    NTCTempSensors[chosenNTC].bSensorType = sensorType;
}

//Set sensor beta coefficient
void NTCTempSensor_SetBetaCoef(NTCTempTypes_t chosenNTC, uint16_t betaCoef)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    NTCTempSensors[chosenNTC].hNTCBetaCoef = betaCoef;
}

//Set sensor resistance coefficient
void NTCTempSensor_SetResistanceCoef(NTCTempTypes_t chosenNTC, float resCoef)
{
    ASSERT(chosenNTC < NTC_NUMBER);
    NTCTempSensors[chosenNTC].hNTCResCoef = resCoef;
}

