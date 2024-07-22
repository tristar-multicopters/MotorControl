/**
  * @file    parameters_conversion.c
  * @brief   This file contains the parameter conversions needed for the MC layer
*/

#include "motor_parameters.h"
#include "board_hardware.h"

// ================================ Private Function Definitions ===================================

void FoldbackParameters_Init(MotorParameters_t * MotorParameters);
void PIDParameters_Init(MotorParameters_t * MotorParameters);
void NTCParameters_Init(MotorParameters_t * MotorParameters);

// ================================ Public Functions ===================================

/*
 * Init parameters that are dependent on motor parameters
 */
void MotorParameters_Init(MotorParameters_t * MotorParameters)
{
    //Init peak current. Peak current is limited by either motor or controller
    if (MotorParameters->ConfigParameters.hPeakCurrentMotorAmps < PEAK_CURRENT_CONTROLLER_amps)
    {
        MotorParameters->ParametersConversion.hPeakCurrentAmps = MotorParameters->ConfigParameters.hPeakCurrentMotorAmps;
    }
    else
    {
        MotorParameters->ParametersConversion.hPeakCurrentAmps = PEAK_CURRENT_CONTROLLER_amps;
    }
    
    //Init nominal peak current
    MotorParameters->ParametersConversion.hNominalPeakCurrent = (int16_t) (MotorParameters->ParametersConversion.hPeakCurrentAmps * 65535 / (2 * MAX_MEASURABLE_CURRENT));
    
    //Init max back EMF Voltage
    MotorParameters->ParametersConversion.hMaxBEMFVoltage = (int16_t) ((MotorParameters->SpeedParameters.hMaxAppliationSpeedRPM * 1.2 * MotorParameters->ConfigParameters.fMotorVotlageConstant * SQRT_2) / (1000u * SQRT_3));
    
    //Init max application speed
    MotorParameters->ParametersConversion.hMaxApplicationSpeedUnit = (MotorParameters->SpeedParameters.hMaxAppliationSpeedRPM * SPEED_UNIT) / _RPM;
    
    //Init torque parameters
    MotorParameters->ParametersConversion.hNominalTorque = (uint16_t) (1.5 * 100 * MotorParameters->ConfigParameters.bPolePairNum * MotorParameters->ConfigParameters.fMotorMagnetFlux * MotorParameters->ParametersConversion.hPeakCurrentAmps);    // Nominal torque to apply to motor in cNm
    MotorParameters->ParametersConversion.hStartingTorque = (uint16_t) (MotorParameters->ParametersConversion.hNominalTorque * MotorParameters->ConfigParameters.fSTTorqueCoef);    // Maximum starting torque to apply to motor in cNm  Only used for Heavy bikes;
    MotorParameters->ParametersConversion.fGainTorqueIqRef = (float) (1 / (100 * 3 * MotorParameters->ConfigParameters.bPolePairNum * MotorParameters->ConfigParameters.fMotorMagnetFlux * MAX_MEASURABLE_CURRENT / (UINT16_MAX))) ;
    
    //Init C1, C3 and C5 parameters
    MotorParameters->ParametersConversion.hC1 = (int16_t) ((((int16_t)F1) * MotorParameters->ConfigParameters.fRS) / (MotorParameters->ConfigParameters.fLS * TF_REGULATION_RATE));
    MotorParameters->ParametersConversion.hC3 = (int16_t) ((((int16_t) F1) * MotorParameters->ParametersConversion.hMaxBEMFVoltage) / (MotorParameters->ConfigParameters.fLS * MAX_MEASURABLE_CURRENT * TF_REGULATION_RATE));
    MotorParameters->ParametersConversion.hC5 = (int16_t) ((((int16_t) F1) * MAX_VOLTAGE) / (MotorParameters->ConfigParameters.fLS * MAX_MEASURABLE_CURRENT * TF_REGULATION_RATE));
    
    //Init foldback, PID and NTC parameters
    FoldbackParameters_Init(MotorParameters);
    PIDParameters_Init(MotorParameters);
    NTCParameters_Init(MotorParameters);
    
    //Init Iq and Id's Kp and Ki
    LookupTableM1IqKp.wXDataFirstValue = MotorParameters->CurrentSpeedPID.currentPIDLutSpeed1;
    LookupTableM1IqKp.hXDataStep = MotorParameters->CurrentSpeedPID.currentPIDLutSpeed2;
    LookupTableM1IqKp.pOutputTable = MotorParameters->CurrentSpeedPID.IqKpVsSpeedTable;
    
    LookupTableM1IqKi.wXDataFirstValue = MotorParameters->CurrentSpeedPID.currentPIDLutSpeed1;
    LookupTableM1IqKi.hXDataStep = MotorParameters->CurrentSpeedPID.currentPIDLutSpeed2;
    LookupTableM1IqKi.pOutputTable = MotorParameters->CurrentSpeedPID.IqKiVsSpeedTable;
    
    LookupTableM1IdKp.wXDataFirstValue = MotorParameters->CurrentSpeedPID.currentPIDLutSpeed1;
    LookupTableM1IdKp.hXDataStep = MotorParameters->CurrentSpeedPID.currentPIDLutSpeed2;
    LookupTableM1IdKp.pOutputTable = MotorParameters->CurrentSpeedPID.IdKpVsSpeedTable;
    
    LookupTableM1IdKi.wXDataFirstValue = MotorParameters->CurrentSpeedPID.currentPIDLutSpeed1;
    LookupTableM1IdKi.hXDataStep = MotorParameters->CurrentSpeedPID.currentPIDLutSpeed2;
    LookupTableM1IdKi.pOutputTable = MotorParameters->CurrentSpeedPID.IdKiVsSpeedTable;

}

// ================================ Private Functions ===================================

/*
 * Init foldback parameters
 */
void FoldbackParameters_Init(MotorParameters_t * MotorParameters)
{
    //Init speed foldback
    MotorParameters->ParametersConversion.FoldbackInitSpeed.bEnableFoldback = true;
    MotorParameters->ParametersConversion.FoldbackInitSpeed.hDefaultOutputLimitHigh = MotorParameters->ParametersConversion.hNominalTorque;
    MotorParameters->ParametersConversion.FoldbackInitSpeed.hDefaultOutputLimitLow = 0;
    MotorParameters->ParametersConversion.FoldbackInitSpeed.hDecreasingEndValue = 10 * MotorParameters->SpeedParameters.hMaxAppliationSpeedRPM;
    MotorParameters->ParametersConversion.FoldbackInitSpeed.hDecreasingRange = 10 * MotorParameters->SpeedParameters.hFoldbackSpeedInterval;
    MotorParameters->ParametersConversion.FoldbackInitSpeed.FoldbackConfig = TRIM;
    
    //Init torque foldback
    MotorParameters->ParametersConversion.FoldbackInitTorque.bEnableFoldback = true;
    MotorParameters->ParametersConversion.FoldbackInitTorque.hDefaultOutputLimitHigh = MotorParameters->ParametersConversion.hStartingTorque;
    MotorParameters->ParametersConversion.FoldbackInitTorque.hDefaultOutputLimitLow = MotorParameters->ParametersConversion.hNominalTorque;
    MotorParameters->ParametersConversion.FoldbackInitTorque.hDecreasingEndValue = 10 * DYNAMICTORQUE_THRESHOLD_SPEED;
    MotorParameters->ParametersConversion.FoldbackInitTorque.hDecreasingRange = 10 * DYNAMICTORQUE_THRESHOLD_SPEED;
    MotorParameters->ParametersConversion.FoldbackInitTorque.FoldbackConfig = SET_THRESHOLD;
    
    //Init heatsink foldback
    MotorParameters->ParametersConversion.FoldbackInitControllerTemp.bEnableFoldback = true;
    MotorParameters->ParametersConversion.FoldbackInitControllerTemp.hDefaultOutputLimitHigh = MotorParameters->ParametersConversion.hNominalTorque;
    MotorParameters->ParametersConversion.FoldbackInitControllerTemp.hDefaultOutputLimitLow = 0;
    MotorParameters->ParametersConversion.FoldbackInitControllerTemp.hDecreasingEndValue = 100 * FOLDBACK_HS_TEMP_END_VALUE;
    MotorParameters->ParametersConversion.FoldbackInitControllerTemp.hDecreasingRange = 100 * FOLDBACK_HS_TEMP_INTERVAL;
    MotorParameters->ParametersConversion.FoldbackInitControllerTemp.FoldbackConfig = TRIM;
    
    //Init motor temp foldback
    MotorParameters->ParametersConversion.FoldbackInitMotorTemp.bEnableFoldback = true;
    MotorParameters->ParametersConversion.FoldbackInitMotorTemp.hDefaultOutputLimitHigh = MotorParameters->ParametersConversion.hNominalTorque;
    MotorParameters->ParametersConversion.FoldbackInitMotorTemp.hDefaultOutputLimitLow = 0;
    MotorParameters->ParametersConversion.FoldbackInitMotorTemp.hDecreasingEndValue = 100 * MotorParameters->TempParameters.hOverTempMotorThresholdC;
    MotorParameters->ParametersConversion.FoldbackInitMotorTemp.hDecreasingRange = 100 * MotorParameters->TempParameters.hFoldbackMotorTempInterval;
    MotorParameters->ParametersConversion.FoldbackInitMotorTemp.FoldbackConfig = TRIM;
}

/*
 * Init PID parameters
 */
void PIDParameters_Init(MotorParameters_t * MotorParameters)
{
    //Init motor control PID
    MotorParameters->ParametersConversion.PIDInitMotorControl.hDefKpGain = FW_KP_GAIN;
    MotorParameters->ParametersConversion.PIDInitMotorControl.hDefKiGain = FW_KI_GAIN;
    MotorParameters->ParametersConversion.PIDInitMotorControl.wUpperIntegralLimit = 0;
    MotorParameters->ParametersConversion.PIDInitMotorControl.wLowerIntegralLimit = (int32_t)(-MotorParameters->ParametersConversion.hNominalPeakCurrent) * (int32_t)FW_KIDIV;
    MotorParameters->ParametersConversion.PIDInitMotorControl.hUpperOutputLimit = 0;
    MotorParameters->ParametersConversion.PIDInitMotorControl.hKpDivisor = FW_KPDIV;
    MotorParameters->ParametersConversion.PIDInitMotorControl.hKiDivisor = FW_KIDIV;
    MotorParameters->ParametersConversion.PIDInitMotorControl.hKpDivisorPOW2 = (uint16_t)FW_KPDIV_LOG;
    MotorParameters->ParametersConversion.PIDInitMotorControl.hKiDivisorPOW2 = (uint16_t)FW_KIDIV_LOG;
    
    //Init Iq PID
    MotorParameters->ParametersConversion.PIDInitIq.hDefKpGain = MotorParameters->TorqueParameters.hPIDTorqueKpDefault;
    MotorParameters->ParametersConversion.PIDInitIq.hDefKiGain = PID_TORQUE_KI_DEFAULT;
    MotorParameters->ParametersConversion.PIDInitIq.wUpperIntegralLimit = MAX_DUTY * TF_KIDIV;
    MotorParameters->ParametersConversion.PIDInitIq.wLowerIntegralLimit = -MAX_DUTY * TF_KIDIV;
    MotorParameters->ParametersConversion.PIDInitIq.hUpperOutputLimit = MAX_DUTY;
    MotorParameters->ParametersConversion.PIDInitIq.hKpDivisor = TF_KPDIV;
    MotorParameters->ParametersConversion.PIDInitIq.hKiDivisor = TF_KIDIV;
    MotorParameters->ParametersConversion.PIDInitIq.hKpDivisorPOW2 = (uint16_t)TF_KPDIV_LOG;
    MotorParameters->ParametersConversion.PIDInitIq.hKiDivisorPOW2 = (uint16_t)TF_KIDIV_LOG;
    
    //Init Id PID
    MotorParameters->ParametersConversion.PIDInitId.hDefKpGain = MotorParameters->FluxParameters.hPIDFluxKPDefault;
    MotorParameters->ParametersConversion.PIDInitId.hDefKiGain = MotorParameters->FluxParameters.hPIDFluxKIDefault;
    MotorParameters->ParametersConversion.PIDInitId.wUpperIntegralLimit = MAX_DUTY * TF_KIDIV;
    MotorParameters->ParametersConversion.PIDInitId.wLowerIntegralLimit = -MAX_DUTY * TF_KIDIV;
    MotorParameters->ParametersConversion.PIDInitId.hUpperOutputLimit = MAX_DUTY;
    MotorParameters->ParametersConversion.PIDInitId.hKpDivisor = TF_KPDIV;
    MotorParameters->ParametersConversion.PIDInitId.hKiDivisor = TF_KIDIV;
    MotorParameters->ParametersConversion.PIDInitId.hKpDivisorPOW2 = (uint16_t)TF_KPDIV_LOG;
    MotorParameters->ParametersConversion.PIDInitId.hKiDivisorPOW2 = (uint16_t)TF_KIDIV_LOG;
    
    //Init speed PID
    MotorParameters->ParametersConversion.PIDInitSpeed.hDefKpGain = MotorParameters->SpeedParameters.hPIDSpeedKpDefault;
    MotorParameters->ParametersConversion.PIDInitSpeed.hDefKiGain = MotorParameters->SpeedParameters.hPIDSpeedKiDefault;
    MotorParameters->ParametersConversion.PIDInitSpeed.wUpperIntegralLimit = SPDCTRL_UPPER_INTEGRAL_LIMIT;
    MotorParameters->ParametersConversion.PIDInitSpeed.wLowerIntegralLimit = 0;
    MotorParameters->ParametersConversion.PIDInitSpeed.hUpperOutputLimit = SPDCTRL_UPPER_INTEGRAL_LIMIT;
    MotorParameters->ParametersConversion.PIDInitSpeed.hKpDivisor = SP_KPDIV;
    MotorParameters->ParametersConversion.PIDInitSpeed.hKiDivisor = MotorParameters->SpeedParameters.hSpKiDiv;
    MotorParameters->ParametersConversion.PIDInitSpeed.hKpDivisorPOW2 = (uint16_t)SP_KPDIV_LOG;
    MotorParameters->ParametersConversion.PIDInitSpeed.hKiDivisorPOW2 = MotorParameters->SpeedParameters.hSpKiDivLog;
    
    //Init speed limit PID
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.hDefKpGain = MotorParameters->SpeedParameters.hPIDSpeedKpDefault;
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.hDefKiGain = MotorParameters->SpeedParameters.hPIDSpeedKiDefault;
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.wUpperIntegralLimit = MotorParameters->ParametersConversion.hStartingTorque * MotorParameters->SpeedParameters.hSpKiDiv;
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.wLowerIntegralLimit = 0;
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.hUpperOutputLimit = MotorParameters->ParametersConversion.hStartingTorque;
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.hKpDivisor = SP_KPDIV;
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.hKiDivisor = MotorParameters->SpeedParameters.hSpKiDiv;
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.hKpDivisorPOW2 = (uint16_t)SP_KPDIV_LOG;
    MotorParameters->ParametersConversion.PIDInitSpeedLimit.hKiDivisorPOW2 = MotorParameters->SpeedParameters.hSpKiDivLog;
    
    //Init back emf observer PID
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.hDefKpGain = PLL_KP_GAIN;
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.hDefKiGain = PLL_KI_GAIN;
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.wUpperIntegralLimit = INT32_MAX;
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.wLowerIntegralLimit = -INT32_MAX;
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.hUpperOutputLimit = INT32_MAX;
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.hKpDivisor = PLL_KPDIV;
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.hKiDivisor = PLL_KIDIV;
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.hKpDivisorPOW2 = (uint16_t)PLL_KPDIV_LOG;
    MotorParameters->ParametersConversion.PIDInitBemfObserverPl.hKiDivisorPOW2 = (uint16_t)PLL_KIDIV_LOG;
}

/*
 * Init NTC parameters
 */
void NTCParameters_Init(MotorParameters_t * MotorParameters)
{
    //Init heatsink NTC
    MotorParameters->ParametersConversion.ControllerNTCInit.bSensorType = REAL_SENSOR;
    MotorParameters->ParametersConversion.ControllerNTCInit.bSensorMixed = false;
    MotorParameters->ParametersConversion.ControllerNTCInit.hOverTempDeactThreshold = OV_TEMP_CONTROLLER_THRESHOLD_C;
    MotorParameters->ParametersConversion.ControllerNTCInit.hOverTempThreshold = OV_TEMP_CONTROLLER_THRESHOLD_C - OV_TEMP_CONTROLLER_HYSTERESIS_C;
    MotorParameters->ParametersConversion.ControllerNTCInit.hFoldbackStartTemp = FOLDBACK_HS_TEMP_END_VALUE - FOLDBACK_HS_TEMP_INTERVAL;
    MotorParameters->ParametersConversion.ControllerNTCInit.hNTCBetaCoef = HEATSINK_NTC_BETA_COEFFICIENT;
    MotorParameters->ParametersConversion.ControllerNTCInit.hNTCResCoef = HEATSINK_NTC_RESISTANCE_COEF;
    MotorParameters->ParametersConversion.ControllerNTCInit.TempRegConv.hChannel = HEATSINK_TEMP_ANALOG_CHANNEL;
    MotorParameters->ParametersConversion.ControllerNTCInit.hLowPassFilterBw = M1_TEMP_SW_FILTER_BW_FACTOR;
    
    //Init motor NTC
    MotorParameters->ParametersConversion.MotorNTCInit.bSensorType = MotorParameters->TempParameters.bMotorTempSensorType;
    MotorParameters->ParametersConversion.MotorNTCInit.bSensorMixed = MOTOR_TEMP_MIXED;
    MotorParameters->ParametersConversion.MotorNTCInit.hOverTempDeactThreshold = MotorParameters->TempParameters.hOverTempMotorThresholdC;
    MotorParameters->ParametersConversion.MotorNTCInit.hOverTempThreshold = MotorParameters->TempParameters.hOverTempMotorThresholdC - MotorParameters->TempParameters.hOverTempMotorHysteresisC;
    MotorParameters->ParametersConversion.MotorNTCInit.hFoldbackStartTemp = (int16_t)(MotorParameters->TempParameters.hOverTempMotorThresholdC - MotorParameters->TempParameters.hFoldbackMotorTempInterval);
    MotorParameters->ParametersConversion.MotorNTCInit.hNTCBetaCoef = MOTOR_NTC_BETA_COEFFICIENT;
    MotorParameters->ParametersConversion.MotorNTCInit.hNTCResCoef = MOTOR_NTC_RESISTANCE_COEF_X_100;
    MotorParameters->ParametersConversion.MotorNTCInit.TempRegConv.hChannel = MOTOR_TEMP_ANALOG_CHANNEL;
    MotorParameters->ParametersConversion.MotorNTCInit.hLowPassFilterBw = M1_TEMP_SW_FILTER_BW_FACTOR;
}
