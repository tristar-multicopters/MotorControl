/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS, STM32G4xx
  */
PWMC_ICS_Handle_t PWM_Handle_M1 = {
  {
    .pFctGetPhaseCurrents              = &ICS_GetPhaseCurrentsInverted,
    .pFctSwitchOffPwm                  = &ICS_SwitchOffPWM,
    .pFctSwitchOnPwm                   = &ICS_SwitchOnPWM,
    .pFctCurrReadingCalib              = &ICS_CurrentReadingPolarizationInverted,
    .pFctTurnOnLowSides                = &ICS_TurnOnLowSides,
    .pFctSetADCSampPointSectX          = &ICS_WriteTIMRegisters,
    .pFctIsOverCurrentOccurred         = &ICS_IsOverCurrentOccurred,
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = MC_NULL,
    .pFctRLDetectionModeDisable        = MC_NULL,
    .pFctRLDetectionModeSetDuty        = MC_NULL,
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
    .Sector = 0,
    .CntPhA = 0,
    .CntPhB = 0,
    .CntPhC = 0,
    .SWerror = 0,
    .TurnOnLowSidesAction = false,
    .OffCalibrWaitTimeCounter = 0,
    .Motor = M1,
    .RLDetectionMode = false,
    .Ia = 0,
    .Ib = 0,
    .Ic = 0,
    .DTTest = 0,
    .DTCompCnt = DTCOMPCNT,
    .PWMperiod          = PWM_PERIOD_CYCLES,
    .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),
    .Ton                 = TON,
    .Toff                = TOFF

  },
  .PhaseAOffset = 0,
  .PhaseBOffset = 0,
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,
  .PolarizationCounter = 0,
  .OverCurrentFlag = false,
  .OverVoltageFlag = false,
  .BrakeActionLock = false,

  .pParams_str = &ICS_ParamsM1
};