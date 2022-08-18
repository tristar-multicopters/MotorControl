/**
  * @file    feed_forward_ctrl.c
  * @brief   This file provides firmware functions that implement the Feed Forward
  *          Control component of the Motor Control application.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "feed_forward_ctrl.h"
#include <stddef.h>

#include "mc_type.h"
#include "bus_voltage_sensor.h"
#include "speed_pos_fdbk.h"
#include "speed_torq_ctrl.h"
#include "r_divider_bus_voltage_sensor.h"

extern ResDivVbusSensorHandle_t * pBusSensorM1;


/* Private macros ------------------------------------------------------------*/
#define SEGMNUM (uint8_t)7 /* coeff no. -1 */
#define SATURATION_TO_S16(a)    if ((a) > 32767)                \
  {                               \
    (a) = 32767;                  \
  }                               \
  else if ((a) < -32767)          \
  {                               \
    (a) = -32767;                 \
  }                               \
  else                            \
  {}                              \


void Feedforward_Init(FeedforwardHandle_t * pHandle, BusVoltageSensorHandle_t * pBusSensor, PIDHandle_t * pPIDId,
              PIDHandle_t * pPIDIq)
{

  pHandle->wConstant1D = pHandle->wDefConstant1D;
  pHandle->wConstant1Q = pHandle->wDefConstant1Q;
  pHandle->wConstant2  = pHandle->wDefConstant2;

  pHandle->pBusSensor = pBusSensor;

  pHandle->pPIDd = pPIDId;

  pHandle->pPIDq = pPIDIq;
}


void Feedforward_Clear(FeedforwardHandle_t * pHandle)
{
  pHandle->Vqdff.q = (int16_t)0;
  pHandle->Vqdff.d = (int16_t)0;

  return;
}


void Feedforward_VqdffComputation(FeedforwardHandle_t * pHandle, qd_t Iqdref, SpdTorqCtrlHandle_t * pSpeedTorqCtrl)
{
  int32_t wtemp1, wtemp2;
  int16_t hSpeed_dpp;
  uint16_t hAvBusVoltage_d;
  SpdPosFdbkHandle_t * SpeedSensor;

  SpeedSensor = SpdTorqCtrl_GetSpeedSensor(pSpeedTorqCtrl);
  hSpeed_dpp = SpdPosFdbk_GetElSpeedDpp(SpeedSensor);
  hAvBusVoltage_d = VbusSensor_GetAvBusVoltageDigital(&(pBusSensorM1->Super)) / 2u;

  if (hAvBusVoltage_d != (uint16_t)0)
  {
    /*q-axes ff voltage calculation */
    wtemp1 = (((int32_t)(hSpeed_dpp) * Iqdref.d) / (int32_t)32768);
    wtemp2 = (wtemp1 * pHandle->wConstant1D) / (int32_t)(hAvBusVoltage_d);
    wtemp2 *= (int32_t)2;

    wtemp1 = ((pHandle->wConstant2 * hSpeed_dpp) / (int32_t)hAvBusVoltage_d)
                   * (int32_t)16;

    wtemp2 = wtemp1 + wtemp2 + pHandle->VqdAvPIout.q;

    SATURATION_TO_S16(wtemp2)

    pHandle->Vqdff.q = (int16_t)(wtemp2);

    /* d-axes ff voltage calculation */
    wtemp1 = (((int32_t)(hSpeed_dpp) * Iqdref.q) / (int32_t)32768);
    wtemp2 = (wtemp1 * pHandle->wConstant1Q) / (int32_t)(hAvBusVoltage_d);
    wtemp2 *= (int32_t)2;

    wtemp2 = (int32_t)pHandle->VqdAvPIout.d - wtemp2;

    SATURATION_TO_S16(wtemp2)

    pHandle->Vqdff.d = (int16_t)(wtemp2);
  }
  else
  {
    pHandle->Vqdff.q = (int16_t) 0;
    pHandle->Vqdff.d = (int16_t) 0;
  }
}


qd_t Feedforward_VqdConditioning(FeedforwardHandle_t * pHandle, qd_t Vqd)
{
  int32_t wtemp;
  qd_t lVqd;

  pHandle->VqdPIout = Vqd;

  wtemp = (int32_t)(Vqd.q) + pHandle->Vqdff.q;

  SATURATION_TO_S16(wtemp)

  lVqd.q= (int16_t)wtemp;

  wtemp = (int32_t)(Vqd.d) + pHandle->Vqdff.d;

  SATURATION_TO_S16(wtemp)

  lVqd.d = (int16_t)wtemp;

  return (lVqd);
}


void Feedforward_DataProcess(FeedforwardHandle_t * pHandle)
{
  int32_t wAux;
  int32_t lowPassFilterBW = (int32_t) pHandle->hVqdLowPassFilterBw - (int32_t)1;
  
#ifdef FULL_MISRA_C_COMPLIANCY
  /* Computation of average Vqd as output by PI(D) current controllers, used by
     feed-forward controller algorithm */
  wAux = (int32_t)(pHandle->VqdAvPIout.q) * lowPassFilterBW;
  wAux += pHandle->VqdPIout.q;

  pHandle->VqdAvPIout.q = (int16_t)(wAux /
                                      (int32_t)(pHandle->hVqdLowPassFilterBw));

  wAux = (int32_t)(pHandle->VqdAvPIout.d) * lowPassFilterBW;
  wAux += pHandle->VqdPIout.d;

  pHandle->VqdAvPIout.d = (int16_t)(wAux /
                                      (int32_t)(pHandle->hVqdLowPassFilterBw));
#else
  /* Computation of average Vqd as output by PI(D) current controllers, used by
     feed-forward controller algorithm */
  wAux = (int32_t)(pHandle->VqdAvPIout.q) * lowPassFilterBW;
  wAux += pHandle->VqdPIout.q;
  pHandle->VqdAvPIout.q = (int16_t) (wAux >>
                                        pHandle->hVqdLowPassFilterBwLog);

  wAux = (int32_t)(pHandle->VqdAvPIout.d) * lowPassFilterBW;
  wAux += pHandle->VqdPIout.d;
  pHandle->VqdAvPIout.d = (int16_t)  (wAux >>
                                         pHandle->hVqdLowPassFilterBwLog);

#endif
}


void Feedforward_InitFOCAdditionalMethods(FeedforwardHandle_t * pHandle)
{
  pHandle->VqdAvPIout.q = 0;
  pHandle->VqdAvPIout.d = 0;
  PID_SetIntegralTerm(pHandle->pPIDq, 0);
  PID_SetIntegralTerm(pHandle->pPIDd, 0);
}


void Feedforward_SetFFConstants(FeedforwardHandle_t * pHandle, FeedforwardTuningStruct_t sNewConstants)
{
  pHandle->wConstant1D = sNewConstants.wConst1D;
  pHandle->wConstant1Q = sNewConstants.wConst1Q;
  pHandle->wConstant2  = sNewConstants.wConst2;
}


FeedforwardTuningStruct_t Feedforward_GetFFConstants(FeedforwardHandle_t * pHandle)
{
  FeedforwardTuningStruct_t LocalConstants;

  LocalConstants.wConst1D = pHandle->wConstant1D;
  LocalConstants.wConst1Q = pHandle->wConstant1Q;
  LocalConstants.wConst2 =  pHandle->wConstant2;

  return (LocalConstants);
}


qd_t Feedforward_GetVqdff(const FeedforwardHandle_t * pHandle)
{
  return (pHandle->Vqdff);
}


qd_t Feedforward_GetVqdAvPIout(const FeedforwardHandle_t * pHandle)
{
  return (pHandle->Vqdff);
}

