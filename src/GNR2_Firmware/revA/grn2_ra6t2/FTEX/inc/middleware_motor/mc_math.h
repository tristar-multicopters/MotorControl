/**
  * @file    mc_math.h
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control application
  *
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_MATH_H
#define MC_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"


#define SQRT_2  1.4142
#define SQRT_3  1.732

/**
  * @brief  Macro to compute logarithm of two
  */
#define LOG2(x) \
  ((x) == 65535 ? 16 : \
   ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 15 : \
    ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 14 : \
     ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2 ? 13 : \
      ((x) == 2*2*2*2*2*2*2*2*2*2*2*2 ? 12 : \
       ((x) == 2*2*2*2*2*2*2*2*2*2*2 ? 11 : \
        ((x) == 2*2*2*2*2*2*2*2*2*2 ? 10 : \
         ((x) == 2*2*2*2*2*2*2*2*2 ? 9 : \
          ((x) == 2*2*2*2*2*2*2*2 ? 8 : \
           ((x) == 2*2*2*2*2*2*2 ? 7 : \
            ((x) == 2*2*2*2*2*2 ? 6 : \
             ((x) == 2*2*2*2*2 ? 5 : \
              ((x) == 2*2*2*2 ? 4 : \
               ((x) == 2*2*2 ? 3 : \
                ((x) == 2*2 ? 2 : \
                 ((x) == 2 ? 1 : \
                  ((x) == 1 ? 0 : -1)))))))))))))))))


/**
  * @brief  Trigonometrical functions type definition
  */
typedef struct
{
  int16_t hCos;
  int16_t hSin;
} TrigComponents_t;

/**
  * @brief  This function transforms stator currents Ia and qIb (which are
  *         directed along axes each displaced by 120 degrees) into currents
  *         Ialpha and Ibeta in a stationary qd reference frame.
  *                               Ialpha = Ia
  *                       Ibeta = -(2*Ib+Ia)/sqrt(3)
  * @param  Curr_Input: stator current Ia and Ib in ab_t format
  * @retval Stator current Ialpha and Ibeta in AlphaBeta_t format
  */
AlphaBeta_t MCMath_Clarke( ab_t Input );

/**
  * @brief  This function transforms stator values alpha and beta, which
  *         belong to a stationary qd reference frame, to a rotor flux
  *         synchronous reference frame (properly oriented), so as Iq and Id.
  *                   Id= Ialpha *sin(theta)+qIbeta *cos(Theta)
  *                   Iq=qIalpha *cos(Theta)-qIbeta *sin(Theta)
  * @param  Curr_Input: stator values alpha and beta in AlphaBeta_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator current q and d in qd_t format
  */
qd_t MCMath_Park( AlphaBeta_t Input, int16_t Theta );

/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to
  *         a rotor flux synchronous rotating frame, to a stationary reference
  *         frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Curr_Input: stator voltage Vq and Vd in qd_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator values alpha and beta in AlphaBeta_t format
  */
AlphaBeta_t MCMath_RevPark( qd_t Input, int16_t Theta );

/**
  * @brief  This function returns cosine and sine functions of the angle fed in
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval TrigComponents_t Cos(angle) and Sin(angle) in TrigComponents_t format
  */
TrigComponents_t MCMath_TrigFunctions( int16_t hAngle );

/**
  * @brief  It calculates the square root of a non-negative s32. It returns 0
  *         for negative s32.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
int32_t MCMath_Sqrt( int32_t wInput );

/**
  * @brief  It executes CORDIC algorithm for rotor position extraction from B-emf
  *         alpha and beta
  * @param  wBemfalfaEst estimated Bemf alpha on the stator reference frame
  *         wBemfbetaEst estimated Bemf beta on the stator reference frame
  * @retval int16_t rotor electrical angle (s16degrees)
  */
int16_t MCMath_PhaseComputation( int32_t wBemfalfaEst, int32_t wBemfbetaEst );

/**
  * @brief  This function codify a floting point number into the relative
  *         32bit integer.
  * @param  float Floting point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
uint32_t MCMath_FloatToIntBit( float x );

/**
  * @brief  This function calculate the amplitude of a signal from its vector components
  * @param  Xvector X vector component value
  * @param  Yvector Y vector component value
  * @retval Amplitude of the signal
  */
int16_t MCMath_AmplitudeFromVectors( int16_t Xvector, int16_t Yvector );


#endif /* MC_MATH_H*/

