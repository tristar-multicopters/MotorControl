/**
  ******************************************************************************
  * @file    vc_config.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module declares global structures used by other vehicule control modules
  *
	******************************************************************************
*/

#include "vc_config.h"

/**
* throttle sensor parameters
*/
Throttle_Handle_t ThrottleSensor =
{
	.hChannelConfig =
	{
		.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
		.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
		.gain = NRF_SAADC_GAIN1_6,
		.reference = NRF_SAADC_REFERENCE_INTERNAL,
		.acq_time = NRF_SAADC_ACQTIME_10US,
		.mode = NRF_SAADC_MODE_SINGLE_ENDED,
		.burst = NRF_SAADC_BURST_DISABLED,
		.pin_p = NRF_SAADC_INPUT_AIN5,
		.pin_n = NRF_SAADC_INPUT_DISABLED,
	},
	.hParam =
	{
		.hLowPassFilterBW  = 8,
		.hOffset = 10710,//15700,//10710,
		.hSlope = -0.7,
		.hStep = 100,
	}
};

