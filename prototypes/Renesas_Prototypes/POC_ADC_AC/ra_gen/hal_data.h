/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#include "r_adc_b.h"
                      #include "r_adc_api.h"
FSP_HEADER
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer0;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer0_ctrl;
extern const timer_cfg_t g_timer0_cfg;

#ifndef NULL
void NULL(timer_callback_args_t * p_args);
#endif
/** ADC on ADC_B instance. */
                    extern const adc_instance_t g_adc_b;

                    /** Access the ADC_B instance using these structures when calling API functions directly (::p_api is not used). */
                    extern adc_b_instance_ctrl_t g_adc_b_ctrl;
                    extern const adc_cfg_t g_adc_b_cfg;
                    extern const adc_b_scan_cfg_t g_adc_b_scan_cfg;

                    #ifndef AC_ADC
                    void AC_ADC(adc_callback_args_t * p_args);
                    #endif
void hal_entry(void);
void g_hal_init(void);


#include "hal_data.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>



void R_BSP_WarmStart(bsp_warm_start_event_t event);


#define BIT_SHIFT_8  (8u)
#define SIZE_64      (64u)
#define LVL_ERR      (1u)       /* error conditions   */
#define RESET_VALUE  (0x00)

#define VREFADCG_VALUE   0x03
#define SHIFT_BY_ONE     0x01
#define SHIFT_BY_THREE   0x03
#define VREFADCG_ENABLE  0x03
#define ADC_READ_DELAY	 0x01

/* Flag to notify that adc scan is started, so start reading adc */
// bool b_ready_to_read;
 
static uint16_t g_adc_b_data;
extern const adc_b_extended_cfg_t g_adc_b_cfg_extend;
static fsp_err_t adc_scan_start(void);
static fsp_err_t adc_scan_stop(void);
static fsp_err_t adc_start_calibration(void);

/* Read the adc data available */
fsp_err_t adc_read_data(void);

/* close the open adc module  */
static void deinit_adc_module(void);
static void LEDs_Blink(void);
fsp_err_t read_process_start(void);

#define BIT_SHIFT_8  (8u)
#define SIZE_64      (64u)

#define LVL_ERR      (1u)       /* error conditions   */

#define RESET_VALUE             (0x00)



fsp_err_t start_gpt_timer (timer_ctrl_t * const p_timer_ctl);
fsp_err_t init_gpt_timer(timer_ctrl_t * const p_timer_ctl, timer_cfg_t const * const p_timer_cfg);
void deinit_gpt_timer(timer_ctrl_t * const p_timer_ctl);

FSP_FOOTER
#endif /* HAL_DATA_H_ */
