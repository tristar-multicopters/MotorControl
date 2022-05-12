/* generated HAL source file - do not edit */
#include "hal_data.h"

dac_instance_ctrl_t g_dac1_ctrl;
const dac_extended_cfg_t g_dac1_ext_cfg =
{
    .enable_charge_pump   = 0,
    .data_format         = DAC_DATA_FORMAT_FLUSH_RIGHT,
    .output_amplifier_enabled = 0,
    .internal_output_enabled = false,
};
const dac_cfg_t g_dac1_cfg =
{
    .channel             = 0,
    .ad_da_synchronized  = false,
    .p_extend            = &g_dac1_ext_cfg
};
/* Instance structure to use this module. */
const dac_instance_t g_dac1 =
{
    .p_ctrl    = &g_dac1_ctrl,
    .p_cfg     = &g_dac1_cfg,
    .p_api     = &g_dac_on_dac
};
icu_instance_ctrl_t g_external_irq0_ctrl;
const external_irq_cfg_t g_external_irq0_cfg =
{
    .channel             = 0,
    .trigger             = EXTERNAL_IRQ_TRIG_RISING,
    .filter_enable       = false,
    .pclk_div            = EXTERNAL_IRQ_PCLK_DIV_BY_64,
    .p_callback          = NULL,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = NULL,
    .ipl                 = (12),
#if defined(VECTOR_NUMBER_ICU_IRQ0)
    .irq                 = VECTOR_NUMBER_ICU_IRQ0,
#else
    .irq                 = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq0 =
{
    .p_ctrl        = &g_external_irq0_ctrl,
    .p_cfg         = &g_external_irq0_cfg,
    .p_api         = &g_external_irq_on_icu
};
poeg_instance_ctrl_t g_poeg0_ctrl;
const poeg_cfg_t g_poeg0_cfg =
{
    .trigger             = (poeg_trigger_t) (POEG_TRIGGER_PIN |  POEG_TRIGGER_SOFTWARE),
    .polarity            = POEG_GTETRG_POLARITY_ACTIVE_HIGH,
    .noise_filter        = POEG_GTETRG_NOISE_FILTER_DISABLED,
    .channel             = 0,
    .ipl                 = (BSP_IRQ_DISABLED),
    .p_callback          = NULL,
    .p_context           = NULL,
#if defined(VECTOR_NUMBER_POEG0_EVENT)
    .irq       = VECTOR_NUMBER_POEG0_EVENT,
#else
    .irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const poeg_instance_t g_poeg0 =
{
    .p_ctrl        = &g_poeg0_ctrl,
    .p_cfg         = &g_poeg0_cfg,
    .p_api         = &g_poeg_on_poeg
};
dac_instance_ctrl_t g_dac0_ctrl;
const dac_extended_cfg_t g_dac0_ext_cfg =
{
    .enable_charge_pump   = 0,
    .data_format         = DAC_DATA_FORMAT_FLUSH_RIGHT,
    .output_amplifier_enabled = 0,
    .internal_output_enabled = false,
};
const dac_cfg_t g_dac0_cfg =
{
    .channel             = 0,
    .ad_da_synchronized  = false,
    .p_extend            = &g_dac0_ext_cfg
};
/* Instance structure to use this module. */
const dac_instance_t g_dac0 =
{
    .p_ctrl    = &g_dac0_ctrl,
    .p_cfg     = &g_dac0_cfg,
    .p_api     = &g_dac_on_dac
};
void g_hal_init(void) {
g_common_init();
}
