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
    .channel             = 1,
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
