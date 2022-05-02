/* generated ELC source file - do not edit */
        #include "r_elc_api.h"
        const elc_cfg_t g_elc_cfg = {
                        .link[ELC_PERIPHERAL_GPT_A] = ELC_EVENT_GPT9_CAPTURE_COMPARE_A, /* GPT9 CAPTURE COMPARE A (Compare match A) */
            .link[ELC_PERIPHERAL_GPT_B] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_GPT_C] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_GPT_D] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_GPT_E] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_GPT_F] = ELC_EVENT_ICU_IRQ12, /* ICU IRQ12 (External pin interrupt 12) */
            .link[ELC_PERIPHERAL_GPT_G] = ELC_EVENT_ICU_IRQ7, /* ICU IRQ7 (External pin interrupt 7) */
            .link[ELC_PERIPHERAL_GPT_H] = ELC_EVENT_ICU_IRQ6, /* ICU IRQ6 (External pin interrupt 6) */
            .link[ELC_PERIPHERAL_DAC0] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_DAC1] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_IOPORTB] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_IOPORTC] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_IOPORTD] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_IOPORTE] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_ADC0] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_ADC0_B] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_ADC0_C] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_ADC1] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_ADC1_B] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_ADC1_C] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_DAC2] = ELC_EVENT_NONE, /* No allocation */
            .link[ELC_PERIPHERAL_DAC3] = ELC_EVENT_NONE, /* No allocation */
        };

#if BSP_TZ_SECURE_BUILD

        void R_BSP_ElcCfgSecurityInit(void);

        /* Configure ELC Security Attribution. */
        void R_BSP_ElcCfgSecurityInit(void)
        {
 #if (2U == BSP_FEATURE_ELC_VERSION)
            uint32_t elcsarbc = UINT32_MAX;

            elcsarbc &=  ~(1U << ELC_PERIPHERAL_GPT_A);
            elcsarbc &=  ~(1U << ELC_PERIPHERAL_GPT_F);
            elcsarbc &=  ~(1U << ELC_PERIPHERAL_GPT_G);
            elcsarbc &=  ~(1U << ELC_PERIPHERAL_GPT_H);

            /* Write the settings to ELCSARn Registers. */
            R_ELC->ELCSARA = 0xFFFFFFFEU;
            R_ELC->ELCSARB = elcsarbc;
 #else
            uint16_t elcsarbc[2] = {0xFFFFU, 0xFFFFU};
            elcsarbc[ELC_PERIPHERAL_GPT_A / 16U] &= (uint16_t) ~(1U << (ELC_PERIPHERAL_GPT_A % 16U));
            elcsarbc[ELC_PERIPHERAL_GPT_F / 16U] &= (uint16_t) ~(1U << (ELC_PERIPHERAL_GPT_F % 16U));
            elcsarbc[ELC_PERIPHERAL_GPT_G / 16U] &= (uint16_t) ~(1U << (ELC_PERIPHERAL_GPT_G % 16U));
            elcsarbc[ELC_PERIPHERAL_GPT_H / 16U] &= (uint16_t) ~(1U << (ELC_PERIPHERAL_GPT_H % 16U));

            /* Write the settins to ELCSARn Registers. */
            R_ELC->ELCSARA = 0xFFFEU;
            R_ELC->ELCSARB = elcsarbc[0];
            R_ELC->ELCSARC = elcsarbc[1];
 #endif
        }
#endif