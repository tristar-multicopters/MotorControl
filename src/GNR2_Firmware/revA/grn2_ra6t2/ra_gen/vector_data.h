/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (11)
        #endif
        /* ISR prototypes */
        void gpt_counter_overflow_isr(void);
        void adc_b_adi0_isr(void);
        void poeg_event_isr(void);
        void gpt_capture_a_isr(void);
        void r_icu_isr(void);
        void agt_int_isr(void);
        void gpt_capture_b_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_GPT4_COUNTER_OVERFLOW ((IRQn_Type) 0) /* GPT4 COUNTER OVERFLOW (Overflow) */
        #define GPT4_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 0) /* GPT4 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_ADC12_ADI0 ((IRQn_Type) 1) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #define ADC12_ADI0_IRQn          ((IRQn_Type) 1) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #define VECTOR_NUMBER_POEG1_EVENT ((IRQn_Type) 2) /* POEG1 EVENT (Port Output disable interrupt B) */
        #define POEG1_EVENT_IRQn          ((IRQn_Type) 2) /* POEG1 EVENT (Port Output disable interrupt B) */
        #define VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW ((IRQn_Type) 3) /* GPT0 COUNTER OVERFLOW (Overflow) */
        #define GPT0_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 3) /* GPT0 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_A ((IRQn_Type) 4) /* GPT0 CAPTURE COMPARE A (Compare match A) */
        #define GPT0_CAPTURE_COMPARE_A_IRQn          ((IRQn_Type) 4) /* GPT0 CAPTURE COMPARE A (Compare match A) */
        #define VECTOR_NUMBER_ICU_IRQ10 ((IRQn_Type) 5) /* ICU IRQ10 (External pin interrupt 10) */
        #define ICU_IRQ10_IRQn          ((IRQn_Type) 5) /* ICU IRQ10 (External pin interrupt 10) */
        #define VECTOR_NUMBER_ICU_IRQ11 ((IRQn_Type) 6) /* ICU IRQ11 (External pin interrupt 11) */
        #define ICU_IRQ11_IRQn          ((IRQn_Type) 6) /* ICU IRQ11 (External pin interrupt 11) */
        #define VECTOR_NUMBER_ICU_IRQ1 ((IRQn_Type) 7) /* ICU IRQ1 (External pin interrupt 1) */
        #define ICU_IRQ1_IRQn          ((IRQn_Type) 7) /* ICU IRQ1 (External pin interrupt 1) */
        #define VECTOR_NUMBER_AGT0_INT ((IRQn_Type) 8) /* AGT0 INT (AGT interrupt) */
        #define AGT0_INT_IRQn          ((IRQn_Type) 8) /* AGT0 INT (AGT interrupt) */
        #define VECTOR_NUMBER_GPT8_COUNTER_OVERFLOW ((IRQn_Type) 9) /* GPT8 COUNTER OVERFLOW (Overflow) */
        #define GPT8_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 9) /* GPT8 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_B ((IRQn_Type) 10) /* GPT8 CAPTURE COMPARE B (Compare match B) */
        #define GPT8_CAPTURE_COMPARE_B_IRQn          ((IRQn_Type) 10) /* GPT8 CAPTURE COMPARE B (Compare match B) */
        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */