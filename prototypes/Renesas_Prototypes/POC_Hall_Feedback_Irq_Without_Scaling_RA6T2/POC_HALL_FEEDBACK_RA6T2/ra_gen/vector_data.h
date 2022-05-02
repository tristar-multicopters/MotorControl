/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (7)
        #endif
        /* ISR prototypes */
        void poeg_event_isr(void);
        void adc_b_adi0_isr(void);
        void gpt_counter_overflow_isr(void);
        void gpt_capture_a_isr(void);
        void r_icu_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_POEG3_EVENT ((IRQn_Type) 0) /* POEG3 EVENT (Port Output disable interrupt D) */
        #define POEG3_EVENT_IRQn          ((IRQn_Type) 0) /* POEG3 EVENT (Port Output disable interrupt D) */
        #define VECTOR_NUMBER_ADC12_ADI0 ((IRQn_Type) 1) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #define ADC12_ADI0_IRQn          ((IRQn_Type) 1) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #define VECTOR_NUMBER_GPT9_COUNTER_OVERFLOW ((IRQn_Type) 2) /* GPT9 COUNTER OVERFLOW (Overflow) */
        #define GPT9_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 2) /* GPT9 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT9_CAPTURE_COMPARE_A ((IRQn_Type) 3) /* GPT9 CAPTURE COMPARE A (Compare match A) */
        #define GPT9_CAPTURE_COMPARE_A_IRQn          ((IRQn_Type) 3) /* GPT9 CAPTURE COMPARE A (Compare match A) */
        #define VECTOR_NUMBER_ICU_IRQ6 ((IRQn_Type) 4) /* ICU IRQ6 (External pin interrupt 6) */
        #define ICU_IRQ6_IRQn          ((IRQn_Type) 4) /* ICU IRQ6 (External pin interrupt 6) */
        #define VECTOR_NUMBER_ICU_IRQ7 ((IRQn_Type) 5) /* ICU IRQ7 (External pin interrupt 7) */
        #define ICU_IRQ7_IRQn          ((IRQn_Type) 5) /* ICU IRQ7 (External pin interrupt 7) */
        #define VECTOR_NUMBER_ICU_IRQ12 ((IRQn_Type) 6) /* ICU IRQ12 (External pin interrupt 12) */
        #define ICU_IRQ12_IRQn          ((IRQn_Type) 6) /* ICU IRQ12 (External pin interrupt 12) */
        #endif /* VECTOR_DATA_H */