/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (1)
        #endif
        /* ISR prototypes */
        void adc_b_adi0_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_ADC12_ADI0 ((IRQn_Type) 0) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #define ADC12_ADI0_IRQn          ((IRQn_Type) 0) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */