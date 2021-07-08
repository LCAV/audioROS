/* DCMI attributes */
#define STM32_HAS_DCMI          TRUE
#define STM32_DCMI_DMA_MSK  (STM32_DMA_STREAM_ID_MSK(2, 1) | \
                                 STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_DCMI_DMA_CHN  0x10000010
