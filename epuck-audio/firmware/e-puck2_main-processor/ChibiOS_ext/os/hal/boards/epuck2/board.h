/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for GCTronic-MobotsEPFL e-puck V2.0_F board.
 */

/*
 * Board identifier.
 */
#define BOARD_epuck2
#define BOARD_NAME                  "GCTronic-MobotsEPFL e-puck V2.0_F"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000U
#endif

#define STM32_HSE_BYPASS

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F407xx

/*
 * IO pins assignments.
 */
#define GPIOA_CAM_MCLK              0U
#define GPIOA_BATT_AN               1U
#define GPIOA_MIC_TIMER_IN          2U
#define GPIOA_MIC_TIMER_OUT         3U
#define GPIOA_CAM_HSYNC             4U
#define GPIOA_AUDIO_SPEAKER         5U
#define GPIOA_CAM_PCLK              6U
#define GPIOA_SPI1_MOSI             7U
#define GPIOA_GEN_IO_1              8U
#define GPIOA_USB_PRESENT           9U
#define GPIOA_DIST_INT              10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_SPI1_CS_ESP32         15U

#define GPIOB_IR5_AN                0U
#define GPIOB_IR4_AN                1U
#define GPIOB_LED_BODY              2U
#define GPIOB_SPI1_SCK              3U
#define GPIOB_SPI1_MISO             4U
#define GPIOB_MIC34_DATA            5U
#define GPIOB_CAM_D5                6U
#define GPIOB_CAM_VSYNC             7U
#define GPIOB_SCL                   8U
#define GPIOB_SDA                   9U
#define GPIOB_SPI1_CS_ENC_L         10U
#define GPIOB_ENC_L_INT             11U
#define GPIOB_PULSE_0               12U
#define GPIOB_MIC_SPI2_SCK          13U
#define GPIOB_PULSE_1               14U
#define GPIOB_MIC12_DATA            15U

#define GPIOC_IR6_AN                0U
#define GPIOC_IR7_AN                1U
#define GPIOC_IR0_AN                2U
#define GPIOC_IR1_AN                3U
#define GPIOC_IR2_AN                4U
#define GPIOC_IR3_AN                5U
#define GPIOC_CAM_D0                6U
#define GPIOC_CAM_D1                7U
#define GPIOC_SDIO_D0               8U
#define GPIOC_SDIO_D1               9U
#define GPIOC_MIC_SPI3_SCK          10U
#define GPIOC_SDIO_D3               11U
#define GPIOC_SDIO_CLK              12U
#define GPIOC_SEL_0                 13U
#define GPIOC_SEL_1                 14U
#define GPIOC_SEL_2                 15U

#define GPIOD_CAN_RX                0U
#define GPIOD_CAN_TX                1U
#define GPIOD_SDIO_CMD              2U
#define GPIOD_REMOTE                3U
#define GPIOD_SEL_3                 4U
#define GPIOD_LED1                  5U
#define GPIOD_LED3                  6U
#define GPIOD_AUDIO_PWR             7U
#define GPIOD_UART_TX               8U
#define GPIOD_UART_RX               9U
#define GPIOD_LED5                  10U
#define GPIOD_LED7                  11U
#define GPIOD_SPI1_CS_ENC_R         12U
#define GPIOD_ENC_R_INT             13U
#define GPIOD_LED_FRONT             14U
#define GPIOD_IMU_INT               15U

#define GPIOE_CAM_D2                0U
#define GPIOE_CAM_D3                1U
#define GPIOE_PULSE_2               2U
#define GPIOE_PULSE_3               3U
#define GPIOE_CAM_D4                4U
#define GPIOE_CAM_D6                5U
#define GPIOE_CAM_D7                6U
#define GPIOE_SD_DETECT             7U
#define GPIOE_MOT_L_IN2             8U
#define GPIOE_MOT_L_IN1             9U
#define GPIOE_MOT_L_IN4             10U
#define GPIOE_MOT_L_IN3             11U
#define GPIOE_MOT_R_IN2             12U
#define GPIOE_MOT_R_IN1             13U
#define GPIOE_MOT_R_IN3             14U
#define GPIOE_MOT_R_IN4             15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_PIN2                  2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_PIN9                  9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_PIN13                 13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_CAM_MCLK               PAL_LINE(GPIOA, 0U)
#define LINE_BATT_AN                PAL_LINE(GPIOA, 1U)
#define LINE_MIC_TIMER_IN           PAL_LINE(GPIOA, 2U)
#define LINE_MIC_TIMER_OUT          PAL_LINE(GPIOA, 3U)
#define LINE_CAM_HSYNC              PAL_LINE(GPIOA, 4U)
#define LINE_AUDIO_SPEAKER          PAL_LINE(GPIOA, 5U)
#define LINE_CAM_PCLK               PAL_LINE(GPIOA, 6U)
#define LINE_SPI1_MOSI              PAL_LINE(GPIOA, 7U)
#define LINE_GEN_IO_1               PAL_LINE(GPIOA, 8U)
#define LINE_USB_PRESENT            PAL_LINE(GPIOA, 9U)
#define LINE_DIST_INT               PAL_LINE(GPIOA, 10U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_SPI1_CS_ESP32          PAL_LINE(GPIOA, 15U)

#define LINE_IR5_AN                 PAL_LINE(GPIOB, 0U)
#define LINE_IR4_AN                 PAL_LINE(GPIOB, 1U)
#define LINE_LED_BODY               PAL_LINE(GPIOB, 2U)
#define LINE_SPI1_SCK               PAL_LINE(GPIOB, 3U)
#define LINE_SPI1_MISO              PAL_LINE(GPIOB, 4U)
#define LINE_MIC34_DATA             PAL_LINE(GPIOB, 5U)
#define LINE_CAM_D5                 PAL_LINE(GPIOB, 6U)
#define LINE_CAM_VSYNC              PAL_LINE(GPIOB, 7U)
#define LINE_SCL                    PAL_LINE(GPIOB, 8U)
#define LINE_SDA                    PAL_LINE(GPIOB, 9U)
#define LINE_SPI1_CS_ENC_L          PAL_LINE(GPIOB, 10U)
#define LINE_ENC_L_INT              PAL_LINE(GPIOB, 11U)
#define LINE_PULSE_0                PAL_LINE(GPIOB, 12U)
#define LINE_MIC_SPI2_SCK           PAL_LINE(GPIOB, 13U)
#define LINE_PULSE_1                PAL_LINE(GPIOB, 14U)
#define LINE_MIC12_DATA             PAL_LINE(GPIOB, 15U)

#define LINE_IR6_AN                 PAL_LINE(GPIOC, 0U)
#define LINE_IR7_AN                 PAL_LINE(GPIOC, 1U)
#define LINE_IR0_AN                 PAL_LINE(GPIOC, 2U)
#define LINE_IR1_AN                 PAL_LINE(GPIOC, 3U)
#define LINE_IR2_AN                 PAL_LINE(GPIOC, 4U)
#define LINE_IR3_AN                 PAL_LINE(GPIOC, 5U)
#define LINE_CAM_D0                 PAL_LINE(GPIOC, 6U)
#define LINE_CAM_D1                 PAL_LINE(GPIOC, 7U)
#define LINE_SDIO_D0                PAL_LINE(GPIOC, 8U)
#define LINE_SDIO_D1                PAL_LINE(GPIOC, 9U)
#define LINE_MIC_SPI3_SCK           PAL_LINE(GPIOC, 10U)
#define LINE_SDIO_D3                PAL_LINE(GPIOC, 11U)
#define LINE_SDIO_CLK               PAL_LINE(GPIOC, 12U)
#define LINE_SEL_0                  PAL_LINE(GPIOC, 13U)
#define LINE_SEL_1                  PAL_LINE(GPIOC, 14U)
#define LINE_SEL_2                  PAL_LINE(GPIOC, 15U)

#define LINE_CAN_RX                 PAL_LINE(GPIOD, 0U)
#define LINE_CAN_TX                 PAL_LINE(GPIOD, 1U)
#define LINE_SDIO_CMD               PAL_LINE(GPIOD, 2U)
#define LINE_REMOTE                 PAL_LINE(GPIOD, 3U)
#define LINE_SEL_3                  PAL_LINE(GPIOD, 4U)
#define LINE_LED1                   PAL_LINE(GPIOD, 5U)
#define LINE_LED3                   PAL_LINE(GPIOD, 6U)
#define LINE_AUDIO_PWR              PAL_LINE(GPIOD, 7U)
#define LINE_UART_TX                PAL_LINE(GPIOD, 8U)
#define LINE_UART_RX                PAL_LINE(GPIOD, 9U)
#define LINE_LED5                   PAL_LINE(GPIOD, 10U)
#define LINE_LED7                   PAL_LINE(GPIOD, 11U)
#define LINE_SPI1_CS_ENC_R          PAL_LINE(GPIOD, 12U)
#define LINE_ENC_R_INT              PAL_LINE(GPIOD, 13U)
#define LINE_LED_FRONT              PAL_LINE(GPIOD, 14U)
#define LINE_IMU_INT                PAL_LINE(GPIOD, 15U)

#define LINE_CAM_D2                 PAL_LINE(GPIOE, 0U)
#define LINE_CAM_D3                 PAL_LINE(GPIOE, 1U)
#define LINE_PULSE_2                PAL_LINE(GPIOE, 2U)
#define LINE_PULSE_3                PAL_LINE(GPIOE, 3U)
#define LINE_CAM_D4                 PAL_LINE(GPIOE, 4U)
#define LINE_CAM_D6                 PAL_LINE(GPIOE, 5U)
#define LINE_CAM_D7                 PAL_LINE(GPIOE, 6U)
#define LINE_SD_DETECT              PAL_LINE(GPIOE, 7U)
#define LINE_MOT_L_IN2              PAL_LINE(GPIOE, 8U)
#define LINE_MOT_L_IN1              PAL_LINE(GPIOE, 9U)
#define LINE_MOT_L_IN4              PAL_LINE(GPIOE, 10U)
#define LINE_MOT_L_IN3              PAL_LINE(GPIOE, 11U)
#define LINE_MOT_R_IN2              PAL_LINE(GPIOE, 12U)
#define LINE_MOT_R_IN1              PAL_LINE(GPIOE, 13U)
#define LINE_MOT_R_IN3              PAL_LINE(GPIOE, 14U)
#define LINE_MOT_R_IN4              PAL_LINE(GPIOE, 15U)



#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - CAM_MCLK                  (alternate 2).
 * PA1  - BATT_AN                   (analog).
 * PA2  - MIC_TIMER_IN              (alternate 3).
 * PA3  - MIC_TIMER_OUT             (alternate 3).
 * PA4  - CAM_HSYNC                 (alternate 13).
 * PA5  - AUDIO_SPEAKER             (analog).
 * PA6  - CAM_PCLK                  (alternate 13).
 * PA7  - SPI1_MOSI                 (alternate 5).
 * PA8  - GEN_IO_1                  (input pullup).
 * PA9  - USB_PRESENT               (input floating).
 * PA10 - DIST_INT                  (input floating).
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - SPI1_CS_ESP32             (output pushpull maximum).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_CAM_MCLK) |   \
                                     PIN_MODE_ANALOG(GPIOA_BATT_AN) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_MIC_TIMER_IN) |\
                                     PIN_MODE_ALTERNATE(GPIOA_MIC_TIMER_OUT) |\
                                     PIN_MODE_ALTERNATE(GPIOA_CAM_HSYNC) |  \
                                     PIN_MODE_ANALOG(GPIOA_AUDIO_SPEAKER) | \
                                     PIN_MODE_ALTERNATE(GPIOA_CAM_PCLK) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) |  \
                                     PIN_MODE_INPUT(GPIOA_GEN_IO_1) |       \
                                     PIN_MODE_INPUT(GPIOA_USB_PRESENT) |    \
                                     PIN_MODE_INPUT(GPIOA_DIST_INT) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_OUTPUT(GPIOA_SPI1_CS_ESP32))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_CAM_MCLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_BATT_AN) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MIC_TIMER_IN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_MIC_TIMER_OUT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAM_HSYNC) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AUDIO_SPEAKER) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAM_PCLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_GEN_IO_1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_PRESENT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_DIST_INT) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_CS_ESP32))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_CAM_MCLK) |      \
                                     PIN_OSPEED_HIGH(GPIOA_BATT_AN) |       \
                                     PIN_OSPEED_HIGH(GPIOA_MIC_TIMER_IN) |  \
                                     PIN_OSPEED_HIGH(GPIOA_MIC_TIMER_OUT) | \
                                     PIN_OSPEED_HIGH(GPIOA_CAM_HSYNC) |     \
                                     PIN_OSPEED_HIGH(GPIOA_AUDIO_SPEAKER) | \
                                     PIN_OSPEED_HIGH(GPIOA_CAM_PCLK) |      \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MOSI) |     \
                                     PIN_OSPEED_HIGH(GPIOA_GEN_IO_1) |      \
                                     PIN_OSPEED_HIGH(GPIOA_USB_PRESENT) |   \
                                     PIN_OSPEED_HIGH(GPIOA_DIST_INT) |      \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DM) |     \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DP) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_CS_ESP32))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_CAM_MCLK) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_BATT_AN) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_MIC_TIMER_IN) |\
                                     PIN_PUPDR_FLOATING(GPIOA_MIC_TIMER_OUT) |\
                                     PIN_PUPDR_PULLUP(GPIOA_CAM_HSYNC) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_AUDIO_SPEAKER) |\
                                     PIN_PUPDR_PULLUP(GPIOA_CAM_PCLK) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MOSI) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_GEN_IO_1) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_PRESENT) |\
                                     PIN_PUPDR_FLOATING(GPIOA_DIST_INT) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_CS_ESP32))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_CAM_MCLK) |         \
                                     PIN_ODR_HIGH(GPIOA_BATT_AN) |          \
                                     PIN_ODR_HIGH(GPIOA_MIC_TIMER_IN) |     \
                                     PIN_ODR_HIGH(GPIOA_MIC_TIMER_OUT) |    \
                                     PIN_ODR_HIGH(GPIOA_CAM_HSYNC) |        \
                                     PIN_ODR_HIGH(GPIOA_AUDIO_SPEAKER) |    \
                                     PIN_ODR_HIGH(GPIOA_CAM_PCLK) |         \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOA_GEN_IO_1) |         \
                                     PIN_ODR_HIGH(GPIOA_USB_PRESENT) |      \
                                     PIN_ODR_HIGH(GPIOA_DIST_INT) |         \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |        \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |        \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_SPI1_CS_ESP32))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_CAM_MCLK, 2U) |      \
                                     PIN_AFIO_AF(GPIOA_BATT_AN, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_MIC_TIMER_IN, 3U) |  \
                                     PIN_AFIO_AF(GPIOA_MIC_TIMER_OUT, 3U) | \
                                     PIN_AFIO_AF(GPIOA_CAM_HSYNC, 13U) |    \
                                     PIN_AFIO_AF(GPIOA_AUDIO_SPEAKER, 0U) | \
                                     PIN_AFIO_AF(GPIOA_CAM_PCLK, 13U) |     \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI, 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_GEN_IO_1, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_USB_PRESENT, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_DIST_INT, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SPI1_CS_ESP32, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - IR5_AN                    (analog).
 * PB1  - IR4_AN                    (analog).
 * PB2  - LED_BODY                  (output pushpull maximum).
 * PB3  - SPI1_SCK                  (alternate 5).
 * PB4  - SPI1_MISO                 (alternate 5).
 * PB5  - MIC34_DATA                (alternate 6).
 * PB6  - CAM_D5                    (alternate 13).
 * PB7  - CAM_VSYNC                 (alternate 13).
 * PB8  - SCL                       (alternate 4).
 * PB9  - SDA                       (alternate 4).
 * PB10 - SPI1_CS_ENC_L             (output pushpull maximum).
 * PB11 - ENC_L_INT                 (input pullup).
 * PB12 - PULSE_0                   (output pushpull maximum).
 * PB13 - MIC_SPI2_SCK              (alternate 5).
 * PB14 - PULSE_1                   (output pushpull maximum).
 * PB15 - MIC12_DATA                (alternate 5).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_IR5_AN) |        \
                                     PIN_MODE_ANALOG(GPIOB_IR4_AN) |        \
                                     PIN_MODE_OUTPUT(GPIOB_LED_BODY) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI1_SCK) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI1_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_MIC34_DATA) | \
                                     PIN_MODE_ALTERNATE(GPIOB_CAM_D5) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_CAM_VSYNC) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SCL) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_SDA) |        \
                                     PIN_MODE_OUTPUT(GPIOB_SPI1_CS_ENC_L) | \
                                     PIN_MODE_INPUT(GPIOB_ENC_L_INT) |      \
                                     PIN_MODE_OUTPUT(GPIOB_PULSE_0) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_MIC_SPI2_SCK) |\
                                     PIN_MODE_OUTPUT(GPIOB_PULSE_1) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_MIC12_DATA))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_IR5_AN) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_IR4_AN) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_BODY) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI1_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI1_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MIC34_DATA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAM_D5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAM_VSYNC) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_SCL) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_SDA) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI1_CS_ENC_L) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENC_L_INT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PULSE_0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MIC_SPI2_SCK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PULSE_1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MIC12_DATA))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_IR5_AN) |        \
                                     PIN_OSPEED_HIGH(GPIOB_IR4_AN) |        \
                                     PIN_OSPEED_HIGH(GPIOB_LED_BODY) |      \
                                     PIN_OSPEED_HIGH(GPIOB_SPI1_SCK) |      \
                                     PIN_OSPEED_HIGH(GPIOB_SPI1_MISO) |     \
                                     PIN_OSPEED_HIGH(GPIOB_MIC34_DATA) |    \
                                     PIN_OSPEED_HIGH(GPIOB_CAM_D5) |        \
                                     PIN_OSPEED_HIGH(GPIOB_CAM_VSYNC) |     \
                                     PIN_OSPEED_HIGH(GPIOB_SCL) |           \
                                     PIN_OSPEED_HIGH(GPIOB_SDA) |           \
                                     PIN_OSPEED_HIGH(GPIOB_SPI1_CS_ENC_L) | \
                                     PIN_OSPEED_HIGH(GPIOB_ENC_L_INT) |     \
                                     PIN_OSPEED_HIGH(GPIOB_PULSE_0) |       \
                                     PIN_OSPEED_HIGH(GPIOB_MIC_SPI2_SCK) |  \
                                     PIN_OSPEED_HIGH(GPIOB_PULSE_1) |       \
                                     PIN_OSPEED_HIGH(GPIOB_MIC12_DATA))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_IR5_AN) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_IR4_AN) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_BODY) |   \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI1_SCK) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI1_MISO) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_MIC34_DATA) |   \
                                     PIN_PUPDR_PULLUP(GPIOB_CAM_D5) |       \
                                     PIN_PUPDR_PULLUP(GPIOB_CAM_VSYNC) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_SCL) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_SDA) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI1_CS_ENC_L) |\
                                     PIN_PUPDR_PULLUP(GPIOB_ENC_L_INT) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_PULSE_0) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_MIC_SPI2_SCK) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PULSE_1) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_MIC12_DATA))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_IR5_AN) |            \
                                     PIN_ODR_LOW(GPIOB_IR4_AN) |            \
                                     PIN_ODR_LOW(GPIOB_LED_BODY) |          \
                                     PIN_ODR_HIGH(GPIOB_SPI1_SCK) |         \
                                     PIN_ODR_HIGH(GPIOB_SPI1_MISO) |        \
                                     PIN_ODR_HIGH(GPIOB_MIC34_DATA) |       \
                                     PIN_ODR_HIGH(GPIOB_CAM_D5) |           \
                                     PIN_ODR_HIGH(GPIOB_CAM_VSYNC) |        \
                                     PIN_ODR_HIGH(GPIOB_SCL) |              \
                                     PIN_ODR_HIGH(GPIOB_SDA) |              \
                                     PIN_ODR_HIGH(GPIOB_SPI1_CS_ENC_L) |    \
                                     PIN_ODR_HIGH(GPIOB_ENC_L_INT) |        \
                                     PIN_ODR_LOW(GPIOB_PULSE_0) |           \
                                     PIN_ODR_HIGH(GPIOB_MIC_SPI2_SCK) |     \
                                     PIN_ODR_LOW(GPIOB_PULSE_1) |           \
                                     PIN_ODR_HIGH(GPIOB_MIC12_DATA))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_IR5_AN, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_IR4_AN, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_LED_BODY, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_SPI1_SCK, 5U) |      \
                                     PIN_AFIO_AF(GPIOB_SPI1_MISO, 5U) |     \
                                     PIN_AFIO_AF(GPIOB_MIC34_DATA, 6U) |    \
                                     PIN_AFIO_AF(GPIOB_CAM_D5, 13U) |       \
                                     PIN_AFIO_AF(GPIOB_CAM_VSYNC, 13U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_SCL, 4U) |           \
                                     PIN_AFIO_AF(GPIOB_SDA, 4U) |           \
                                     PIN_AFIO_AF(GPIOB_SPI1_CS_ENC_L, 0U) | \
                                     PIN_AFIO_AF(GPIOB_ENC_L_INT, 0U) |     \
                                     PIN_AFIO_AF(GPIOB_PULSE_0, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_MIC_SPI2_SCK, 5U) |  \
                                     PIN_AFIO_AF(GPIOB_PULSE_1, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_MIC12_DATA, 5U))

/*
 * GPIOC setup:
 *
 * PC0  - IR6_AN                    (analog).
 * PC1  - IR7_AN                    (analog).
 * PC2  - IR0_AN                    (analog).
 * PC3  - IR1_AN                    (analog).
 * PC4  - IR2_AN                    (analog).
 * PC5  - IR3_AN                    (analog).
 * PC6  - CAM_D0                    (alternate 13).
 * PC7  - CAM_D1                    (alternate 13).
 * PC8  - SDIO_D0                   (alternate 12).
 * PC9  - SDIO_D1                   (alternate 12).
 * PC10 - MIC_SPI3_SCK              (alternate 6).
 * PC11 - SDIO_D3                   (alternate 12).
 * PC12 - SDIO_CLK                  (alternate 12).
 * PC13 - SEL_0                     (input pulldown).
 * PC14 - SEL_1                     (input pulldown).
 * PC15 - SEL_2                     (input pulldown).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_IR6_AN) |        \
                                     PIN_MODE_ANALOG(GPIOC_IR7_AN) |        \
                                     PIN_MODE_ANALOG(GPIOC_IR0_AN) |        \
                                     PIN_MODE_ANALOG(GPIOC_IR1_AN) |        \
                                     PIN_MODE_ANALOG(GPIOC_IR2_AN) |        \
                                     PIN_MODE_ANALOG(GPIOC_IR3_AN) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_CAM_D0) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_CAM_D1) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D0) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D1) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_MIC_SPI3_SCK) |\
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D3) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_CLK) |   \
                                     PIN_MODE_INPUT(GPIOC_SEL_0) |          \
                                     PIN_MODE_INPUT(GPIOC_SEL_1) |          \
                                     PIN_MODE_INPUT(GPIOC_SEL_2))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOC_IR6_AN) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_IR7_AN) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_IR0_AN) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_IR1_AN) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_IR2_AN) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_IR3_AN) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_CAM_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_CAM_D1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MIC_SPI3_SCK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SEL_0) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SEL_1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SEL_2))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_IR6_AN) |        \
                                     PIN_OSPEED_HIGH(GPIOC_IR7_AN) |        \
                                     PIN_OSPEED_HIGH(GPIOC_IR0_AN) |        \
                                     PIN_OSPEED_HIGH(GPIOC_IR1_AN) |        \
                                     PIN_OSPEED_HIGH(GPIOC_IR2_AN) |        \
                                     PIN_OSPEED_HIGH(GPIOC_IR3_AN) |        \
                                     PIN_OSPEED_HIGH(GPIOC_CAM_D0) |        \
                                     PIN_OSPEED_HIGH(GPIOC_CAM_D1) |        \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D0) |       \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D1) |       \
                                     PIN_OSPEED_HIGH(GPIOC_MIC_SPI3_SCK) |  \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D3) |       \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_CLK) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SEL_0) |         \
                                     PIN_OSPEED_HIGH(GPIOC_SEL_1) |         \
                                     PIN_OSPEED_HIGH(GPIOC_SEL_2))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_IR6_AN) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_IR7_AN) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_IR0_AN) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_IR1_AN) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_IR2_AN) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_IR3_AN) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_CAM_D0) |       \
                                     PIN_PUPDR_PULLUP(GPIOC_CAM_D1) |       \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_MIC_SPI3_SCK) | \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D3) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_CLK) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOC_SEL_0) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_SEL_1) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_SEL_2))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_IR6_AN) |            \
                                     PIN_ODR_LOW(GPIOC_IR7_AN) |            \
                                     PIN_ODR_LOW(GPIOC_IR0_AN) |            \
                                     PIN_ODR_LOW(GPIOC_IR1_AN) |            \
                                     PIN_ODR_LOW(GPIOC_IR2_AN) |            \
                                     PIN_ODR_LOW(GPIOC_IR3_AN) |            \
                                     PIN_ODR_HIGH(GPIOC_CAM_D0) |           \
                                     PIN_ODR_HIGH(GPIOC_CAM_D1) |           \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D0) |          \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D1) |          \
                                     PIN_ODR_HIGH(GPIOC_MIC_SPI3_SCK) |     \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D3) |          \
                                     PIN_ODR_HIGH(GPIOC_SDIO_CLK) |         \
                                     PIN_ODR_LOW(GPIOC_SEL_0) |             \
                                     PIN_ODR_LOW(GPIOC_SEL_1) |             \
                                     PIN_ODR_LOW(GPIOC_SEL_2))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_IR6_AN, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_IR7_AN, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_IR0_AN, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_IR1_AN, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_IR2_AN, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_IR3_AN, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_CAM_D0, 13U) |       \
                                     PIN_AFIO_AF(GPIOC_CAM_D1, 13U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SDIO_D0, 12U) |      \
                                     PIN_AFIO_AF(GPIOC_SDIO_D1, 12U) |      \
                                     PIN_AFIO_AF(GPIOC_MIC_SPI3_SCK, 6U) |  \
                                     PIN_AFIO_AF(GPIOC_SDIO_D3, 12U) |      \
                                     PIN_AFIO_AF(GPIOC_SDIO_CLK, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SEL_0, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_SEL_1, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_SEL_2, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - CAN_RX                    (alternate 9).
 * PD1  - CAN_TX                    (alternate 9).
 * PD2  - SDIO_CMD                  (alternate 12).
 * PD3  - REMOTE                    (input pullup).
 * PD4  - SEL_3                     (input pulldown).
 * PD5  - LED1                      (output pushpull maximum).
 * PD6  - LED3                      (output pushpull maximum).
 * PD7  - AUDIO_PWR                 (output pushpull maximum).
 * PD8  - UART_TX                   (alternate 7).
 * PD9  - UART_RX                   (alternate 7).
 * PD10 - LED5                      (output pushpull maximum).
 * PD11 - LED7                      (output pushpull maximum).
 * PD12 - SPI1_CS_ENC_R             (output pushpull maximum).
 * PD13 - ENC_R_INT                 (input pullup).
 * PD14 - LED_FRONT                 (output pushpull maximum).
 * PD15 - IMU_INT                   (input floating).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_CAN_RX) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_CAN_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_SDIO_CMD) |   \
                                     PIN_MODE_INPUT(GPIOD_REMOTE) |         \
                                     PIN_MODE_INPUT(GPIOD_SEL_3) |          \
                                     PIN_MODE_OUTPUT(GPIOD_LED1) |          \
                                     PIN_MODE_OUTPUT(GPIOD_LED3) |          \
                                     PIN_MODE_OUTPUT(GPIOD_AUDIO_PWR) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_UART_TX) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_UART_RX) |    \
                                     PIN_MODE_OUTPUT(GPIOD_LED5) |          \
                                     PIN_MODE_OUTPUT(GPIOD_LED7) |          \
                                     PIN_MODE_OUTPUT(GPIOD_SPI1_CS_ENC_R) | \
                                     PIN_MODE_INPUT(GPIOD_ENC_R_INT) |      \
                                     PIN_MODE_OUTPUT(GPIOD_LED_FRONT) |     \
                                     PIN_MODE_INPUT(GPIOD_IMU_INT))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_CAN_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CAN_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDIO_CMD) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_REMOTE) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SEL_3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LED1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LED3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_AUDIO_PWR) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_UART_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_UART_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LED5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LED7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SPI1_CS_ENC_R) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_ENC_R_INT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LED_FRONT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_IMU_INT))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_CAN_RX) |        \
                                     PIN_OSPEED_HIGH(GPIOD_CAN_TX) |        \
                                     PIN_OSPEED_HIGH(GPIOD_SDIO_CMD) |      \
                                     PIN_OSPEED_HIGH(GPIOD_REMOTE) |        \
                                     PIN_OSPEED_HIGH(GPIOD_SEL_3) |         \
                                     PIN_OSPEED_HIGH(GPIOD_LED1) |          \
                                     PIN_OSPEED_HIGH(GPIOD_LED3) |          \
                                     PIN_OSPEED_HIGH(GPIOD_AUDIO_PWR) |     \
                                     PIN_OSPEED_HIGH(GPIOD_UART_TX) |       \
                                     PIN_OSPEED_HIGH(GPIOD_UART_RX) |       \
                                     PIN_OSPEED_HIGH(GPIOD_LED5) |          \
                                     PIN_OSPEED_HIGH(GPIOD_LED7) |          \
                                     PIN_OSPEED_HIGH(GPIOD_SPI1_CS_ENC_R) | \
                                     PIN_OSPEED_HIGH(GPIOD_ENC_R_INT) |     \
                                     PIN_OSPEED_HIGH(GPIOD_LED_FRONT) |     \
                                     PIN_OSPEED_HIGH(GPIOD_IMU_INT))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_CAN_RX) |       \
                                     PIN_PUPDR_PULLUP(GPIOD_CAN_TX) |       \
                                     PIN_PUPDR_PULLUP(GPIOD_SDIO_CMD) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_REMOTE) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_SEL_3) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_LED1) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_LED3) |       \
                                     PIN_PUPDR_PULLUP(GPIOD_AUDIO_PWR) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_UART_TX) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_UART_RX) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_LED5) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_LED7) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_SPI1_CS_ENC_R) |\
                                     PIN_PUPDR_PULLUP(GPIOD_ENC_R_INT) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_LED_FRONT) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_IMU_INT))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_CAN_RX) |           \
                                     PIN_ODR_HIGH(GPIOD_CAN_TX) |           \
                                     PIN_ODR_HIGH(GPIOD_SDIO_CMD) |         \
                                     PIN_ODR_HIGH(GPIOD_REMOTE) |           \
                                     PIN_ODR_LOW(GPIOD_SEL_3) |             \
                                     PIN_ODR_HIGH(GPIOD_LED1) |             \
                                     PIN_ODR_HIGH(GPIOD_LED3) |             \
                                     PIN_ODR_LOW(GPIOD_AUDIO_PWR) |         \
                                     PIN_ODR_HIGH(GPIOD_UART_TX) |          \
                                     PIN_ODR_HIGH(GPIOD_UART_RX) |          \
                                     PIN_ODR_HIGH(GPIOD_LED5) |             \
                                     PIN_ODR_HIGH(GPIOD_LED7) |             \
                                     PIN_ODR_HIGH(GPIOD_SPI1_CS_ENC_R) |    \
                                     PIN_ODR_HIGH(GPIOD_ENC_R_INT) |        \
                                     PIN_ODR_LOW(GPIOD_LED_FRONT) |         \
                                     PIN_ODR_HIGH(GPIOD_IMU_INT))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_CAN_RX, 9U) |        \
                                     PIN_AFIO_AF(GPIOD_CAN_TX, 9U) |        \
                                     PIN_AFIO_AF(GPIOD_SDIO_CMD, 12U) |     \
                                     PIN_AFIO_AF(GPIOD_REMOTE, 0U) |        \
                                     PIN_AFIO_AF(GPIOD_SEL_3, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_LED1, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_LED3, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_AUDIO_PWR, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_UART_TX, 7U) |       \
                                     PIN_AFIO_AF(GPIOD_UART_RX, 7U) |       \
                                     PIN_AFIO_AF(GPIOD_LED5, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_LED7, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_SPI1_CS_ENC_R, 0U) | \
                                     PIN_AFIO_AF(GPIOD_ENC_R_INT, 0U) |     \
                                     PIN_AFIO_AF(GPIOD_LED_FRONT, 0U) |     \
                                     PIN_AFIO_AF(GPIOD_IMU_INT, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - CAM_D2                    (alternate 13).
 * PE1  - CAM_D3                    (alternate 13).
 * PE2  - PULSE_2                   (output pushpull maximum).
 * PE3  - PULSE_3                   (output pushpull maximum).
 * PE4  - CAM_D4                    (alternate 13).
 * PE5  - CAM_D6                    (alternate 13).
 * PE6  - CAM_D7                    (alternate 13).
 * PE7  - SD_DETECT                 (input floating).
 * PE8  - MOT_L_IN2                 (output pushpull maximum).
 * PE9  - MOT_L_IN1                 (output pushpull maximum).
 * PE10 - MOT_L_IN4                 (output pushpull maximum).
 * PE11 - MOT_L_IN3                 (output pushpull maximum).
 * PE12 - MOT_R_IN2                 (output pushpull maximum).
 * PE13 - MOT_R_IN1                 (output pushpull maximum).
 * PE14 - MOT_R_IN3                 (output pushpull maximum).
 * PE15 - MOT_R_IN4                 (output pushpull maximum).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_CAM_D2) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_CAM_D3) |     \
                                     PIN_MODE_OUTPUT(GPIOE_PULSE_2) |       \
                                     PIN_MODE_OUTPUT(GPIOE_PULSE_3) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_CAM_D4) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_CAM_D6) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_CAM_D7) |     \
                                     PIN_MODE_INPUT(GPIOE_SD_DETECT) |      \
                                     PIN_MODE_OUTPUT(GPIOE_MOT_L_IN2) |     \
                                     PIN_MODE_OUTPUT(GPIOE_MOT_L_IN1) |     \
                                     PIN_MODE_OUTPUT(GPIOE_MOT_L_IN4) |     \
                                     PIN_MODE_OUTPUT(GPIOE_MOT_L_IN3) |     \
                                     PIN_MODE_OUTPUT(GPIOE_MOT_R_IN2) |     \
                                     PIN_MODE_OUTPUT(GPIOE_MOT_R_IN1) |     \
                                     PIN_MODE_OUTPUT(GPIOE_MOT_R_IN3) |     \
                                     PIN_MODE_OUTPUT(GPIOE_MOT_R_IN4))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_CAM_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_CAM_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PULSE_2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PULSE_3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_CAM_D4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_CAM_D6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_CAM_D7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SD_DETECT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MOT_L_IN2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MOT_L_IN1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MOT_L_IN4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MOT_L_IN3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MOT_R_IN2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MOT_R_IN1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MOT_R_IN3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MOT_R_IN4))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_CAM_D2) |        \
                                     PIN_OSPEED_HIGH(GPIOE_CAM_D3) |        \
                                     PIN_OSPEED_HIGH(GPIOE_PULSE_2) |       \
                                     PIN_OSPEED_HIGH(GPIOE_PULSE_3) |       \
                                     PIN_OSPEED_HIGH(GPIOE_CAM_D4) |        \
                                     PIN_OSPEED_HIGH(GPIOE_CAM_D6) |        \
                                     PIN_OSPEED_HIGH(GPIOE_CAM_D7) |        \
                                     PIN_OSPEED_HIGH(GPIOE_SD_DETECT) |     \
                                     PIN_OSPEED_HIGH(GPIOE_MOT_L_IN2) |     \
                                     PIN_OSPEED_HIGH(GPIOE_MOT_L_IN1) |     \
                                     PIN_OSPEED_HIGH(GPIOE_MOT_L_IN4) |     \
                                     PIN_OSPEED_HIGH(GPIOE_MOT_L_IN3) |     \
                                     PIN_OSPEED_HIGH(GPIOE_MOT_R_IN2) |     \
                                     PIN_OSPEED_HIGH(GPIOE_MOT_R_IN1) |     \
                                     PIN_OSPEED_HIGH(GPIOE_MOT_R_IN3) |     \
                                     PIN_OSPEED_HIGH(GPIOE_MOT_R_IN4))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_CAM_D2) |       \
                                     PIN_PUPDR_PULLUP(GPIOE_CAM_D3) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PULSE_2) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_PULSE_3) |    \
                                     PIN_PUPDR_PULLUP(GPIOE_CAM_D4) |       \
                                     PIN_PUPDR_PULLUP(GPIOE_CAM_D6) |       \
                                     PIN_PUPDR_PULLUP(GPIOE_CAM_D7) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_SD_DETECT) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_MOT_L_IN2) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_MOT_L_IN1) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_MOT_L_IN4) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_MOT_L_IN3) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_MOT_R_IN2) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_MOT_R_IN1) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_MOT_R_IN3) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_MOT_R_IN4))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_CAM_D2) |           \
                                     PIN_ODR_HIGH(GPIOE_CAM_D3) |           \
                                     PIN_ODR_LOW(GPIOE_PULSE_2) |           \
                                     PIN_ODR_LOW(GPIOE_PULSE_3) |           \
                                     PIN_ODR_HIGH(GPIOE_CAM_D4) |           \
                                     PIN_ODR_HIGH(GPIOE_CAM_D6) |           \
                                     PIN_ODR_HIGH(GPIOE_CAM_D7) |           \
                                     PIN_ODR_HIGH(GPIOE_SD_DETECT) |        \
                                     PIN_ODR_LOW(GPIOE_MOT_L_IN2) |         \
                                     PIN_ODR_LOW(GPIOE_MOT_L_IN1) |         \
                                     PIN_ODR_LOW(GPIOE_MOT_L_IN4) |         \
                                     PIN_ODR_LOW(GPIOE_MOT_L_IN3) |         \
                                     PIN_ODR_LOW(GPIOE_MOT_R_IN2) |         \
                                     PIN_ODR_LOW(GPIOE_MOT_R_IN1) |         \
                                     PIN_ODR_LOW(GPIOE_MOT_R_IN3) |         \
                                     PIN_ODR_LOW(GPIOE_MOT_R_IN4))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_CAM_D2, 13U) |       \
                                     PIN_AFIO_AF(GPIOE_CAM_D3, 13U) |       \
                                     PIN_AFIO_AF(GPIOE_PULSE_2, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_PULSE_3, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_CAM_D4, 13U) |       \
                                     PIN_AFIO_AF(GPIOE_CAM_D6, 13U) |       \
                                     PIN_AFIO_AF(GPIOE_CAM_D7, 13U) |       \
                                     PIN_AFIO_AF(GPIOE_SD_DETECT, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_MOT_L_IN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_MOT_L_IN1, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_MOT_L_IN4, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_MOT_L_IN3, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_MOT_R_IN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_MOT_R_IN1, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_MOT_R_IN3, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_MOT_R_IN4, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (input floating).
 * PF1  - PIN1                      (input floating).
 * PF2  - PIN2                      (input floating).
 * PF3  - PIN3                      (input floating).
 * PF4  - PIN4                      (input floating).
 * PF5  - PIN5                      (input floating).
 * PF6  - PIN6                      (input floating).
 * PF7  - PIN7                      (input floating).
 * PF8  - PIN8                      (input floating).
 * PF9  - PIN9                      (input floating).
 * PF10 - PIN10                     (input floating).
 * PF11 - PIN11                     (input floating).
 * PF12 - PIN12                     (input floating).
 * PF13 - PIN13                     (input floating).
 * PF14 - PIN14                     (input floating).
 * PF15 - PIN15                     (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (input floating).
 * PG1  - PIN1                      (input floating).
 * PG2  - PIN2                      (input floating).
 * PG3  - PIN3                      (input floating).
 * PG4  - PIN4                      (input floating).
 * PG5  - PIN5                      (input floating).
 * PG6  - PIN6                      (input floating).
 * PG7  - PIN7                      (input floating).
 * PG8  - PIN8                      (input floating).
 * PG9  - PIN9                      (input floating).
 * PG10 - PIN10                     (input floating).
 * PG11 - PIN11                     (input floating).
 * PG12 - PIN12                     (input floating).
 * PG13 - PIN13                     (input floating).
 * PG14 - PIN14                     (input floating).
 * PG15 - PIN15                     (input floating).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (input floating).
 * PH3  - PIN3                      (input floating).
 * PH4  - PIN4                      (input floating).
 * PH5  - PIN5                      (input floating).
 * PH6  - PIN6                      (input floating).
 * PH7  - PIN7                      (input floating).
 * PH8  - PIN8                      (input floating).
 * PH9  - PIN9                      (input floating).
 * PH10 - PIN10                     (input floating).
 * PH11 - PIN11                     (input floating).
 * PH12 - PIN12                     (input floating).
 * PH13 - PIN13                     (input floating).
 * PH14 - PIN14                     (input floating).
 * PH15 - PIN15                     (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_HIGH(GPIOH_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input floating).
 * PI1  - PIN1                      (input floating).
 * PI2  - PIN2                      (input floating).
 * PI3  - PIN3                      (input floating).
 * PI4  - PIN4                      (input floating).
 * PI5  - PIN5                      (input floating).
 * PI6  - PIN6                      (input floating).
 * PI7  - PIN7                      (input floating).
 * PI8  - PIN8                      (input floating).
 * PI9  - PIN9                      (input floating).
 * PI10 - PIN10                     (input floating).
 * PI11 - PIN11                     (input floating).
 * PI12 - PIN12                     (input floating).
 * PI13 - PIN13                     (input floating).
 * PI14 - PIN14                     (input floating).
 * PI15 - PIN15                     (input floating).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_HIGH(GPIOI_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
