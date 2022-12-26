/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics STM32F429I-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F429I_DISCOVERY
#define BOARD_NAME                  "HY21008"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F429xx

/*
 * IO pins assignments.
 */
#define GPIOA_UART4_TX              0U
#define GPIOA_UART4_RX              1U
#define GPIOA_UART2_TX              2U
#define GPIOA_UART2_RX              3U
#define GPIOA_SPI1_NSS              4U
#define GPIOA_SPI1_SCK              5U
#define GPIOA_SPI1_MISO             6U
#define GPIOA_SPI1_MOSI             7U
#define GPIOA_IDO_7                 8U
#define GPIOA_UART1_TX              9U
#define GPIOA_UART1_RX              10U
#define GPIOA_CAN1_RX               11U
#define GPIOA_CAN1_TX               12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_IDO_2                 15U

#define GPIOB_AD2S_A1_12            0U
#define GPIOB_AD2S_SMP_34           1U
#define GPIOB_BOOT1                 2U
#define GPIOB_SWO                   3U
#define GPIOB_IDI_3                 4U
#define GPIOB_CAN2_RX               5U
#define GPIOB_CAN2_TX               6U
#define GPIOB_7                     7U
#define GPIOB_IDI_2                 8U
#define GPIOB_IDI_1                 9U
#define GPIOB_AD2S_SMP_12           10U
#define GPIOB_AD2S_RST_34           11U
#define GPIOB_SPI2_NSS              12U
#define GPIOB_SPI2_SCK              13U
#define GPIOB_SPI2_MISO             14U
#define GPIOB_SPI2_MOSI             15U

#define GPIOC_FMC_SDNWE                 0U
#define GPIOC_AD76_STBY             1U
#define GPIOC_AD76_OS2              2U
#define GPIOC_AD76_OS1              3U
#define GPIOC_AD2S_WR2              4U
#define GPIOC_AD2S_A0_12            5U
#define GPIOC_UART6_TX              6U
#define GPIOC_UART6_RX              7U
#define GPIOC_IDO_5                 8U
#define GPIOC_IDO_6                 9U
#define GPIOC_IDO_1                 10U
#define GPIOC_IDO_0                 11U
#define GPIOC_IDI_7                 12U
#define GPIOC_AD57_RST              13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_FMC_D2                0U
#define GPIOD_FMC_D3                1U
#define GPIOD_IDI_6                 2U
#define GPIOD_FMC_CLK               3U
#define GPIOD_FMC_NOE               4U
#define GPIOD_FMC_NWE               5U
#define GPIOD_FMC_NWAIT             6U
#define GPIOD_FMC_NE1               7U
#define GPIOD_FMC_D13               8U
#define GPIOD_FMC_D14               9U
#define GPIOD_FMC_D15               10U
#define GPIOD_FMC_A16               11U
#define GPIOD_FMC_A17               12U
#define GPIOD_FMC_A18               13U
#define GPIOD_FMC_D0                14U
#define GPIOD_FMC_D1                15U

#define GPIOE_FMC_NBL0              0U
#define GPIOE_FMC_NBL1              1U
#define GPIOE_SPI4_SCK              2U
#define GPIOE_FMC_A19               3U
#define GPIOE_SPI4_NSS              4U
#define GPIOE_SPI4_MISO             5U
#define GPIOE_SPI4_MOSI             6U
#define GPIOE_FMC_D4                7U
#define GPIOE_FMC_D5                8U
#define GPIOE_FMC_D6                9U
#define GPIOE_FMC_D7                10U
#define GPIOE_FMC_D8                11U
#define GPIOE_FMC_D9                12U
#define GPIOE_FMC_D10               13U
#define GPIOE_FMC_D11               14U
#define GPIOE_FMC_D12               15U

#define GPIOF_FMC_A0                0U
#define GPIOF_FMC_A1                1U
#define GPIOF_FMC_A2                2U
#define GPIOF_FMC_A3                3U
#define GPIOF_FMC_A4                4U
#define GPIOF_FMC_A5                5U
#define GPIOF_SPI5_NSS              6U
#define GPIOF_SPI5_SCK              7U
#define GPIOF_SPI5_MISO             8U
#define GPIOF_SPI5_MOSI             9U
#define GPIOF_AD76_CONV_A           10U
#define GPIOF_FMC_SDNRAS            11U
#define GPIOF_FMC_A6                12U
#define GPIOF_FMC_A7                13U
#define GPIOF_FMC_A8                14U
#define GPIOF_FMC_A9                15U

#define GPIOG_FMC_A10               0U
#define GPIOG_FMC_A11               1U
#define GPIOG_FMC_A12               2U
#define GPIOG_FMC_A13               3U
#define GPIOG_FMC_BA0               4U
#define GPIOG_FMC_BA1               5U
#define GPIOG_IDO_15                6U
#define GPIOG_IDO_14                7U
#define GPIOG_FMC_SDCLK             8U
#define GPIOG_FMC_NE2               9U
#define GPIOG_FMC_NE3               10U
#define GPIOG_IDI_5                 11U
#define GPIOG_FMC_NE4               12U
#define GPIOG_IDO_POWER_EN          13U
#define GPIOG_IDI_4                 14U
#define GPIOG_FMC_SDNCAS            15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_AD76_OS0              2U
#define GPIOH_AD2S_WR4              3U
#define GPIOH_I2C2_SCL              4U
#define GPIOH_I2C2_SDA              5U
#define GPIOH_SD_NE1                6U
#define GPIOH_SDCKE1                7U
#define GPIOH_AD2S_RST_12           8U
#define GPIOH_AD2S_A0_23            9U
#define GPIOH_AD2S_A1_23            10U
#define GPIOH_LED_ACT               11U
#define GPIOH_IDO_13                12U
#define GPIOH_IDO_12                13U
#define GPIOH_IDO_11                14U
#define GPIOH_IDO_10                15U

#define GPIOI_IDO_9                 0U
#define GPIOI_IDO_8                 1U
#define GPIOI_IDO_4                 2U
#define GPIOI_IDO_3                 3U
#define GPIOI_IDI_0                 4U
#define GPIOI_AD76_DOUTB            5U
#define GPIOI_AD57_BIN              6U
#define GPIOI_AD57_LATCH            7U
#define GPIOI_AD76_BUSY             8U
#define GPIOI_AD76_FSTDATA          9U
#define GPIOI_AD76_RST              10U
#define GPIOI_AD76_CONV_B           11U
#define GPIOI_12                    12U
#define GPIOI_13                    13U
#define GPIOI_14                    14U
#define GPIOI_15                    15U

/*
 * IO lines assignments.
 */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

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

 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_UART4_TX)  |      \
                                     PIN_MODE_ALTERNATE(GPIOA_UART4_RX)  |      \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2_TX)  |      \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2_RX)  |      \
                                     PIN_MODE_OUTPUT   (GPIOA_SPI1_NSS)  |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK)  |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) |      \
                                     PIN_MODE_OUTPUT   (GPIOA_IDO_7)     |      \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_TX)  |      \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RX)  |      \
                                     PIN_MODE_ALTERNATE(GPIOA_CAN1_RX)   |      \
                                     PIN_MODE_ALTERNATE(GPIOA_CAN1_TX)   |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO)     |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK)     |      \
                                     PIN_MODE_OUTPUT   (GPIOA_IDO_2))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_UART4_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART4_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_NSS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_IDO_7) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN1_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN1_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_IDO_2))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_UART4_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_UART4_RX) |  \
                                     PIN_OSPEED_HIGH(GPIOA_UART2_TX) |  \
                                     PIN_OSPEED_HIGH(GPIOA_UART2_RX) |        \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_NSS) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_SCK) |       \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MISO) |        \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MOSI) |    \
                                     PIN_OSPEED_HIGH(GPIOA_IDO_7) |      \
                                     PIN_OSPEED_HIGH(GPIOA_UART1_TX) |    \
                                     PIN_OSPEED_HIGH(GPIOA_UART1_RX) |    \
                                     PIN_OSPEED_HIGH(GPIOA_CAN1_RX) |        \
                                     PIN_OSPEED_HIGH(GPIOA_CAN1_TX) |        \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_LOW(GPIOA_IDO_2))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_UART4_TX) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_UART4_RX) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_TX) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_RX) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SPI1_NSS) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SPI1_SCK) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SPI1_MISO) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SPI1_MOSI) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_IDO_7) |   \
                                     PIN_PUPDR_PULLUP(GPIOA_UART1_TX) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_UART1_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_CAN1_RX) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_CAN1_TX) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_IDO_2))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_UART4_TX) |           \
                                     PIN_ODR_HIGH(GPIOA_UART4_RX) |        \
                                     PIN_ODR_HIGH(GPIOA_UART2_TX) |        \
                                     PIN_ODR_HIGH(GPIOA_UART2_RX) |           \
                                     PIN_ODR_HIGH(GPIOA_SPI1_NSS) |        \
                                     PIN_ODR_HIGH(GPIOA_SPI1_SCK) |             \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO) |           \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI) |          \
                                     PIN_ODR_HIGH(GPIOA_IDO_7) |         \
                                     PIN_ODR_HIGH(GPIOA_UART1_TX) |          \
                                     PIN_ODR_HIGH(GPIOA_UART1_RX) |          \
                                     PIN_ODR_HIGH(GPIOA_CAN1_RX) |           \
                                     PIN_ODR_HIGH(GPIOA_CAN1_TX) |           \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_IDO_2))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_UART4_TX, 8U) |        \
                                     PIN_AFIO_AF(GPIOA_UART4_RX, 8U) |     \
                                     PIN_AFIO_AF(GPIOA_UART2_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOA_UART2_RX, 7U) |       \
                                     PIN_AFIO_AF(GPIOA_SPI1_NSS, 0U) |    \
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK, 5U) |          \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO, 5U) |       \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI, 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_IDO_7, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_UART1_TX, 7U) |       \
                                     PIN_AFIO_AF(GPIOA_UART1_RX, 7U) |       \
                                     PIN_AFIO_AF(GPIOA_CAN1_RX, 9U) |       \
                                     PIN_AFIO_AF(GPIOA_CAN1_TX, 9U) |       \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_IDO_2, 0U))

/*
 * GPIOB setup:
 *
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_AD2S_A1_12) |     \
                                     PIN_MODE_OUTPUT(GPIOB_AD2S_SMP_34) |     \
                                     PIN_MODE_INPUT(GPIOB_BOOT1) |          \
                                     PIN_MODE_ALTERNATE(GPIOB_SWO) |        \
                                     PIN_MODE_INPUT(GPIOB_IDI_3) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN2_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN2_TX) |  \
                                     PIN_MODE_INPUT(GPIOB_7) |           \
                                     PIN_MODE_INPUT(GPIOB_IDI_2) |     \
                                     PIN_MODE_INPUT(GPIOB_IDI_1) |     \
                                     PIN_MODE_OUTPUT(GPIOB_AD2S_SMP_12) |     \
                                     PIN_MODE_OUTPUT(GPIOB_AD2S_RST_34) |     \
                                     PIN_MODE_OUTPUT(GPIOB_SPI2_NSS) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_AD2S_A1_12) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AD2S_SMP_34) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IDI_3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN2_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN2_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IDI_2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IDI_1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AD2S_SMP_12) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AD2S_RST_34) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_NSS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_SCK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_AD2S_A1_12) |        \
                                     PIN_OSPEED_HIGH(GPIOB_AD2S_SMP_34) |        \
                                     PIN_OSPEED_HIGH(GPIOB_BOOT1) |         \
                                     PIN_OSPEED_HIGH(GPIOB_SWO) |           \
                                     PIN_OSPEED_VERYLOW(GPIOB_IDI_3) |       \
                                     PIN_OSPEED_HIGH(GPIOB_CAN2_RX) |    \
                                     PIN_OSPEED_HIGH(GPIOB_CAN2_TX) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_7) |       \
                                     PIN_OSPEED_HIGH(GPIOB_IDI_2) |        \
                                     PIN_OSPEED_HIGH(GPIOB_IDI_1) |        \
                                     PIN_OSPEED_HIGH(GPIOB_AD2S_SMP_12) |        \
                                     PIN_OSPEED_HIGH(GPIOB_AD2S_RST_34) |        \
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_NSS) |     \
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_SCK) |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MISO) |     \
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_AD2S_A1_12) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_AD2S_SMP_34) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_BOOT1) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_IDI_3) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_CAN2_RX) | \
                                     PIN_PUPDR_PULLUP(GPIOB_CAN2_TX) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_7) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_IDI_2) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_IDI_1) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_AD2S_SMP_12) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_AD2S_RST_34) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI2_NSS) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI2_SCK) |\
                                     PIN_PUPDR_PULLUP(GPIOB_SPI2_MISO) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_AD2S_A1_12) |           \
                                     PIN_ODR_HIGH(GPIOB_AD2S_SMP_34) |           \
                                     PIN_ODR_HIGH(GPIOB_BOOT1) |            \
                                     PIN_ODR_HIGH(GPIOB_SWO) |              \
                                     PIN_ODR_HIGH(GPIOB_IDI_3) |             \
                                     PIN_ODR_HIGH(GPIOB_CAN2_RX) |       \
                                     PIN_ODR_HIGH(GPIOB_CAN2_TX) |        \
                                     PIN_ODR_HIGH(GPIOB_7) |             \
                                     PIN_ODR_HIGH(GPIOB_IDI_2) |           \
                                     PIN_ODR_HIGH(GPIOB_IDI_1) |           \
                                     PIN_ODR_HIGH(GPIOB_AD2S_SMP_12) |           \
                                     PIN_ODR_HIGH(GPIOB_AD2S_RST_34) |           \
                                     PIN_ODR_HIGH(GPIOB_SPI2_NSS) |        \
                                     PIN_ODR_HIGH(GPIOB_SPI2_SCK) |      \
                                     PIN_ODR_HIGH(GPIOB_SPI2_MISO) |        \
                                     PIN_ODR_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_AD2S_A1_12, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_AD2S_SMP_34, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_BOOT1, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_SWO, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_IDI_3, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_CAN2_RX, 9U) |   \
                                     PIN_AFIO_AF(GPIOB_CAN2_TX, 9U) |    \
                                     PIN_AFIO_AF(GPIOB_7, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_IDI_2, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_IDI_1, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_AD2S_SMP_12, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_AD2S_RST_34, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_SPI2_NSS, 0) |    \
                                     PIN_AFIO_AF(GPIOB_SPI2_SCK, 5U) |   \
                                     PIN_AFIO_AF(GPIOB_SPI2_MISO, 5U) |    \
                                     PIN_AFIO_AF(GPIOB_SPI2_MOSI, 5U))

/*
 * GPIOC setup:
 *
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ALTERNATE(GPIOC_FMC_SDNWE) |  \
                                     PIN_MODE_OUTPUT(GPIOC_AD76_STBY) |  \
                                     PIN_MODE_OUTPUT(GPIOC_AD76_OS2) |   \
                                     PIN_MODE_OUTPUT(GPIOC_AD76_OS1) |           \
                                     PIN_MODE_OUTPUT(GPIOC_AD2S_WR2) |    \
                                     PIN_MODE_OUTPUT(GPIOC_AD2S_A0_12) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_UART6_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_UART6_RX) |     \
                                     PIN_MODE_OUTPUT(GPIOC_IDO_5) |           \
                                     PIN_MODE_OUTPUT(GPIOC_IDO_6) |   \
                                     PIN_MODE_OUTPUT(GPIOC_IDO_1) |     \
                                     PIN_MODE_OUTPUT(GPIOC_IDO_0) |          \
                                     PIN_MODE_INPUT(GPIOC_IDI_7) |          \
                                     PIN_MODE_OUTPUT(GPIOC_AD57_RST) |          \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_FMC_SDNWE) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD76_STBY) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD76_OS2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD76_OS1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD2S_WR2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD2S_A0_12) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_UART6_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_UART6_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_IDO_5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_IDO_6) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_IDO_1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_IDO_0) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_IDI_7) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD57_RST) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_FMC_SDNWE) |     \
                                     PIN_OSPEED_HIGH(GPIOC_AD76_STBY) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_AD76_OS2) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_AD76_OS1) |       \
                                     PIN_OSPEED_HIGH(GPIOC_AD2S_WR2) |    \
                                     PIN_OSPEED_HIGH(GPIOC_AD2S_A0_12) |     \
                                     PIN_OSPEED_HIGH(GPIOC_UART6_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOC_UART6_RX) |        \
                                     PIN_OSPEED_VERYLOW(GPIOC_IDO_5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_IDO_6) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_IDO_1) |        \
                                     PIN_OSPEED_VERYLOW(GPIOC_IDO_0) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_IDI_7) |      \
                                     PIN_OSPEED_HIGH(GPIOC_AD57_RST) |      \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_IN) |      \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_FMC_SDNWE) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_AD76_STBY) |\
                                     PIN_PUPDR_PULLUP(GPIOC_AD76_OS2) |\
                                     PIN_PUPDR_PULLUP(GPIOC_AD76_OS1) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_AD2S_WR2) | \
                                     PIN_PUPDR_PULLUP(GPIOC_AD2S_A0_12) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_UART6_TX) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_UART6_RX) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOC_IDO_5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_IDO_6) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOC_IDO_1) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_IDO_0) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_IDI_7) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_AD57_RST) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_FMC_SDNWE) |        \
                                     PIN_ODR_HIGH(GPIOC_AD76_STBY) |     \
                                     PIN_ODR_HIGH(GPIOC_AD76_OS2) |      \
                                     PIN_ODR_HIGH(GPIOC_AD76_OS1) |             \
                                     PIN_ODR_HIGH(GPIOC_AD2S_WR2) |       \
                                     PIN_ODR_HIGH(GPIOC_AD2S_A0_12) |        \
                                     PIN_ODR_HIGH(GPIOC_UART6_TX) |        \
                                     PIN_ODR_HIGH(GPIOC_UART6_RX) |           \
                                     PIN_ODR_HIGH(GPIOC_IDO_5) |             \
                                     PIN_ODR_HIGH(GPIOC_IDO_6) |         \
                                     PIN_ODR_HIGH(GPIOC_IDO_1) |           \
                                     PIN_ODR_HIGH(GPIOC_IDO_0) |            \
                                     PIN_ODR_HIGH(GPIOC_IDI_7) |            \
                                     PIN_ODR_HIGH(GPIOC_AD57_RST) |            \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_FMC_SDNWE, 12U) |    \
                                     PIN_AFIO_AF(GPIOC_AD76_STBY, 0U) |  \
                                     PIN_AFIO_AF(GPIOC_AD76_OS2, 0U) |   \
                                     PIN_AFIO_AF(GPIOC_AD76_OS1, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_AD2S_WR2, 0U) |    \
                                     PIN_AFIO_AF(GPIOC_AD2S_A0_12, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_UART6_TX, 8U) |    \
                                     PIN_AFIO_AF(GPIOC_UART6_RX, 8U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_IDO_5, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_IDO_6, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_IDO_1, 0U) |       \
                                     PIN_AFIO_AF(GPIOC_IDO_0, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_IDI_7, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_AD57_RST, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_FMC_D2) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D3) |     \
                                     PIN_MODE_INPUT(GPIOD_IDI_6) |           \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_CLK) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_NOE) |           \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_NWE) |           \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_NWAIT) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_NE1) |           \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D13) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D14) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D15) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_A16) |         \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_A17) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_A18) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D0) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D1))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_FMC_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_IDI_6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_CLK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_NOE) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_NWE) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_NWAIT) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_NE1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D14) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D15) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_A16) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_A17) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_A18) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D1))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_FMC_D2) |        \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D3) |        \
                                     PIN_OSPEED_VERYLOW(GPIOD_IDI_6) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_CLK) |        \
                                     PIN_OSPEED_VERYLOW(GPIOD_FMC_NOE) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_FMC_NWE) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_NWAIT) |        \
                                     PIN_OSPEED_VERYLOW(GPIOD_FMC_NE1) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D13) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D14) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D15) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_A16) |        \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_A17) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_A18) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D0) |        \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D1))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_FMC_D2) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D3) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_IDI_6) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_CLK) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_FMC_NOE) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_FMC_NWE) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_NWAIT) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_FMC_NE1) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_FMC_D13) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_FMC_D14) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_FMC_D15) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_FMC_A16) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_A17) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_A18) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D0) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D1))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_FMC_D2) |           \
                                     PIN_ODR_HIGH(GPIOD_FMC_D3) |           \
                                     PIN_ODR_HIGH(GPIOD_IDI_6) |             \
                                     PIN_ODR_HIGH(GPIOD_FMC_CLK) |           \
                                     PIN_ODR_HIGH(GPIOD_FMC_NOE) |             \
                                     PIN_ODR_HIGH(GPIOD_FMC_NWE) |             \
                                     PIN_ODR_HIGH(GPIOD_FMC_NWAIT) |           \
                                     PIN_ODR_HIGH(GPIOD_FMC_NE1) |             \
                                     PIN_ODR_HIGH(GPIOD_FMC_D13) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_D14) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_D15) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_A16) |           \
                                     PIN_ODR_HIGH(GPIOD_FMC_A17) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_A18) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_D0) |           \
                                     PIN_ODR_HIGH(GPIOD_FMC_D1))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_FMC_D2, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D3, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_IDI_6, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_FMC_CLK, 14U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_NOE, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_FMC_NWE, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_FMC_NWAIT, 14U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_NE1, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_FMC_D13, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_FMC_D14, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_FMC_D15, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_FMC_A16, 0U) |        \
                                     PIN_AFIO_AF(GPIOD_FMC_A17, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_A18, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D0, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D1, 12U))

/*
 * GPIOE setup:
 *
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_FMC_NBL0) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_NBL1) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_SCK) |           \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_A19) |           \
                                     PIN_MODE_OUTPUT(GPIOE_SPI4_NSS) |           \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_MISO) |           \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_MOSI) |           \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D4) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D5) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D6) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D7) |     \
                                     PIN_MODE_OUTPUT(GPIOE_FMC_D8) |     \
                                     PIN_MODE_OUTPUT(GPIOE_FMC_D9) |     \
                                     PIN_MODE_OUTPUT(GPIOE_FMC_D10) |    \
                                     PIN_MODE_OUTPUT(GPIOE_FMC_D11) |    \
                                     PIN_MODE_OUTPUT(GPIOE_FMC_D12))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_FMC_NBL0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_NBL1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_SCK) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_A19) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_NSS) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_MISO) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_MOSI) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D11) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D12))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_FMC_NBL0) |      \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_NBL1) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SPI4_SCK) |       \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_A19) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SPI4_NSS) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SPI4_MISO) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SPI4_MOSI) |       \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D4) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D5) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D6) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D7) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D8) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D9) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D10) |       \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D11) |       \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D12))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_FMC_NBL0) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_NBL1) |   \
                                     PIN_PUPDR_PULLUP(GPIOE_SPI4_SCK) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_FMC_A19) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_SPI4_NSS) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_SPI4_MISO) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_SPI4_MOSI) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D4) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D5) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D6) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D7) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D8) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D9) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D10) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D11) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D12))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_FMC_NBL0) |         \
                                     PIN_ODR_HIGH(GPIOE_FMC_NBL1) |         \
                                     PIN_ODR_HIGH(GPIOE_SPI4_SCK) |             \
                                     PIN_ODR_HIGH(GPIOE_FMC_A19) |             \
                                     PIN_ODR_HIGH(GPIOE_SPI4_NSS) |             \
                                     PIN_ODR_HIGH(GPIOE_SPI4_MISO) |             \
                                     PIN_ODR_HIGH(GPIOE_SPI4_MOSI) |             \
                                     PIN_ODR_HIGH(GPIOE_FMC_D4) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D5) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D6) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D7) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D8) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D9) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D10) |          \
                                     PIN_ODR_HIGH(GPIOE_FMC_D11) |          \
                                     PIN_ODR_HIGH(GPIOE_FMC_D12))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_FMC_NBL0, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_FMC_NBL1, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_SPI4_SCK, 5U) |          \
                                     PIN_AFIO_AF(GPIOE_FMC_A19, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_SPI4_NSS, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_SPI4_MISO, 5U) |          \
                                     PIN_AFIO_AF(GPIOE_SPI4_MOSI, 5U) |          \
                                     PIN_AFIO_AF(GPIOE_FMC_D4, 12U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_FMC_D5, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D6, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D7, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D8, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D9, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D10, 12U) |      \
                                     PIN_AFIO_AF(GPIOE_FMC_D11, 12U) |      \
                                     PIN_AFIO_AF(GPIOE_FMC_D12, 12U))

/*
 * GPIOF setup:
 *
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_FMC_A0) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A1) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A2) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A3) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A4) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A5) |     \
                                     PIN_MODE_OUTPUT(GPIOF_SPI5_NSS) |           \
                                     PIN_MODE_ALTERNATE(GPIOF_SPI5_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOF_SPI5_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOF_SPI5_MOSI) |  \
                                     PIN_MODE_OUTPUT(GPIOF_AD76_CONV_A) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_SDNRAS) | \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A6) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A7) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A8) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A9))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_FMC_A0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SPI5_NSS) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SPI5_SCK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SPI5_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SPI5_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_AD76_CONV_A) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_SDNRAS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A9))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_FMC_A0) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A1) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A2) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A3) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A4) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A5) |        \
                                     PIN_OSPEED_HIGH(GPIOF_SPI5_NSS) |       \
                                     PIN_OSPEED_HIGH(GPIOF_SPI5_SCK) |       \
                                     PIN_OSPEED_HIGH(GPIOF_SPI5_MISO) |     \
                                     PIN_OSPEED_HIGH(GPIOF_SPI5_MOSI) |     \
                                     PIN_OSPEED_HIGH(GPIOF_AD76_CONV_A) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_SDNRAS) |    \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A6) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A7) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A8) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A9))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_FMC_A0) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A1) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A2) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A3) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A4) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A5) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOF_SPI5_NSS) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_SPI5_SCK) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOF_SPI5_MISO) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOF_SPI5_MOSI) |  \
                                     PIN_PUPDR_FLOATING(GPIOF_AD76_CONV_A) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_SDNRAS) | \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A6) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A7) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A8) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A9))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_FMC_A0) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A1) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A2) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A3) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A4) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A5) |           \
                                     PIN_ODR_HIGH(GPIOF_SPI5_NSS) |             \
                                     PIN_ODR_HIGH(GPIOF_SPI5_SCK) |          \
                                     PIN_ODR_HIGH(GPIOF_SPI5_MISO) |        \
                                     PIN_ODR_HIGH(GPIOF_SPI5_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOF_AD76_CONV_A) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_SDNRAS) |       \
                                     PIN_ODR_HIGH(GPIOF_FMC_A6) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A7) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A8) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A9))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_FMC_A0, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A1, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A2, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A3, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A4, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A5, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_SPI5_NSS, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_SPI5_SCK, 5U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_SPI5_MISO, 5U) |     \
                                     PIN_AFIO_AF(GPIOF_SPI5_MOSI, 5U) |     \
                                     PIN_AFIO_AF(GPIOF_AD76_CONV_A, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_SDNRAS, 12U) |   \
                                     PIN_AFIO_AF(GPIOF_FMC_A6, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A7, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A8, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A9, 12U))

/*
 * GPIOG setup:
 *
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ALTERNATE(GPIOG_FMC_A10) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_A11) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_A12) |           \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_A13) |           \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_BA0) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_BA1) |    \
                                     PIN_MODE_OUTPUT(GPIOG_IDO_15) |     \
                                     PIN_MODE_OUTPUT(GPIOG_IDO_14) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_SDCLK) |  \
                                     PIN_MODE_OUTPUT(GPIOG_FMC_NE2) |           \
                                     PIN_MODE_OUTPUT(GPIOG_FMC_NE3) |     \
                                     PIN_MODE_INPUT(GPIOG_IDI_5) |     \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_NE4) |     \
                                     PIN_MODE_OUTPUT(GPIOG_IDO_POWER_EN) |    \
                                     PIN_MODE_INPUT(GPIOG_IDI_4) |      \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_FMC_A10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_A11) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_A12) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_A13) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_BA0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_BA1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_IDO_15) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_IDO_14) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_SDCLK) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_NE2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_NE3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_IDI_5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_NE4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_IDO_POWER_EN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_IDI_4) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_FMC_A10) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_A11) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_FMC_A12) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_FMC_A13) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_BA0) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_BA1) |       \
                                     PIN_OSPEED_HIGH(GPIOG_IDO_15) |        \
                                     PIN_OSPEED_HIGH(GPIOG_IDO_14) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_SDCLK) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_FMC_NE2) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_NE3) |        \
                                     PIN_OSPEED_HIGH(GPIOG_IDI_5) |        \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_NE4) |        \
                                     PIN_OSPEED_HIGH(GPIOG_IDO_POWER_EN) |    \
                                     PIN_OSPEED_HIGH(GPIOG_IDI_4) |      \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_FMC_A10) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_A11) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_FMC_A12) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_FMC_A13) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_BA0) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_BA1) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOG_IDO_15) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOG_IDO_14) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_SDCLK) |  \
                                     PIN_PUPDR_PULLUP(GPIOG_FMC_NE2) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_NE3) |     \
                                     PIN_PUPDR_PULLUP(GPIOG_IDI_5) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_NE4) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOG_IDO_POWER_EN) | \
                                     PIN_PUPDR_PULLUP(GPIOG_IDI_4) |   \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_FMC_A10) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_A11) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_A12) |             \
                                     PIN_ODR_HIGH(GPIOG_FMC_A13) |             \
                                     PIN_ODR_HIGH(GPIOG_FMC_BA0) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_BA1) |          \
                                     PIN_ODR_HIGH(GPIOG_IDO_15) |           \
                                     PIN_ODR_HIGH(GPIOG_IDO_14) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_SDCLK) |        \
                                     PIN_ODR_HIGH(GPIOG_FMC_NE2) |             \
                                     PIN_ODR_HIGH(GPIOG_FMC_NE3) |           \
                                     PIN_ODR_HIGH(GPIOG_IDI_5) |           \
                                     PIN_ODR_HIGH(GPIOG_FMC_NE4) |           \
                                     PIN_ODR_LOW(GPIOG_IDO_POWER_EN) |        \
                                     PIN_ODR_LOW(GPIOG_IDI_4) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_FMC_A10, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_A11, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_A12, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_FMC_A13, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_FMC_BA0, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_BA1, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_IDO_15, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_IDO_14, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_FMC_SDCLK, 12U) |    \
                                     PIN_AFIO_AF(GPIOG_FMC_NE2, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_FMC_NE3, 14U) |       \
                                     PIN_AFIO_AF(GPIOG_IDI_5, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_FMC_NE4, 14U) |       \
                                     PIN_AFIO_AF(GPIOG_IDO_POWER_EN, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_IDI_4, 0U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_SDNCAS, 12U))

/*
 * GPIOH setup:
 *
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_AD76_OS0) |           \
                                     PIN_MODE_INPUT(GPIOH_AD2S_WR4) |           \
                                     PIN_MODE_ALTERNATE(GPIOH_I2C2_SCL) |           \
                                     PIN_MODE_ALTERNATE(GPIOH_I2C2_SDA) |           \
                                     PIN_MODE_INPUT(GPIOH_SD_NE1) |           \
                                     PIN_MODE_INPUT(GPIOH_SDCKE1) |           \
                                     PIN_MODE_OUTPUT(GPIOH_AD2S_RST_12) |           \
                                     PIN_MODE_OUTPUT(GPIOH_AD2S_A0_23) |           \
                                     PIN_MODE_OUTPUT(GPIOH_AD2S_A1_23) |          \
                                     PIN_MODE_OUTPUT(GPIOH_LED_ACT) |          \
                                     PIN_MODE_INPUT(GPIOH_IDO_13) |          \
                                     PIN_MODE_INPUT(GPIOH_IDO_12) |          \
                                     PIN_MODE_INPUT(GPIOH_IDO_11) |          \
                                     PIN_MODE_INPUT(GPIOH_IDO_10))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_AD76_OS0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_AD2S_WR4) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_I2C2_SCL) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_I2C2_SDA) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_SD_NE1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_SDCKE1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_AD2S_RST_12) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_AD2S_A0_23) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_AD2S_A1_23) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LED_ACT) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_IDO_13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_IDO_12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_IDO_11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_IDO_10))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_AD76_OS0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_AD2S_WR4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_I2C2_SCL) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_I2C2_SDA) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_SD_NE1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_SDCKE1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_AD2S_RST_12) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_AD2S_A0_23) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_AD2S_A1_23) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_LED_ACT) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_IDO_13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_IDO_12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_IDO_11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_IDO_10))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_AD76_OS0) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_AD2S_WR4) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_I2C2_SCL) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_I2C2_SDA) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_SD_NE1) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_SDCKE1) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_AD2S_RST_12) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_AD2S_A0_23) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_AD2S_A1_23) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_LED_ACT) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_IDO_13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_IDO_12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_IDO_11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_IDO_10))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_AD76_OS0) |             \
                                     PIN_ODR_HIGH(GPIOH_AD2S_WR4) |             \
                                     PIN_ODR_HIGH(GPIOH_I2C2_SCL) |             \
                                     PIN_ODR_HIGH(GPIOH_I2C2_SDA) |             \
                                     PIN_ODR_HIGH(GPIOH_SD_NE1) |             \
                                     PIN_ODR_HIGH(GPIOH_SDCKE1) |             \
                                     PIN_ODR_HIGH(GPIOH_AD2S_RST_12) |             \
                                     PIN_ODR_HIGH(GPIOH_AD2S_A0_23) |             \
                                     PIN_ODR_HIGH(GPIOH_AD2S_A1_23) |            \
                                     PIN_ODR_HIGH(GPIOH_LED_ACT) |            \
                                     PIN_ODR_HIGH(GPIOH_IDO_13) |            \
                                     PIN_ODR_HIGH(GPIOH_IDO_12) |            \
                                     PIN_ODR_HIGH(GPIOH_IDO_11) |            \
                                     PIN_ODR_HIGH(GPIOH_IDO_10))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_AD76_OS0, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_AD2S_WR4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_I2C2_SCL, 4U) |          \
                                     PIN_AFIO_AF(GPIOH_I2C2_SDA, 4U) |          \
                                     PIN_AFIO_AF(GPIOH_SD_NE1, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_SDCKE1, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_AD2S_RST_12, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_AD2S_A0_23, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_AD2S_A1_23, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_LED_ACT, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_IDO_13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_IDO_12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_IDO_11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_IDO_10, 0U))

/*
 * GPIOI setup:
 *
 */
#define VAL_GPIOI_MODER             (PIN_MODE_OUTPUT(GPIOI_IDO_9) |           \
                                     PIN_MODE_OUTPUT(GPIOI_IDO_8) |           \
                                     PIN_MODE_OUTPUT(GPIOI_IDO_4) |           \
                                     PIN_MODE_OUTPUT(GPIOI_IDO_3) |           \
                                     PIN_MODE_INPUT(GPIOI_IDI_0) |           \
                                     PIN_MODE_INPUT(GPIOI_AD76_DOUTB) |           \
                                     PIN_MODE_OUTPUT(GPIOI_AD57_BIN) |           \
                                     PIN_MODE_OUTPUT(GPIOI_AD57_LATCH) |           \
                                     PIN_MODE_INPUT(GPIOI_AD76_BUSY) |           \
                                     PIN_MODE_INPUT(GPIOI_AD76_FSTDATA) |           \
                                     PIN_MODE_OUTPUT(GPIOI_AD76_RST) |          \
                                     PIN_MODE_OUTPUT(GPIOI_AD76_CONV_B) |          \
                                     PIN_MODE_INPUT(GPIOI_12) |          \
                                     PIN_MODE_INPUT(GPIOI_13) |          \
                                     PIN_MODE_INPUT(GPIOI_14) |          \
                                     PIN_MODE_INPUT(GPIOI_15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_IDO_9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_IDO_8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_IDO_4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_IDO_3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_IDI_0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_AD76_DOUTB) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_AD57_BIN) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_AD57_LATCH) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_AD76_BUSY) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_AD76_FSTDATA) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_AD76_RST) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_AD76_CONV_B) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOI_IDO_9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_IDO_8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_IDO_4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_IDO_3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_IDI_0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_AD76_DOUTB) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_AD57_BIN) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_AD57_LATCH) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_AD76_BUSY) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_AD76_FSTDATA) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_AD76_RST) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_AD76_CONV_B) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_IDO_9) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_IDO_8) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_IDO_4) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_IDO_3) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_IDI_0) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_AD76_DOUTB) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_AD57_BIN) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_AD57_LATCH) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_AD76_BUSY) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_AD76_FSTDATA) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_AD76_RST) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_AD76_CONV_B) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_12) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_13) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_14) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_IDO_9) |             \
                                     PIN_ODR_HIGH(GPIOI_IDO_8) |             \
                                     PIN_ODR_HIGH(GPIOI_IDO_4) |             \
                                     PIN_ODR_HIGH(GPIOI_IDO_3) |             \
                                     PIN_ODR_HIGH(GPIOI_IDI_0) |             \
                                     PIN_ODR_HIGH(GPIOI_AD76_DOUTB) |             \
                                     PIN_ODR_LOW(GPIOI_AD57_BIN) |             \
                                     PIN_ODR_HIGH(GPIOI_AD57_LATCH) |             \
                                     PIN_ODR_HIGH(GPIOI_AD76_BUSY) |             \
                                     PIN_ODR_HIGH(GPIOI_AD76_FSTDATA) |             \
                                     PIN_ODR_HIGH(GPIOI_AD76_RST) |            \
                                     PIN_ODR_HIGH(GPIOI_AD76_CONV_B) |            \
                                     PIN_ODR_HIGH(GPIOI_12) |            \
                                     PIN_ODR_HIGH(GPIOI_13) |            \
                                     PIN_ODR_HIGH(GPIOI_14) |            \
                                     PIN_ODR_HIGH(GPIOI_15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_IDO_9, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_IDO_8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_IDO_4, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_IDO_3, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_IDI_0, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_AD76_DOUTB, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_AD57_BIN, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_AD57_LATCH, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_AD76_BUSY, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_AD76_FSTDATA, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_AD76_RST, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_AD76_CONV_B, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_13, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_14, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

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
