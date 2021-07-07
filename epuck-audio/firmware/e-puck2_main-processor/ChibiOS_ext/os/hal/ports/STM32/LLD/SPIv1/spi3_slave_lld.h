/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

/**
 * @file    STM32/SPIv1/spi_lld.h
 * @brief   STM32 SPI subsystem low level driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef _SPI3_SLAVE_LLD_H_
#define _SPI3_SLAVE_LLD_H_

#include "spi3_slave.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   SPI3 driver enable switch.
 * @details If set to @p TRUE the support for SPI3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI3_SLAVE) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI3_SLAVE                  FALSE
#endif

/**
 * @brief   SPI3 interrupt priority level setting.
 */
#if !defined(STM32_SPI_SPI3_SLAVE_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI3_SLAVE_IRQ_PRIORITY         10
#endif

/**
 * @brief   SPI3 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_SPI_SPI3_SLAVE_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI3_SLAVE_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI DMA error hook.
 */
#if !defined(STM32_SPI_SLAVE_DMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define STM32_SPI_SLAVE_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_SPI_USE_SPI3_SLAVE && !STM32_HAS_SPI3
#error "SPI3 slave not present in the selected device"
#endif

#if !STM32_SPI_USE_SPI3_SLAVE
#error "SPI driver activated but no SPI peripheral assigned"
#endif

#if STM32_SPI_USE_SPI3_SLAVE &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_SPI_SPI3_SLAVE_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI3 slave"
#endif

#if STM32_SPI_USE_SPI3_SLAVE &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_SPI_SPI3_SLAVE_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI3"
#endif

/* The following checks are only required when there is a DMA able to
   reassign streams to different channels.*/
#if STM32_ADVANCED_DMA
/* Check on the presence of the DMA streams settings in mcuconf.h.*/
#if STM32_SPI_USE_SPI3_SLAVE && (!defined(STM32_SPI_SPI3_SLAVE_RX_DMA_STREAM) ||        \
                           !defined(STM32_SPI_SPI3_SLAVE_TX_DMA_STREAM))
#error "SPI3 slave DMA streams not defined"
#endif

/* Check on the validity of the assigned DMA channels.*/
#if STM32_SPI_USE_SPI3_SLAVE &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI3_SLAVE_RX_DMA_STREAM, STM32_SPI3_RX_DMA_MSK)
#error "invalid DMA stream associated to SPI3 RX"
#endif

#if STM32_SPI_USE_SPI3_SLAVE &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI3_SLAVE_TX_DMA_STREAM, STM32_SPI3_TX_DMA_MSK)
#error "invalid DMA stream associated to SPI3 TX"
#endif
#endif /* STM32_ADVANCED_DMA */

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/


/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_SPI_USE_SPI3_SLAVE && !defined(__DOXYGEN__)
extern SPISlaveDriver SPISLAVED3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi3_slave_lld_init(void);
  void spi3_slave_lld_start(SPISlaveDriver *spip);
  void spi3_slave_lld_stop(SPISlaveDriver *spip);
  void spi3_slave_lld_ignore(SPISlaveDriver *spip, size_t n);
  void spi3_slave_lld_exchange(SPISlaveDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void spi3_slave_lld_stop_exchange(SPISlaveDriver *spip);
  void spi3_slave_lld_send(SPISlaveDriver *spip, size_t n, const void *txbuf);
  void spi3_slave_lld_receive(SPISlaveDriver *spip, size_t n, void *rxbuf);
  uint16_t spi3_slave_lld_polled_exchange(SPISlaveDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* _SPI3_SLAVE_LLD_H_ */

/** @} */
