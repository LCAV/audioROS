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
 * @file    STM32/SPIv1/spi_lld.c
 * @brief   STM32 SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"
#include "spi3_slave_lld.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define SPI3_SLAVE_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI3_SLAVE_RX_DMA_STREAM,                        \
                       STM32_SPI3_RX_DMA_CHN)

#define SPI3_SLAVE_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI3_SLAVE_TX_DMA_STREAM,                        \
                       STM32_SPI3_TX_DMA_CHN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static uint16_t dummytx;
static uint16_t dummyrx;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared end-of-rx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi3_slave_lld_serve_rx_interrupt(SPISlaveDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_SPI_SLAVE_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_SPI_SLAVE_DMA_ERROR_HOOK(spip);
  }
#else
  (void)flags;
#endif

  /* Callbacks handling, note it is portable code defined in the high
     level driver.*/
  if ((flags & STM32_DMA_ISR_TCIF) != 0) {
    /* Transfer complete processing.*/
    _spi3_slave_isr_full_code(spip);
  }
  else if ((flags & STM32_DMA_ISR_HTIF) != 0) {
    /* Half transfer processing.*/
    _spi3_slave_isr_half_code(spip);
  }

}

/**
 * @brief   Shared end-of-tx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi3_slave_lld_serve_tx_interrupt(SPISlaveDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_SPI_SLAVE_DMA_ERROR_HOOK)
  (void)spip;
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_SPI_SLAVE_DMA_ERROR_HOOK(spip);
  }
#else
  (void)spip;
  (void)flags;
#endif

  /* Callbacks handling, note it is portable code defined in the high
     level driver.*/
  if ((flags & STM32_DMA_ISR_TCIF) != 0) {
    /* Transfer complete processing.*/
    _spi3_slave_isr_full_code(spip);
  }
  else if ((flags & STM32_DMA_ISR_HTIF) != 0) {
    /* Half transfer processing.*/
    _spi3_slave_isr_half_code(spip);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi3_slave_lld_init(void) {

  dummytx = 0xFFFF;

#if STM32_SPI_USE_SPI3_SLAVE
  spi3SlaveObjectInit(&SPISLAVED3);
  SPISLAVED3.spi       = SPI3;
  SPISLAVED3.dmarx     = STM32_DMA_STREAM(STM32_SPI_SPI3_SLAVE_RX_DMA_STREAM);
  SPISLAVED3.dmatx     = STM32_DMA_STREAM(STM32_SPI_SPI3_SLAVE_TX_DMA_STREAM);
  SPISLAVED3.rxdmamode = STM32_DMA_CR_CHSEL(SPI3_SLAVE_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI3_SLAVE_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_CIRC |
                    STM32_DMA_CR_HTIE |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPISLAVED3.txdmamode = STM32_DMA_CR_CHSEL(SPI3_SLAVE_TX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI3_SLAVE_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#endif

}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi3_slave_lld_start(SPISlaveDriver *spip) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_SLAVE_STOP) {
#if STM32_SPI_USE_SPI3_SLAVE
    if (&SPISLAVED3 == spip) {
      bool b;
      b = dmaStreamAllocate(spip->dmarx,
                            STM32_SPI_SPI3_SLAVE_IRQ_PRIORITY,
                            (stm32_dmaisr_t)spi3_slave_lld_serve_rx_interrupt,
                            (void *)spip);
      osalDbgAssert(!b, "stream already allocated");
      b = dmaStreamAllocate(spip->dmatx,
                            STM32_SPI_SPI3_SLAVE_IRQ_PRIORITY,
                            (stm32_dmaisr_t)spi3_slave_lld_serve_tx_interrupt,
                            (void *)spip);
      osalDbgAssert(!b, "stream already allocated");
      rccEnableSPI3(FALSE);
    }
#endif

    /* DMA setup.*/
    dmaStreamSetPeripheral(spip->dmarx, &spip->spi->DR);
    dmaStreamSetPeripheral(spip->dmatx, &spip->spi->DR);
  }

  /* Configuration-specific DMA setup.*/
  if ((spip->config->cr1 & SPI_CR1_DFF) == 0) {
    /* Frame width is 8 bits or smaller.*/
    spip->rxdmamode = (spip->rxdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                      STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE;
    spip->txdmamode = (spip->txdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                      STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE;
  }
  else {
    /* Frame width is larger than 8 bits.*/
    spip->rxdmamode = (spip->rxdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                      STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    spip->txdmamode = (spip->txdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                      STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
  }
  /* SPI setup and enable.*/
  spip->spi->CR1  = 0;
  spip->spi->CR1  = spip->config->cr1; // | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
  spip->spi->CR2  = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN; // SPI_CR2_SSOE |
  spip->spi->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi3_slave_lld_stop(SPISlaveDriver *spip) {

  /* If in ready state then disables the SPI clock.*/
  if (spip->state == SPI_SLAVE_READY) {

    /* SPI disable.*/
    spip->spi->CR1 = 0;
    spip->spi->CR2 = 0;
    dmaStreamRelease(spip->dmarx);
    dmaStreamRelease(spip->dmatx);

#if STM32_SPI_USE_SPI3_SLAVE
    if (&SPISLAVED3 == spip)
      rccDisableSPI3(FALSE);
#endif

  }
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi3_slave_lld_ignore(SPISlaveDriver *spip, size_t n) {

  dmaStreamSetMemory0(spip->dmarx, &dummyrx);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode);

  dmaStreamSetMemory0(spip->dmatx, &dummytx);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi3_slave_lld_exchange(SPISlaveDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  dmaStreamSetMemory0(spip->dmarx, rxbuf);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode| STM32_DMA_CR_MINC);

  dmaStreamSetMemory0(spip->dmatx, txbuf);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode | STM32_DMA_CR_MINC);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi3_slave_lld_send(SPISlaveDriver *spip, size_t n, const void *txbuf) {

  dmaStreamSetMemory0(spip->dmarx, &dummyrx);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode);

  dmaStreamSetMemory0(spip->dmatx, txbuf);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode | STM32_DMA_CR_MINC);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi3_slave_lld_receive(SPISlaveDriver *spip, size_t n, void *rxbuf) {

  dmaStreamSetMemory0(spip->dmarx, rxbuf);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode | STM32_DMA_CR_MINC);

  dmaStreamSetMemory0(spip->dmatx, &dummytx);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi3_slave_lld_polled_exchange(SPISlaveDriver *spip, uint16_t frame) {

  spip->spi->DR = frame;
  while ((spip->spi->SR & SPI_SR_RXNE) == 0)
    ;
  return spip->spi->DR;
}

/**
 * @brief   Stops the ongoing data exchange.
 * @details The ongoing data exchange, if any, is stopped, if the driver
 *          was not active the function does nothing.
 *
 * @param[in] spip      pointer to the @p SPISlaveDriver object
 *
 * @notapi
 */
void spi3_slave_lld_stop_exchange(SPISlaveDriver *spip) {

  /* Stop TX DMA, if enabled.*/
  if (NULL != spip->dmatx)
    dmaStreamDisable(spip->dmatx)

  /* Stop RX DMA, if enabled.*/
  if (NULL != spip->dmarx)
    dmaStreamDisable(spip->dmarx);
}

#endif /* HAL_USE_SPI */

/** @} */
