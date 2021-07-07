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
 * @file    spi.c
 * @brief   SPI Driver code.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"
#include "spi3_slave.h"
#include "../ports/STM32/LLD/SPIv1/spi3_slave_lld.h"

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
/** @brief SPI3 driver identifier.*/
#if STM32_SPI_USE_SPI3_SLAVE || defined(__DOXYGEN__)
SPISlaveDriver SPISLAVED3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   SPI Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void spi3SlaveInit(void) {

  spi3_slave_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p SPIDriver structure.
 *
 * @param[out] spip     pointer to the @p SPIDriver object
 *
 * @init
 */
void spi3SlaveObjectInit(SPISlaveDriver *spip) {

  spip->state = SPI_SLAVE_STOP;
  spip->config = NULL;
#if SPI_USE_WAIT == TRUE
  spip->thread = NULL;
#endif
#if SPI_USE_MUTUAL_EXCLUSION == TRUE
  osalMutexObjectInit(&spip->mutex);
#endif
#if defined(SPI_DRIVER_EXT_INIT_HOOK)
  SPI_DRIVER_EXT_INIT_HOOK(spip);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] config    pointer to the @p SPIConfig object
 *
 * @api
 */
void spi3SlaveStart(SPISlaveDriver *spip, const SPISlaveConfig *config) {

  osalDbgCheck((spip != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((spip->state == SPI_SLAVE_STOP) || (spip->state == SPI_SLAVE_READY),
                "invalid state");
  spip->config = config;
  spi3_slave_lld_start(spip);
  spip->state = SPI_SLAVE_READY;
  osalSysUnlock();
}

/**
 * @brief Deactivates the SPI peripheral.
 * @note  Deactivating the peripheral also enforces a release of the slave
 *        select line.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @api
 */
void spi3SlaveStop(SPISlaveDriver *spip) {

  osalDbgCheck(spip != NULL);

  osalSysLock();
  osalDbgAssert((spip->state == SPI_SLAVE_STOP) || (spip->state == SPI_SLAVE_READY),
                "invalid state");
  spi3_slave_lld_stop(spip);
  spip->state = SPI_SLAVE_STOP;
  osalSysUnlock();
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @api
 */
void spi3SlaveStartIgnore(SPISlaveDriver *spip, size_t n) {

  osalDbgCheck((spip != NULL) && (n > 0U));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_SLAVE_READY, "not ready");
  spi3SlaveStartIgnoreI(spip, n);
  osalSysUnlock();
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
void spi3SlaveStartExchange(SPISlaveDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  osalDbgCheck((spip != NULL) && (n > 0U) &&
               (rxbuf != NULL) && (txbuf != NULL));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_SLAVE_READY, "not ready");
  spi3SlaveStartExchangeI(spip, n, txbuf, rxbuf);
  osalSysUnlock();
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @api
 */
void spi3SlaveStartSend(SPISlaveDriver *spip, size_t n, const void *txbuf) {

  osalDbgCheck((spip != NULL) && (n > 0U) && (txbuf != NULL));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_SLAVE_READY, "not ready");
  spi3SlaveStartSendI(spip, n, txbuf);
  osalSysUnlock();
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
void spi3SlaveStartReceive(SPISlaveDriver *spip, size_t n, void *rxbuf) {

  osalDbgCheck((spip != NULL) && (n > 0U) && (rxbuf != NULL));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_SLAVE_READY, "not ready");
  spi3SlaveStartReceiveI(spip, n, rxbuf);
  osalSysUnlock();
}

#if (SPI_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Ignores data on the SPI bus.
 * @details This synchronous function performs the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     In order to use this function the option @p SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @api
 */
void spi3SlaveIgnore(SPISlaveDriver *spip, size_t n) {

  osalDbgCheck((spip != NULL) && (n > 0U));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_SLAVE_READY, "not ready");
  osalDbgAssert(spip->config->end_cb == NULL, "has callback");
  spi3SlaveStartIgnoreI(spip, n);
  (void) osalThreadSuspendS(&spip->thread);
  osalSysUnlock();
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This synchronous function performs a simultaneous transmit/receive
 *          operation.
 * @pre     In order to use this function the option @p SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
void spi3SlaveExchange(SPISlaveDriver *spip, size_t n,
                 const void *txbuf, void *rxbuf) {

  osalDbgCheck((spip != NULL) && (n > 0U) &&
               (rxbuf != NULL) && (txbuf != NULL));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_SLAVE_READY, "not ready");
  osalDbgAssert(spip->config->end_cb == NULL, "has callback");
  spi3SlaveStartExchangeI(spip, n, txbuf, rxbuf);
  (void) osalThreadSuspendS(&spip->thread);
  osalSysUnlock();
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This synchronous function performs a transmit operation.
 * @pre     In order to use this function the option @p SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @api
 */
void spi3SlaveSend(SPISlaveDriver *spip, size_t n, const void *txbuf) {

  osalDbgCheck((spip != NULL) && (n > 0U) && (txbuf != NULL));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_SLAVE_READY, "not ready");
  osalDbgAssert(spip->config->end_cb == NULL, "has callback");
  spi3SlaveStartSendI(spip, n, txbuf);
  (void) osalThreadSuspendS(&spip->thread);
  osalSysUnlock();
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This synchronous function performs a receive operation.
 * @pre     In order to use this function the option @p SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
void spi3SlaveReceive(SPISlaveDriver *spip, size_t n, void *rxbuf) {

  osalDbgCheck((spip != NULL) && (n > 0U) && (rxbuf != NULL));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_SLAVE_READY, "not ready");
  osalDbgAssert(spip->config->end_cb == NULL, "has callback");
  spi3SlaveStartReceiveI(spip, n, rxbuf);
  (void) osalThreadSuspendS(&spip->thread);
  osalSysUnlock();
}
#endif /* SPI_USE_WAIT == TRUE */

#if (SPI_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the SPI bus.
 * @details This function tries to gain ownership to the SPI bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p SPI_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @api
 */
void spi3SlaveAcquireBus(SPISlaveDriver *spip) {

  osalDbgCheck(spip != NULL);

  osalMutexLock(&spip->mutex);
}

/**
 * @brief   Releases exclusive access to the SPI bus.
 * @pre     In order to use this function the option @p SPI_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @api
 */
void spi3SlaveReleaseBus(SPISlaveDriver *spip) {

  osalDbgCheck(spip != NULL);

  osalMutexUnlock(&spip->mutex);
}

/**
 * @brief   Stops the ongoing data exchange.
 * @details The ongoing data exchange, if any, is stopped, if the driver
 *          was not active the function does nothing.
 *
 * @param[in] spip      pointer to the @p SPISlaveDriver object
 *
 * @api
 */
void spi3SlaveStopExchange(SPISlaveDriver *spip) {
	osalDbgCheck((spip != NULL));

	osalSysLock();
	osalDbgAssert((spip->state == SPI_SLAVE_READY) ||
	                (spip->state == SPI_SLAVE_ACTIVE) ||
	                (spip->state == SPI_SLAVE_COMPLETE),
	                "invalid state");
	spi3SlaveStopExchangeI(spip);
	osalSysUnlock();
}

#endif /* SPI_USE_MUTUAL_EXCLUSION == TRUE */

#endif /* HAL_USE_SPI == TRUE */

/** @} */
