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
 * @file    spi.h
 * @brief   SPI Driver macros and structures.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef _SPI3_SLAVE_H_
#define _SPI3_SLAVE_H_


#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SPI configuration options
 * @{
 */
/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(SPI_USE_WAIT) || defined(__DOXYGEN__)
#define SPI_USE_WAIT                TRUE
#endif

/**
 * @brief   Enables the @p spiAcquireBus() and @p spiReleaseBus() APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(SPI_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define SPI_USE_MUTUAL_EXCLUSION    TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  SPI_SLAVE_UNINIT = 0,                   /**< Not initialized.                   */
  SPI_SLAVE_STOP = 1,                     /**< Stopped.                           */
  SPI_SLAVE_READY = 2,                    /**< Ready.                             */
  SPI_SLAVE_ACTIVE = 3,                   /**< Exchanging data.                   */
  SPI_SLAVE_COMPLETE = 4                  /**< Asynchronous operation complete.   */
} spislavestate_t;


/**
 * @brief   Type of a structure representing an SPI driver.
 */
typedef struct SPISlaveDriver SPISlaveDriver;

/**
 * @brief   SPI notification callback type.
 *
 * @param[in] spip      pointer to the @p SPISlaveDriver object triggering the
 *                      callback
 * @param[in] offset    offset in buffers of the data to read/write
 * @param[in] n         number of samples to read/write*
 */
typedef void (*spislavecallback_t)(SPISlaveDriver *spip, size_t offset, size_t n);

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /**
   * @brief Operation complete callback or @p NULL.
   */
  spislavecallback_t             end_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief The chip select line port.
   */
  ioportid_t                ssport;
  /**
   * @brief The chip select line pad number.
   */
  uint16_t                  sspad;
  /**
   * @brief SPI initialization data.
   */
  uint16_t                  cr1;
  /**
   * @brief   Number of samples to exchange.
   */
  size_t                    size;
} SPISlaveConfig;

/**
 * @brief   Structure representing an SPI driver.
 */
struct SPISlaveDriver {
  /**
   * @brief Driver state.
   */
  spislavestate_t                state;
  /**
   * @brief Current configuration data.
   */
  const SPISlaveConfig           *config;
#if SPI_USE_WAIT || defined(__DOXYGEN__)
  /**
   * @brief Waiting thread.
   */
  thread_reference_t        thread;
#endif /* SPI_USE_WAIT */
#if SPI_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
  /**
   * @brief Mutex protecting the bus.
   */
  mutex_t                   mutex;
#endif /* SPI_USE_MUTUAL_EXCLUSION */
#if defined(SPI_DRIVER_EXT_FIELDS)
  SPI_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the SPIx registers block.
   */
  SPI_TypeDef               *spi;
  /**
   * @brief Receive DMA stream.
   */
  const stm32_dma_stream_t  *dmarx;
  /**
   * @brief Transmit DMA stream.
   */
  const stm32_dma_stream_t  *dmatx;
  /**
   * @brief RX DMA mode bit mask.
   */
  uint32_t                  rxdmamode;
  /**
   * @brief TX DMA mode bit mask.
   */
  uint32_t                  txdmamode;
};


/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */

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
 * @iclass
 */
#define spi3SlaveStartIgnoreI(spip, n) {                                          \
  (spip)->state = SPI_SLAVE_ACTIVE;                                               \
  spi3_slave_lld_ignore(spip, n);                                                  \
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
 * @iclass
 */
#define spi3SlaveStartExchangeI(spip, n, txbuf, rxbuf) {                          \
  (spip)->state = SPI_SLAVE_ACTIVE;                                               \
  spi3_slave_lld_exchange(spip, n, txbuf, rxbuf);                                  \
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
 * @iclass
 */
#define spi3SlaveStartSendI(spip, n, txbuf) {                                     \
  (spip)->state = SPI_SLAVE_ACTIVE;                                               \
  spi3_slave_lld_send(spip, n, txbuf);                                             \
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
 * @iclass
 */
#define spi3SlaveStartReceiveI(spip, n, rxbuf) {                                  \
  (spip)->state = SPI_SLAVE_ACTIVE;                                               \
  spi3_slave_lld_receive(spip, n, rxbuf);                                          \
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 * @note    This API is implemented as a macro in order to minimize latency.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
#define spi3SlavePolledExchange(spip, frame) spi_lld_polled_exchange(spip, frame)
/** @} */

/**
 * @name    Low level driver helper macros
 * @{
 */
#if (SPI_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
#define _spi3_slave_wakeup_isr(spip) {                                             \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(spip)->thread, MSG_OK);                               \
  osalSysUnlockFromISR();                                                   \
}
#else /* !SPI_USE_WAIT */
#define _spi3_slave_wakeup_isr(spip)
#endif /* !SPI_USE_WAIT */

/**
 * @brief   Stops the ongoing data exchange.
 * @details The ongoing data exchange, if any, is stopped, if the driver
 *          was not active the function does nothing.
 *
 * @param[in] spip      pointer to the @p SPISlaveDriver object
 *
 * @iclass
 */
#define spi3SlaveStopExchangeI(spip) {                                            \
  spi3_slave_lld_stop_exchange(spip);                                              \
  (spip)->state = SPI_SLAVE_READY;                                                \
}

/**
 * @brief   Common ISR code, half buffer event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] spip      pointer to the @p SPISlaveDriver object
 *
 * @notapi
 */
#define _spi3_slave_isr_half_code(spip) {                                          \
  if ((spip)->config->end_cb != NULL) {                                     \
    (spip)->config->end_cb(spip, 0, (spip)->config->size / 2);              \
  }                                                                         \
}

/**
 * @brief   Common ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] spip      pointer to the @p SPISlaveDriver object
 *
 * @notapi
 */
#define _spi3_slave_isr_full_code(spip) {                                               \
  if ((spip)->config->end_cb) {                                             \
    (spip)->state = SPI_SLAVE_COMPLETE;                                           \
    (spip)->config->end_cb(spip,                                            \
                           (spip)->config->size / 2,                        \
                           (spip)->config->size / 2);                       \
    if ((spip)->state == SPI_SLAVE_COMPLETE)                                      \
      (spip)->state = SPI_SLAVE_READY;                                            \
  }                                                                         \
  else                                                                      \
    (spip)->state = SPI_SLAVE_READY;                                              \
  _spi3_slave_wakeup_isr(spip);                                                    \
}

/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
#if STM32_SPI_USE_SPI3_SLAVE && !defined(__DOXYGEN__)
extern SPISlaveDriver SPISLAVED3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi3SlaveInit(void);
  void spi3SlaveObjectInit(SPISlaveDriver *spip);
  void spi3SlaveStart(SPISlaveDriver *spip, const SPISlaveConfig *config);
  void spi3SlaveStop(SPISlaveDriver *spip);
  void spi3SlaveStartIgnore(SPISlaveDriver *spip, size_t n);
  void spi3SlaveStartExchange(SPISlaveDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void spi3SlaveStopExchange(SPISlaveDriver *spip);
  void spi3SlaveStartSend(SPISlaveDriver *spip, size_t n, const void *txbuf);
  void spi3SlaveStartReceive(SPISlaveDriver *spip, size_t n, void *rxbuf);
#if SPI_USE_WAIT
  void spi3SlaveIgnore(SPISlaveDriver *spip, size_t n);
  void spi3SlaveExchange(SPISlaveDriver *spip, size_t n, const void *txbuf, void *rxbuf);
  void spi3SlaveSend(SPISlaveDriver *spip, size_t n, const void *txbuf);
  void spi3SlaveReceive(SPISlaveDriver *spip, size_t n, void *rxbuf);
#endif
#if SPI_USE_MUTUAL_EXCLUSION
  void spi3SlaveAcquireBus(SPISlaveDriver *spip);
  void spi3SlaveReleaseBus(SPISlaveDriver *spip);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI == TRUE */

#endif /* _SPI3_SLAVE_H_ */

/** @} */
