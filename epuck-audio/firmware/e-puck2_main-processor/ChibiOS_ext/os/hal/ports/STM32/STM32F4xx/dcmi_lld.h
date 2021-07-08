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
 * @file    STM32F4xx/dcmi_lld.h
 * @brief   STM32 DCMI subsystem low level driver header.
 *
 * @addtogroup DCMI
 * @{
 */

#ifndef _DCMI_LLD_H_
#define _DCMI_LLD_H_

#include "stm32_registry.h"
#include "stm32_rcc.h"

#if HAL_USE_DCMI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define STM32_DCMI_CR_EDM_MASK (DCMI_CR_EDM_0|DCMI_CR_EDM_1)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   DCMI driver enable switch.
 * @details If set to @p TRUE the support for DCMI is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DCMI_USE_DCMI) || defined(__DOXYGEN__)
#define STM32_DCMI_USE_DCMI                  FALSE
#endif

/**
 * @brief   DCMI interrupt priority level setting.
 */
#if !defined(STM32_DCMI_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DCMI_IRQ_PRIORITY         6
#endif

/**
 * @brief   DCMI DMA interrupt priority level setting.
 */
#if !defined(STM32_DCMI_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DCMI_DMA_IRQ_PRIORITY         12
#endif

/**
 * @brief   DCMI DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DCMI_DCMI_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DCMI_DMA_PRIORITY         2
#endif

/**
 * @brief   DMA stream used for DCMI operations.
 * @note    This option is only available on platforms with enhanced DMA.
 */
#if !defined(STM32_DCMI_DMA_STREAM) || defined(__DOXYGEN__)
#define STM32_DCMI_DMA_STREAM        STM32_DMA_STREAM_ID(2, 1)
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_DCMI_USE_DCMI && !STM32_HAS_DCMI
#error "DCMI not present in the selected device"
#endif

#if !STM32_DCMI_USE_DCMI
#error "DCMI driver activated but no DCMI peripheral assigned"
#endif

#if STM32_DCMI_USE_DCMI &&                                                   \
   !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DCMI_IRQ_PRIORITY)
#error "Invalid DCMI IRQ priority assigned to DCMI"
#endif

#if STM32_DCMI_USE_DCMI &&                                                   \
   !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DCMI_DMA_IRQ_PRIORITY)
#error "Invalid DMA IRQ priority assigned to DCMI"
#endif

#if STM32_DCMI_USE_DCMI &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DCMI_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DCMI"
#endif

#if STM32_DCMI_USE_DCMI &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_DCMI_DMA_STREAM, STM32_DCMI_DMA_MSK)
#error "invalid DMA stream associated to DCMI"
#endif

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Possible DCMI failure causes.
 * @note    Error codes are architecture dependent and should not relied upon.
 */
typedef enum {
  DCMI_ERR_DMAFAILURE = 0,					/**< DMA operations failure.	*/
  DCMI_ERR_OVERFLOW = 1						/**< DCMI overflow condition.	*/
} dcmierror_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  DCMI_UNINIT = 0,					/**< Not initialized.									*/
  DCMI_STOP = 1,					/**< Stopped.											*/
  DCMI_READY = 2,					/**< Ready to begin listening.							*/
  DCMI_ACTIVE_STREAM = 3,			/**< Listening for frames, continuous.					*/
  DCMI_ACTIVE_ONESHOT = 4,			/**< Listening for frames, one shot.					*/
  DCMI_COMPLETE = 5,				/**< Frame complete, callback pending.					*/
  DCMI_ERROR = 6,					/**< DCMI or DMA error.									*/
  DCMI_TIMEOUT = 7					/**< DCMI timeout, cannot stop correctly.				*/
} dcmistate_t;

/**
 * @brief   Type of a structure representing a DCMI driver.
 */
typedef struct DCMIDriver DCMIDriver;

/**
 * @brief   DCMI notification callback type.
 *
 * @param[in] dcmip		pointer to the @p DCMIDriver object triggering the callback.
 */
typedef void (*dcmicallback_t)(DCMIDriver *dcmip);

/**
 * @brief   DCMI error callback type.
 *
 * @param[in] dcmi      pointer to the @p DCMIDriver object triggering the callback.
 * @param[in] err       DCMI error code.
 */
typedef void (*dcmierrorcallback_t)(DCMIDriver *adcp, dcmierror_t err);

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /**
   * @brief DCMI frame complete callback or @p NULL.
   */
  dcmicallback_t			frame_end_cb;
  /**
   * @brief DMA transfer complete callback or @p NULL.
   */
  dcmicallback_t			transfer_complete_cb;
  /**
   * @brief Error callback or @p NULL.
   */
  dcmierrorcallback_t		error_cb;
  /**
   * @brief DCMI CR register initialization data.
   */
  uint32_t					cr;
} DCMIConfig;

/**
 * @brief   Structure representing a DCMI driver.
 */
struct DCMIDriver {
  /**
   * @brief Driver state.
   */
  dcmistate_t				state;
  /**
   * @brief Current configuration data.
   */
  const DCMIConfig			*config;
  /**
   * @brief Pointer to the DCMI registers block.
   */
  DCMI_TypeDef				*dcmi;
  /**
   * @brief DMA stream.
   */
  const stm32_dma_stream_t	*dmastp;
  /**
   * @brief DMA mode bit mask.
   */
  uint32_t					dmamode;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_DCMI_USE_DCMI && !defined(__DOXYGEN__)
extern DCMIDriver DCMID;
#endif

#ifdef __cplusplus
extern "C" {
#endif
    void dcmi_lld_init(void);
	void dcmi_lld_prepare(DCMIDriver *dcmip, uint32_t transactionSize, void* rxbuf0, void* rxbuf1);
	void dcmi_lld_unprepare(DCMIDriver *dcmip);
	void dcmi_lld_start_oneshot(DCMIDriver *dcmip);
    void dcmi_lld_start_stream(DCMIDriver *dcmip);
	msg_t dcmi_lld_stop_stream(DCMIDriver *dcmip);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DCMI */

#endif /* _DCMI_LLD_H_ */

/** @} */
