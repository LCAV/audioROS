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
 * @file    STM32F4xx/dcmi_lld.c
 * @brief   STM32 DCMI subsystem low level driver source.
 *
 * @addtogroup DCMI
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "dcmi_lld.h"
#include "dcmi.h"

#if HAL_USE_DCMI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define DCMI_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_DCMI_DMA_STREAM, STM32_DCMI_DMA_CHN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief DCMI driver identifier.*/
#if STM32_DCMI_USE_DCMI || defined(__DOXYGEN__)
DCMIDriver DCMID;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared "DMA transaction complete" service routine.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 * @param[in] flags      pre-shifted content of the ISR register
 */
static void dcmi_lld_serve_dma_rx_interrupt(DCMIDriver *dcmip, uint32_t flags) {

	/* DMA errors handling.*/
	if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
		_dcmi_isr_error_code(dcmip, DCMI_ERR_DMAFAILURE);
	}
	if( dcmip->config->transfer_complete_cb != NULL ) {
		dcmip->config->transfer_complete_cb(dcmip);
	}

}

/*===========================================================================*/
/* Driver interrupt handlers                               */
/*===========================================================================*/

/**
 * @brief   DCMI interrupt handler
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector178) {
	OSAL_IRQ_PROLOGUE();

	uint32_t flags =  DCMI->MISR;
	if ((flags & DCMI_MISR_FRAME_MIS) != 0) { // Capture complete.
        DCMI->ICR |= DCMI_ICR_FRAME_ISC;
        _dcmi_isr_code(&DCMID);
	}
	if ((flags & DCMI_MISR_OVF_MIS) != 0) { // DMA overflow.
        DCMI->ICR |= DCMI_ICR_OVF_ISC;
		_dcmi_isr_error_code(&DCMID, DCMI_ERR_OVERFLOW);
	}
    //DCMI->ICR |= DCMI_ICR_FRAME_ISC | DCMI_ICR_OVF_ISC | DCMI_ICR_ERR_ISC | DCMI_ICR_VSYNC_ISC | DCMI_ICR_LINE_ISC;

	OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level DCMI driver initialization.
 *
 * @notapi
 */
void dcmi_lld_init(void) {
	dcmiObjectInit(&DCMID);
	DCMID.dcmi = DCMI;
	DCMID.dmastp = STM32_DMA_STREAM(STM32_DCMI_DMA_STREAM);
	DCMID.dmamode = STM32_DMA_CR_CHSEL(1) | 					// Channel 1.
                    STM32_DMA_CR_PL(STM32_DCMI_DMA_PRIORITY) |	// High priority level.
                    STM32_DMA_CR_DIR_P2M |						// Peripheral to memory.
                    STM32_DMA_CR_TCIE |							// Transfer complete interrupt enabled.
                    STM32_DMA_CR_DMEIE |						// Direct mode error interrupt enabled.
                    STM32_DMA_CR_TEIE |							// Transfer error interrupt enabled.
                    STM32_DMA_CR_PBURST_SINGLE |				// Single transfer (no burst).
                    STM32_DMA_CR_MBURST_SINGLE |				// Single transfer (no burst).
                    STM32_DMA_CR_PSIZE_WORD |					// Peripheral data size = 4 bytes.
                    STM32_DMA_CR_MSIZE_WORD |					// Memory data size = 4 bytes.
                    STM32_DMA_CR_MINC |							// Increment memory address after each data transfer.
                    STM32_DMA_CR_CIRC;							// Circular mode.
                    //STM32_DMA_CR_HTIE |						// Half transfer interrupt enabled.
}

/**
 * @brief   Configures the DCMI peripheral and enable DMA.
 *
 * @param[in] dcmip				pointer to the @p DCMIDriver object
 * @param[in] transactionSize	Size of each receive buffer, in DCMI words.
 * @param[out] rxbuf0			the pointer to the first receive buffer
 * @param[out] rxbuf1			the pointer to the second receive buffer
 
 * @notapi
 */
void dcmi_lld_prepare(DCMIDriver *dcmip, uint32_t transactionSize, void* rxbuf0, void* rxbuf1) {

	/* If in stopped state then enables the DCMI and DMA.*/
	if (dcmip->state == DCMI_STOP) {
		if (&DCMID == dcmip) {
			/* DMA setup */
			bool b;
			b = dmaStreamAllocate(dcmip->dmastp, STM32_DCMI_DMA_IRQ_PRIORITY, (stm32_dmaisr_t)dcmi_lld_serve_dma_rx_interrupt, (void *)dcmip);
			osalDbgAssert(!b, "stream already allocated");
			dmaStreamSetPeripheral(dcmip->dmastp, &dcmip->dcmi->DR);
			dmaStreamSetFIFO(dcmip->dmastp, STM32_DMA_FCR_DMDIS); // Direct mode enabled (FIFO disabled).
			dmaStreamSetMemory0(dcmip->dmastp, rxbuf0);
			dmaStreamSetMemory1(dcmip->dmastp, rxbuf1);
			dmaStreamSetTransactionSize(dcmip->dmastp, transactionSize/4); // The DMA transactions are 32-bit width.
			// If second buffer not given, then turn off double buffering.
			if(rxbuf1 == NULL) {
				dcmip->dmamode &= (~STM32_DMA_CR_DBM);
			} else {
				dcmip->dmamode |= STM32_DMA_CR_DBM;
			}
			dmaStreamSetMode(dcmip->dmastp, dcmip->dmamode);
			dmaStreamEnable(dcmip->dmastp);

			/* DCMI setup */
			rccEnableDCMI(FALSE); // Enable DCMI clock.
			nvicEnableVector(DCMI_IRQn, STM32_DCMI_IRQ_PRIORITY);
			// Interrupt enable register.
			dcmip->dcmi->IER |= DCMI_IER_FRAME_IE; // Capture complete.
			//dcmip->dcmi->IER |= DCMI_IER_VSYNC_IE; // Interrupt generated when vsync become active (start of frame).
			dcmip->dcmi->IER |= DCMI_IER_OVF_IE; // Overrun (by DMA).
			// Control Regsiter.
			dcmip->dcmi->CR  = (dcmip->config->cr & ~(DCMI_CR_CAPTURE | DCMI_CR_ENABLE)); // Do not enable here because we don't still know the capture mode that will be used.
		}
	}

}

/**
 * @brief Deactivates the DCMI peripheral and release DMA.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 *
 * @notapi
 */
void dcmi_lld_unprepare(DCMIDriver *dcmip) {

	if(dcmip->state == DCMI_READY) {
		if(&DCMID == dcmip) {
			/* DCMI disable.*/
			dcmip->dcmi->CR &= ~(DCMI_CR_CAPTURE|DCMI_CR_ENABLE);
			dcmip->dcmi->IER &= ~(DCMI_IER_FRAME_IE|DCMI_IER_VSYNC_IE|DCMI_IER_ERR_IE|DCMI_IER_OVF_IE);
			rccDisableDCMI(FALSE);
			nvicDisableVector(DCMI_IRQn);
			/* DMA release */
			dmaStreamRelease(dcmip->dmastp);
		}
	}
}

 /**
 * @brief   Capture a single frame from the DCMI.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 *
 * @notapi
 */
void dcmi_lld_start_oneshot(DCMIDriver *dcmip) {
	if(dcmip->state == DCMI_READY) {
		dcmip->dcmi->CR |= DCMI_CR_CM;
		dcmip->dcmi->CR |= DCMI_CR_ENABLE;
		dcmip->dcmi->CR |= DCMI_CR_CAPTURE;
	}
}

 /**
 * @brief   Begins reception of frames from the DCMI.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 *
 * @notapi
 */
void dcmi_lld_start_stream(DCMIDriver *dcmip) {
	if(dcmip->state == DCMI_READY) {
		dcmip->dcmi->CR &= (~DCMI_CR_CM);
		dcmip->dcmi->CR |= DCMI_CR_ENABLE;
		dcmip->dcmi->CR |= DCMI_CR_CAPTURE;
	}
}

 /**
 * @brief   Stops reception of frames from the DCMI.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 *                      .
 * 
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @notapi
 */
msg_t dcmi_lld_stop_stream(DCMIDriver *dcmip) {
    systime_t start, end;
	if(dcmip->state == DCMI_ACTIVE_STREAM) {
		dcmip->dcmi->CR &= ~(DCMI_CR_CAPTURE); // If a capture is ongoing, the bit will be effectively cleared by hardware after the frame end.
		/* Calculating the time window for the timeout on waiting for CAPTURE flag bit to be cleared.*/
        start = osalOsGetSystemTimeX();
		end = start + OSAL_MS2ST(200);
		/* Waits until CAPTURE flag is reset or, alternatively, for a timeout condition.*/
		while (true) {
			/* If the CAPTURE bit is cleared then the capture process is terminated correctly.*/
            if((dcmip->dcmi->CR & DCMI_CR_CAPTURE) == 0) {
				return MSG_OK;
            }
			/* If the system time went outside the allowed window then a timeout condition is returned.*/
			if (!osalOsIsTimeWithinX(osalOsGetSystemTimeX(), start, end)) {
				return MSG_TIMEOUT;
			}
		}
	}
	return MSG_OK;
}

#endif /* HAL_USE_DCMI */

/** @} */
