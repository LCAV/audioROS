/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    dcmi.c
 * @brief   DCMI Driver code.
 *
 * @addtogroup DCMI
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "dcmi.h"

#if HAL_USE_DCMI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   DCMI Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void dcmiInit(void) {
	dcmi_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p DCMIDriver structure.
 *
 * @param[out] dcmip     pointer to the @p DCMIDriver object
 *
 * @init
 */
void dcmiObjectInit(DCMIDriver *dcmip) {
	dcmip->state = DCMI_STOP;
	dcmip->config = NULL;
#if defined(DCMI_DRIVER_EXT_INIT_HOOK)
	DCMI_DRIVER_EXT_INIT_HOOK(dcmip); // Needed??
#endif
}

 /**
 * @brief   Configures the DCMI peripheral.
 * @details This function configure the DCMI peripheral but keep it disabled; the DMA is configured and enabled.
 * @post    Upon either of the two buffers being filled, the configured callback
 *          (transfer_complete_cb) is invoked.
 *          At the end of each frame the configured callback
 *          (frame_end_cb) is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes equal to
 *          8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] dcmip				pointer to the @p DCMIDriver object
 * @param[in] config			pointer to the @p DCMIConfig object
 * @param[in] transactionSize	Size of each receive buffer, in DCMI words.
 * @param[out] rxbuf0			the pointer to the first receive buffer
 * @param[out] rxbuf1			the pointer to the second receive buffer
 *
 * @api
 */
void dcmiPrepare(DCMIDriver *dcmip, const DCMIConfig *config, uint32_t transactionSize, void* rxbuf0, void* rxbuf1) {
	// The maximum number of transactions is 65535, each transaction is 32-bit width.
	// The "transactionSize" is expressed in bytes thus its maximum value is 65535*4, moreover its value must be a multiple of 4.
	osalDbgCheck((dcmip != NULL) && (config != NULL) && (transactionSize <= (65535*4)) && ((transactionSize%4)==0) && (rxbuf0 != NULL));
	osalSysLock();
	osalDbgAssert((dcmip->state == DCMI_STOP), "invalid state");
	dcmip->config = config;
	dcmi_lld_prepare(dcmip, transactionSize, rxbuf0, rxbuf1);
	dcmip->state = DCMI_READY;
	osalSysUnlock();
}

/**
 * @brief Deactivates the DCMI peripheral.
 * @details This function disable the DCMI and related interrupts; also the DMA is released.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 *
 * @api
 */
void dcmiUnprepare(DCMIDriver *dcmip) {
	osalDbgCheck(dcmip != NULL);
	osalSysLock();
	osalDbgAssert((dcmip->state == DCMI_READY), "invalid state");
	dcmi_lld_unprepare(dcmip);
	dcmip->state = DCMI_STOP;
	osalSysUnlock();
}

 /**
 * @brief   Capture a single frame from the DCMI.
 * @details This asynchronous function starts a single shot receive operation.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 *
 * @api
 */
void dcmiStartOneShot(DCMIDriver *dcmip) {
	osalDbgCheck((dcmip != NULL));
	osalSysLock();
	osalDbgAssert(dcmip->state == DCMI_READY, "not ready");
	dcmiStartOneShotI(dcmip);
	osalSysUnlock();
}

 /**
 * @brief   Begins reception of frames from the DCMI.
 * @details This asynchronous function starts a continuous receive operation.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 *
 * @api
 */
void dcmiStartStream(DCMIDriver *dcmip) {
	osalDbgCheck((dcmip != NULL));
	osalSysLock();
	osalDbgAssert(dcmip->state == DCMI_READY, "not ready");
	dcmiStartStreamI(dcmip);
	osalSysUnlock();
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
 * @api
 */
msg_t dcmiStopStream(DCMIDriver *dcmip) {
	osalDbgCheck((dcmip != NULL));
	osalSysLock();
	osalDbgAssert(dcmip->state == DCMI_ACTIVE_STREAM, "not ready");
    msg_t ret = dcmi_lld_stop_stream(dcmip);
	if(ret == MSG_OK) {
		dcmip->state = DCMI_READY;
	} else {
		dcmip->state = DCMI_TIMEOUT;
	}
	osalSysUnlock();
	return ret;
}

#endif /* HAL_USE_DCMI */

/** @} */
