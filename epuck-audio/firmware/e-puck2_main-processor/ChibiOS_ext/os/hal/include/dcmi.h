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
 * @file    dcmi.h
 * @brief   DCMI Driver macros and structures.
 *
 * @addtogroup DCMI
 * @{
 */

#ifndef _DCMI_H_
#define _DCMI_H_

#include "../ports/STM32/STM32F4xx/dcmi_lld.h"

#if (HAL_USE_DCMI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

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

/**
 * @name    Macro Functions
 * @{
 */
 
/**
 * @brief   Begins reception of frames from the DCMI.
 * @details This asynchronous function starts a continuous receive operation.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 *
 * @iclass
 */
#define dcmiStartStreamI(dcmip) {               \
    dcmi_lld_start_stream(dcmip);               \
    (dcmip)->state = DCMI_ACTIVE_STREAM;        \
}

/**
 * @brief   Capture a single frame from the DCMI.
 * @details This asynchronous function starts a single shot receive operation.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 *
 * @iclass
 */
#define dcmiStartOneShotI(dcmip) {              \
    dcmi_lld_start_oneshot(dcmip);              \
    (dcmip)->state = DCMI_ACTIVE_ONESHOT;       \
}


/**
 * @name    Low Level driver helper macros
 * @{
 */

/**
 * @brief   Common DCMI Frame Complete ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 *
 * @notapi
 */
#define _dcmi_isr_code(dcmip) {												\
	bool oneShot = ((dcmip)->state == DCMI_ACTIVE_ONESHOT);					\
	bool isReady = (dcmip)->state == DCMI_READY;							\
	if(((dcmip)->state==DCMI_TIMEOUT) || ((dcmip)->state==DCMI_ERROR)) {	\
		return;																\
	}																		\
	(dcmip)->state = DCMI_COMPLETE;											\
	if ((dcmip)->config->frame_end_cb != NULL) {							\
		(dcmip)->config->frame_end_cb(dcmip);								\
	}																		\
	if(oneShot) {															\
		(dcmip)->state = DCMI_READY;										\
	} else {																\
		if(isReady) {														\
			(dcmip)->state = DCMI_READY;									\
		} else {															\
			(dcmip)->state = DCMI_ACTIVE_STREAM;							\
		}																	\
	}																		\
}
/** @} */

/**
 * @brief   Common ISR code, error event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 * @param[in] err       platform dependent error code
 *
 * @notapi
 */
#define _dcmi_isr_error_code(dcmip, err) {								\
    if((dcmip)->state == DCMI_ACTIVE_STREAM) {							\
        dcmi_lld_stop_stream(dcmip);									\
    }																	\
	(dcmip)->state = DCMI_ERROR;										\
	if ((dcmip)->config->error_cb != NULL) {							\
		(dcmip)->config->error_cb(dcmip, err);							\
	}																	\
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
	void dcmiInit(void);
	void dcmiObjectInit(DCMIDriver *dcmip);
    void dcmiPrepare(DCMIDriver *dcmip, const DCMIConfig *config, uint32_t transactionSize, void* rxbuf0, void* rxbuf1);
    void dcmiUnprepare(DCMIDriver *dcmip);
    void dcmiStartOneShot(DCMIDriver *dcmip);
    void dcmiStartStream(DCMIDriver *dcmip);
    msg_t dcmiStopStream(DCMIDriver *dcmip);

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DCMI */

#endif /* _DCMI_H_ */

/** @} */
