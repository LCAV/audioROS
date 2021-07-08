#include <hal.h>
#include "e_uart_char.h"
#include "../usbcfg.h"

void e_init_uart1(void) {
	return;
}

/*! \brief Check if something is comming on uart 1
 * \return the number of characters available, 0 if none are available */
int  e_ischar_uart1(void) {
	// To be implemented!
	return 0;
}

int  e_getchar_uart1(char *car) {
	return chSequentialStreamRead(&SD3, (uint8_t*)car, 1);

//	if (SDU1.config->usbp->state == USB_ACTIVE) {
//		//return chnReadTimeout(&SDU1, car, 1, TIME_IMMEDIATE);
//		//return chnReadTimeout(&SDU1, car, 1, MS2ST(10));
//		return chSequentialStreamRead(&SDU1, (uint8_t*)car, 1);
//	}
//	//otherwise there is no wait state, this means the other threads can not be processed
//	chThdSleepMilliseconds(10);
//	return 0;
}

void e_send_uart1_char(const char * buff, int length) {
	chSequentialStreamWrite(&SD3, (uint8_t*)buff, length);

//	if (SDU1.config->usbp->state == USB_ACTIVE) {
//		//chnWriteTimeout(&SDU1, (uint8_t*)buff, length, TIME_INFINITE);
//		chSequentialStreamWrite(&SDU1, (uint8_t*)buff, length);
//	}
}

/*! \brief  To check if the sending operation is done
 * \return 1 if buffer sending is in progress, return 0 if not
 */
int  e_uart1_sending(void) {
	// To be implemented
	return 0;
}

void e_init_uart2(int baud) {
	(void)baud;
	return;
}

/*! \brief Check if something is comming on uart 2
 * \return the number of characters available, 0 if none are available */
int  e_ischar_uart2(void) {
	// To be implemented!
	return 0;
}

int  e_getchar_uart2(char *car) {
	if (SDU1.config->usbp->state == USB_ACTIVE) {
		//return chnReadTimeout(&SDU1, car, 1, TIME_IMMEDIATE);
		//return chnReadTimeout(&SDU1, car, 1, MS2ST(10));
		return chSequentialStreamRead(&SDU1, (uint8_t*)car, 1);
	}
	//otherwise there is no wait state, this means the other threads can not be processed
	chThdSleepMilliseconds(10);
	return 0;
}

void e_send_uart2_char(const char * buff, int length) {
	if (SDU1.config->usbp->state == USB_ACTIVE) {
		//chnWriteTimeout(&SDU1, (uint8_t*)buff, length, TIME_INFINITE);
		chSequentialStreamWrite(&SDU1, (uint8_t*)buff, length);
	}
}

/*! \brief  To check if the sending operation is done
 * \return 1 if buffer sending is in progress, return 0 if not
 */
int  e_uart2_sending(void) {
	// To be implemented.
	return 0;
}

