#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include "camera/dcmi_camera.h"
#include "button.h"
#include "leds.h"
#include "spi_comm.h"

//uint8_t spiRxBuff[SPI_COMMAND_SIZE];
//uint8_t spiTxBuff[SPI_COMMAND_SIZE];
//uint8_t spiHeader[SPI_DATA_HEADER_SIZE];

uint8_t spi_rx_buff[SPI_PACKET_MAX_SIZE];
uint8_t spi_tx_buff[SPI_PACKET_MAX_SIZE];

event_source_t ss_event;

/*
 * SPI communication thread.
 */
static THD_WORKING_AREA(spi_thread_wa, 1024);
static THD_FUNCTION(spi_thread, p) {
	(void)p;
	chRegSetThreadName("SPI thread");

//	uint32_t i = 0;
//	//uint16_t transCount = 0; // image size / SPI_BUFF_LEN
//	uint8_t id = 0;
//	uint16_t checksum = 0;
//	volatile uint32_t delay = 0;
//	uint16_t packetId = 0;
//	uint16_t numPackets = 0;
//	uint32_t remainingBytes = 0;
//	uint32_t spiDataIndex = 0;
//	event_listener_t ss_listener;
//	//eventmask_t evt;
//
//	// Create a fixed command packet content for debugging.
//	// The first two bytes are fixed to 0xAA, 0xBB, this is for synchronization purposes with the ESP32.
//	// The last byte represents the checksum (block check character), this is also used for synchronization purposes.
//	// The command packet content will be ([index]value): [0]0xAA, [1]0xBB, [2]3, [3]4, [4]5, ..., [61]62, [62]63, [63]checksum.
//	spiTxBuff[0] = 0xAA;
//	checksum += spiTxBuff[0];
//	spiTxBuff[1] = 0xBB;
//	checksum += spiTxBuff[1];
//	for(i=2; i<SPI_COMMAND_SIZE-1; i++) {
//		spiTxBuff[i] = i+1;
//		checksum += spiTxBuff[i];
//	}
//	spiTxBuff[SPI_COMMAND_SIZE-1] = checksum&0xFF; // Block check character checksum.
//
//	// Create a fixed data packet content for debugging.
//	// The data packet content will be: 0, 1, ..., 254, 255, 0, 1, ..., 254, 255, 0, 1, ...
//	sample_buffer = malloc(76800);
//	id = 0;
//	for(i=0; i<76800; i++) {
//		sample_buffer[i] = id;
//		if(id == 255) {
//			id = 0;
//		} else {
//			id++;
//		}
//	}
//
//	chEvtRegister(&ss_event, &ss_listener, 0);
//
//	//dcmiStartOneShot(&DCMID);
//	//chThdSleepMilliseconds(500);

	while (true) {

//		memset(spi_tx_buff, 0x00, 12);
		get_all_rgb_state(&spi_tx_buff[0]);
//		spi_tx_buff[0] = 50;
//		spi_tx_buff[4] = 50;
//		spi_tx_buff[8] = 50;
//		spi_tx_buff[9] = 1;
//		spi_tx_buff[10] = 50;
//		spi_tx_buff[11] = 50;
		spiSelect(&SPID1);
//		for(delay=0; delay<SPI_DELAY; delay++) {
//			__NOP();
//		}
		spiExchange(&SPID1, 12, spi_tx_buff, spi_rx_buff);
//		for(delay=0; delay<SPI_DELAY; delay++) {
//			__NOP();
//		}
		spiUnselect(&SPID1);

//		// A little pause is needed for the communication to work, 400 NOP loops last about 26 us.
//		// Probably this pause can be avoided since we loose some time computing the checksum...
//		for(delay=0; delay<SPI_DELAY; delay++) {
//			__NOP();
//		}

		button_set_state(spi_rx_buff[0]);

		//because of DMA problem between SPI1 and DCMI, we need to wait the end of 
		//the DCMI transfer before beginning this one
		// !!!!  not sure it will work when thransfer size of SPI will be greater !!!!
		if(DCMID.state == DCMI_ACTIVE_STREAM || DCMID.state == DCMI_ACTIVE_ONESHOT){
			//between 26Hz and 12Hz depending ont the capture time of the camera
			wait_image_ready();
		}
		else{
			//100Hz
			chThdSleepMilliseconds(20);
		}

//		//evt = chEvtWaitAny(ALL_EVENTS);
//
//		//if (evt & EVENT_MASK(0)) {
//
//			palSetPad(GPIOD, 13) ; // Orange.
//			palSetPad(GPIOD, 15); // Blue.
//
//			memset(spiRxBuff, 0x00, SPI_COMMAND_SIZE);
//			spiSelect(&SPID1);
//			//chThdSleepMilliseconds(1);
//			spiExchange(&SPID1, SPI_COMMAND_SIZE, spiTxBuff, spiRxBuff);
//			//chThdSleepMilliseconds(1);
//			//spiReceive(&SPID1, SPI_COMMAND_SIZE, spiRxBuff);
//			spiUnselect(&SPID1);
//
//			// A little pause is needed for the communication to work, 400 NOP loops last about 26 us.
//			// Probably this pause can be avoided/reduced since we loose some time computing the checksum...
//			for(delay=0; delay<SPI_DELAY; delay++) {
//				__NOP();
//			}
//
//			// Compute the checksum (block check character) to verify the command is received correctly.
//			// If the command is incorrect, wait for the next command, this is an easy way to synchronize the two chips.
//			checksum = 0;
//			for(i=0; i<SPI_COMMAND_SIZE-1; i++) {
//				checksum += spiRxBuff[i];
//			}
//			checksum = checksum &0xFF;
//			// The checksum coupled with the two fixed bytes increase the probability that the command packet is identified correctly.
//			if(checksum != spiRxBuff[SPI_COMMAND_SIZE-1] || spiRxBuff[0]!=0xAA || spiRxBuff[1]!=0xBB) {
//				//chprintf((BaseSequentialStream *)&SDU1, "F:%.3d %.3d %.3d %.3d %.3d %.3d %.3d\r\n", spiRxBuff[0], spiRxBuff[1], spiRxBuff[2], spiRxBuff[3], spiRxBuff[SPI_COMMAND_SIZE-3], spiRxBuff[SPI_COMMAND_SIZE-2], spiRxBuff[SPI_COMMAND_SIZE-1]);
//				//chThdSleepMilliseconds(1); // Give time to the ESP32 to be listening.
//				continue;
//			}
//
//			palClearPad(GPIOD, 15); // Blue.
//
//			/*
//			spiSelect(&SPID1);
//			spiExchange(&SPID1, SPI_COMMAND_SIZE, spiTxBuff, spiRxBuff);
//			//spiReceive(&SPID1, SPI_COMMAND_SIZE, spiRxBuff);
//			spiUnselect(&SPID1);
//
//			// A little pause is needed for the communication to work, 400 NOP loops last about 26 us.
//			// Probably this pause can be avoided since we loose some time computing the checksum...
//			for(delay=0; delay<SPI_DELAY; delay++) {
//				__NOP();
//			}
//			*/
//
//			// Modify the packet content in order to test the exchange of a dynamic payload instead of a fixed one.
//			/*
//			if(spiTxBuff[0] >= SPI_COMMAND_SIZE*0) {
//				for(i=0; i<SPI_COMMAND_SIZE; i++) {
//					spiTxBuff[i] = i;
//				}
//			} else {
//				for(i=0; i<SPI_COMMAND_SIZE; i++) {
//					spiTxBuff[i] += SPI_COMMAND_SIZE;
//				}
//			}
//			*/
//
//			numPackets = 76800/SPI_DATA_PAYLOAD_SIZE;
//			remainingBytes = 76800%SPI_DATA_PAYLOAD_SIZE;
//			spiDataIndex = 0;
//
//			for(packetId=0; packetId<numPackets; packetId++) {
//				/*
//				spiHeader[0] = packetId&0xFF;
//				spiHeader[1] = packetId>>8;
//				spiHeader[2] = SPI_DATA_PAYLOAD_SIZE&0xFF;
//				spiHeader[3] = SPI_DATA_PAYLOAD_SIZE>>8;
//
//				spiSelect(&SPID1);
//				spiSend(&SPID1, SPI_DATA_HEADER_SIZE, spiHeader);
//				spiUnselect(&SPID1);
//				// A little pause is needed for the communication to work, 400 NOP loops last about 26 us.
//				for(delay=0; delay<SPI_DELAY; delay++) {
//					__NOP();
//				}
//				*/
//
//				spiSelect(&SPID1);
//				//chThdSleepMilliseconds(1);
//				spiSend(&SPID1, SPI_DATA_PAYLOAD_SIZE, &sample_buffer[spiDataIndex]);
//				//chThdSleepMilliseconds(1);
//				spiUnselect(&SPID1);
//				// A little pause is needed for the communication to work, 400 NOP loops last about 26 us.
//				for(delay=0; delay<SPI_DELAY; delay++) {
//					__NOP();
//				}
//
//				spiDataIndex += SPI_DATA_PAYLOAD_SIZE;
//			}
//			if(remainingBytes > 0) {
//				/*
//				spiHeader[0] = packetId&0xFF;
//				spiHeader[1] = packetId>>8;
//				spiHeader[2] = remainingBytes&0xFF;
//				spiHeader[3] = remainingBytes>>8;
//
//				spiSelect(&SPID1);
//				spiSend(&SPID1, SPI_DATA_HEADER_SIZE, spiHeader);
//				spiUnselect(&SPID1);
//				// A little pause is needed for the communication to work, 400 NOP loops last about 26 us.
//				for(delay=0; delay<SPI_DELAY; delay++) {
//					__NOP();
//				}
//				*/
//
//				spiSelect(&SPID1);
//				//palSetPad(GPIOD, 15); // Blue.
//				//chThdSleepMilliseconds(1);
//				spiSend(&SPID1, remainingBytes, &sample_buffer[spiDataIndex]);
//				//chThdSleepMilliseconds(1);
//				//palClearPad(GPIOD, 15); // Blue.
//				spiUnselect(&SPID1);
//
//				// A little pause is needed for the communication to work, 400 NOP loops last about 26 us.
//				for(delay=0; delay<SPI_DELAY; delay++) {
//					__NOP();
//				}
//			}
//
//			/*
//			for(transCount=0; transCount<76800/SPI_PACKET_SIZE; transCount++) {
//				spiSelect(&SPID1);
//				spiSend(&SPID1, SPI_PACKET_SIZE, &sample_buffer[transCount*SPI_PACKET_SIZE]);
//				spiUnselect(&SPID1);
//				// A little pause is needed for the communication to work, 400 NOP loops last about 26 us.
//				for(delay=0; delay<SPI_DELAY; delay++) {
//					__NOP();
//				}
//			}
//			*/
//
//
//			palClearPad(GPIOD, 13) ; // Orange.
//
//			//chprintf((BaseSequentialStream *)&SDU1, "recv: %d, %d, %d, %d, %d, %d, %d\r\n", spiRxBuff[0], spiRxBuff[1], spiRxBuff[2], spiRxBuff[3], spiRxBuff[SPI_COMMAND_SIZE-3], spiRxBuff[SPI_COMMAND_SIZE-2], spiRxBuff[SPI_COMMAND_SIZE-1]);
//			//chprintf((BaseSequentialStream *)&SDU1, "n=%d, r=%d\r\n", numPackets, remainingBytes);
//
//			//chThdSleepMilliseconds(50);
//			//dcmiStartOneShot(&DCMID);
//			//chThdSleepMilliseconds(500);
//
//		//} // Event handling.
//
//		//break;

	} // Infinite loop.

//	free(sample_buffer);

}

void spi_comm_start(void) {
	//	osalEventObjectInit(&ss_event);

	// SPI1 maximum speed is 42 MHz, ESP32 supports at most 10MHz, so use a prescaler of 1/8 (84 MHz / 8 = 10.5 MHz).
	// SPI1 configuration (10.5 MHz, CPHA=0, CPOL=0, MSb first).
	static const SPIConfig hs_spicfg = {
		NULL,
		GPIOA,
		15,
		SPI_CR1_BR_1
		//SPI_CR1_BR_1 | SPI_CR1_BR_0 // 5.25 MHz
	};
	spiStart(&SPID1, &hs_spicfg);	// Setup transfer parameters.
	chThdCreateStatic(spi_thread_wa, sizeof(spi_thread_wa), NORMALPRIO+1, spi_thread, NULL);
	//chThdCreateStatic(spi_thread_wa, sizeof(spi_thread_wa), NORMALPRIO + 1, spi_thread, NULL);
}

