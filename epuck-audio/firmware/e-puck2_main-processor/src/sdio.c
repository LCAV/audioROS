#include <hal.h>
#include "sdio.h"

static bool sdio_connected = false;

void sdio_start(void) {
	static uint8_t sd_scratchpad[512]; // Working area for SDC driver.
	static const SDCConfig sdccfg = { //  SDIO configuration.
	  sd_scratchpad,
	  SDC_MODE_1BIT // Use 1-bit mode instead of 4-bit to avoid conflicts with the microphones.
	};
	sdcStart(&SDCD1, &sdccfg);
}

uint8_t sdio_is_present(void) {
	return blkIsInserted(&SDCD1);
}

uint8_t sdio_connect(void) {
	if(!sdio_connected){
		if(sdcConnect(&SDCD1) == HAL_SUCCESS){
			sdio_connected = true;
		}else{
			return HAL_FAILED;
		}
	}
	return HAL_SUCCESS;
}

uint8_t sdio_disconnect(void) {
	if(sdio_connected){
		if(sdcDisconnect(&SDCD1) == HAL_SUCCESS){
			sdio_connected = false;
		}else{
			return HAL_FAILED;
		}
	}
	return HAL_SUCCESS;
	
}

