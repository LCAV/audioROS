
#include "e_ad_conv.h"
#include "audio/microphone.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"

int16_t *e_mic_scan;			/*!< Array to store the mic values */
unsigned int e_last_mic_scan_id = MIC_SAMP_NB-1; // ID of the last scan in the mic array, with e-puck2 it's always the last array index.

void e_init_ad_scan(unsigned char only_micro)
{
	(void)only_micro;
	proximity_start();
	imu_start();
	mic_start(NULL);
	e_mic_scan = mic_get_buffer_ptr();
}

unsigned char e_ad_is_acquisition_completed(void)
{
	return 1;
}

unsigned char e_ad_is_array_filled(void)
{
	return 1;
}

void e_ad_scan_on(void)
{
	return;
}

void e_ad_scan_off(void)
{
	return;
}
