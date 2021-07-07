#include "e_micro.h"
#include "e_ad_conv.h"
#include "../audio/microphone.h"
#include <hal.h>
#include "../usbcfg.h"
#include "ch.h"
#include "chprintf.h"

/*****************************************************
 * user function                                     *
 *****************************************************/

int e_get_micro(unsigned int micro_id)
{
	int16_t value = mic_get_last(micro_id);
	return (value>>4); // Adapt the values to be compatible with e-puck1.x (16 bits => 12 bits).
}

int e_get_micro_average(unsigned int micro_id, unsigned int filter_size)
{
	int16_t* mic_data = mic_get_buffer_ptr();
	int32_t temp = 0;
	uint16_t i,j;

	if ((micro_id < 3) && (filter_size > 0) && (filter_size <= MIC_SAMP_NB))
	{
		for (i=0,j=(MIC_SAMP_NB-filter_size); i<filter_size ; i++,j++)
		{
			temp += mic_data[micro_id+j*4];
		}
	}
	return ((int)(temp/filter_size));
}

int e_get_micro_volume (unsigned int micro_id)
{
	int32_t value = mic_get_volume(micro_id);
	return (value>>4); // Adapt the values to be compatible with e-puck1.x (16 bits => 12 bits).
}

void e_get_micro_last_values (int micro_id, int16_t * result, unsigned samples_nb)
{
	uint16_t i;
	int16_t* mic_data = mic_get_buffer_ptr();

	if (samples_nb > 0 && samples_nb <= MIC_SAMP_NB)
	{
		for (i=0 ; i<samples_nb ; i++)
		{
			result[i] = mic_data[(MIC_SAMP_NB-1-i)*4+micro_id];
		}
	}
}

void e_test_micro(void) {
	int i=0;
	int16_t mic_res[3] = {0};
	int16_t* mic_data = mic_get_buffer_ptr();
	mic_data[639] = 10; // mic3
	mic_data[638] = 11; // mic2
	mic_data[637] = 12; // mic1
	mic_data[636] = 13; // mic0
	mic_data[635] = 14; // mic3
	mic_data[634] = 15; // mic2
	mic_data[633] = 16; // mic1
	mic_data[632] = 17; // mic0
	mic_data[631] = 18; // mic3
	mic_data[630] = 19; // mic2
	mic_data[629] = 20; // mic1
	mic_data[628] = 21; // mic0

	chprintf((BaseSequentialStream *)&SDU1, "mic0 last values: ");
	e_get_micro_last_values(0, mic_res, 3);
	for(i=0; i<3; i++) {
		chprintf((BaseSequentialStream *)&SDU1, "%d, ", mic_res[i]);
	}
	chprintf((BaseSequentialStream *)&SDU1, "\r\n");

	chprintf((BaseSequentialStream *)&SDU1, "mic1 last values: ");
	e_get_micro_last_values(1, mic_res, 3);
	for(i=0; i<3; i++) {
		chprintf((BaseSequentialStream *)&SDU1, "%d, ", mic_res[i]);
	}
	chprintf((BaseSequentialStream *)&SDU1, "\r\n");

	chprintf((BaseSequentialStream *)&SDU1, "mic2 last values: ");
	e_get_micro_last_values(2, mic_res, 3);
	for(i=0; i<3; i++) {
		chprintf((BaseSequentialStream *)&SDU1, "%d, ", mic_res[i]);
	}
	chprintf((BaseSequentialStream *)&SDU1, "\r\n");

	chprintf((BaseSequentialStream *)&SDU1, "mic3 last values: ");
	e_get_micro_last_values(3, mic_res, 3);
	for(i=0; i<3; i++) {
		chprintf((BaseSequentialStream *)&SDU1, "%d, ", mic_res[i]);
	}
	chprintf((BaseSequentialStream *)&SDU1, "\r\n");

	chprintf((BaseSequentialStream *)&SDU1, "mic0 avg = %d\r\n", e_get_micro_average(0, 3));
	chprintf((BaseSequentialStream *)&SDU1, "mic1 avg = %d\r\n", e_get_micro_average(1, 3));
	chprintf((BaseSequentialStream *)&SDU1, "mic2 avg = %d\r\n", e_get_micro_average(2, 3));
	chprintf((BaseSequentialStream *)&SDU1, "mic3 avg = %d\r\n", e_get_micro_average(3, 3));
}
