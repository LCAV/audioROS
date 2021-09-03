#include <ch.h>
#include <hal.h>
#include "audio_thread.h"

#define STATE_STOPPED     0
#define STATE_PLAYING     1
#define SINUS_BUFFER_SIZE 360
#define NOISE_REDUCTION_FACTOR (40)

#define DAC_USED          DACD2
#define TIMER_DAC         GPTD6

/*
 * The DAC of the uC is configured to output a buffer given in a circular manner, 
 * in our case a sine wave to produce sound. A GPT (general purpose timer) is used to trigger
 * the DAC. Each time the DAC is triggered, it output the following value of the buffer.
 * 
 * By changing the interval of the GPT, we change the frequency of the sound.
 * 
 */

/*
 * DAC test buffer (sine wave).
 */
static const dacsample_t sinus_buffer[SINUS_BUFFER_SIZE] = {
  ( (uint16_t)2047 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2082 / NOISE_REDUCTION_FACTOR), ( (uint16_t)2118 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2154 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2189 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2225 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2260 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2296 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2331 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2367 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2402 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2437 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2472 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2507 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2542 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2576 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2611 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2645 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2679 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2713 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2747 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2780 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2813 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2846 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2879 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2912 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2944 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2976 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3008 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3039 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3070 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3101 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3131 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3161 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3191 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3221 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3250 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3278 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3307 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3335 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3362 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3389 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3416 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3443 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3468 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3494 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3519 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3544 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3568 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3591 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3615 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3637 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3660 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3681 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3703 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3723 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3744 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3763 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3782 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3801 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3819 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3837 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3854 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3870 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3886 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3902 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3917 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3931 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3944 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3958 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3970 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3982 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3993 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4004 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4014 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4024 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4033 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4041 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4049 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4056 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4062 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4068 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4074 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4078 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4082 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4086 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4089 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4091 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4092 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4093 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4094 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4093 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4092 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4091 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4089 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4086 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4082 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4078 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4074 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4068 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4062 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4056 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4049 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4041 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4033 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)4024 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4014 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)4004 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3993 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3982 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3970 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3958 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3944 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3931 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3917 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3902 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3886 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3870 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3854 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3837 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3819 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3801 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3782 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3763 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3744 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3723 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3703 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3681 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3660 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3637 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3615 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3591 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3568 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3544 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3519 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3494 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3468 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3443 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3416 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3389 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3362 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3335 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3307 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3278 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3250 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3221 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3191 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3161 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3131 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3101 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3070 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3039 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)3008 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2976 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2944 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2912 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2879 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2846 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2813 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2780 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2747 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2713 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2679 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2645 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2611 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2576 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2542 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2507 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2472 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2437 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2402 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2367 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2331 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2296 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2260 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2225 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2189 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2154 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2118 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2082 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)2047 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2012 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1976 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1940 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1905 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1869 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1834 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1798 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1763 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1727 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1692 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1657 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1622 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1587 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1552 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1518 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1483 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1449 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1415 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1381 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1347 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1314 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1281 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1248 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1215 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1182 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1150 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1118 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1086 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1055 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1024 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)993 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)963 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)933 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)903 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)873 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)844 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)816 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)787 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)759 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)732 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)705 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)678 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)651 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)626 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)600 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)575 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)550 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)526 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)503 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)479 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)457 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)434 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)413 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)391 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)371 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)350 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)331 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)312 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)293 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)275 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)257 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)240 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)224 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)208 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)192 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)177 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)163 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)150 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)136 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)124 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)112 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)101 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)90 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)80 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)70 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)61 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)53 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)45 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)38 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)32 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)26 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)20 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)16 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)12 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)8 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)5 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)0 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)3 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)5 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)8 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)12 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)16 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)20 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)26 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)32 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)38 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)45 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)53 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)61 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)70 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)80 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)90 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)101 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)112 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)124 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)136 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)150 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)163 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)177 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)192 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)208 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)224 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)240 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)257 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)275 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)293 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)312 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)331 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)350 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)371 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)391 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)413 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)434 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)457 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)479 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)503 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)526 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)550 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)575 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)600 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)626 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)651 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)678 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)705 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)732 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)759 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)787 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)816 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)844 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)873 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)903 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)933 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)963 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)993 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1024 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1055 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1086 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1118 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1150 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1182 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1215 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1248 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1281 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1314 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1347 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1381 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1415 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1449 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1483 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1518 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1552 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1587 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1622 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1657 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1692 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1727 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1763 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1798 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1834 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1869 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1905 / NOISE_REDUCTION_FACTOR),
  ( (uint16_t)1940 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)1976 / NOISE_REDUCTION_FACTOR),  ( (uint16_t)2012 / NOISE_REDUCTION_FACTOR)
};

static DACConversionGroup dac_conversion;

static uint8_t dac_state = STATE_STOPPED;

/***************************INTERNAL FUNCTIONS************************************/

/*
 * DAC error callback.
 */
static void error_cb(DACDriver *dacp, dacerror_t err) {

  (void)dacp;
  (void)err;

  chSysHalt("DAC failure");
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void dac_start(void)  {
    dac_conversion.num_channels = 1U;
    dac_conversion.end_cb = NULL;
    dac_conversion.error_cb = error_cb;
    dac_conversion.trigger = DAC_TRG(0); // Timer 6 TRGO event.

    static DACConfig dac_config;
    dac_config.init = 2048; // Set start value to half of the range.
    dac_config.datamode = DAC_DHRM_12BIT_RIGHT;

    dacStart(&DAC_USED, &dac_config);

    /* start timer for DAC trigger */
	static GPTConfig config;
    config.frequency = STM32_TIMCLK1; /* run timer at full frequency */
    config.callback = NULL;
    config.cr2 = TIM_CR2_MMS_1; /* trigger output on timer update */
    config.dier = 0U;
	
    gptStart(&TIMER_DAC, &config);

    //because when we do nothing, the speaker produces noise due to noise on the alimentation
    dac_stop();

}

void dac_play(uint16_t freq) {
	if(dac_state == STATE_STOPPED) {
		dac_state = STATE_PLAYING;
		dac_conversion.end_cb = NULL;
		dac_power_speaker(true); // Turn on audio.
		dacStartConversion(&DAC_USED, &dac_conversion, sinus_buffer, SINUS_BUFFER_SIZE);
		gptStartContinuous(&TIMER_DAC, STM32_TIMCLK1 / (freq*SINUS_BUFFER_SIZE));
	} else {
		gptChangeInterval(&TIMER_DAC, STM32_TIMCLK1 / (freq*SINUS_BUFFER_SIZE));
	}
}

void dac_play_buffer(uint16_t * buf, uint32_t size, uint32_t sampling_frequency, daccallback_t end_cb) {
  if(dac_state == STATE_STOPPED) {
    dac_state = STATE_PLAYING;
    dac_conversion.end_cb = end_cb;
    dac_power_speaker(true); // Turn on audio.
    dacStartConversion(&DAC_USED, &dac_conversion, buf, size);
    gptStartContinuous(&TIMER_DAC, STM32_TIMCLK1 / sampling_frequency);
  } else {
    gptChangeInterval(&TIMER_DAC, STM32_TIMCLK1 / sampling_frequency);
  }
}

void dac_change_bufferI(uint16_t* buf, uint32_t size, uint32_t sampling_frequency){
  gptStopTimerI(&TIMER_DAC);
  dacStopConversionI(&DAC_USED);

  dacStartConversionI(&DAC_USED, &dac_conversion, buf, size);
  gptStartContinuousI(&TIMER_DAC, STM32_TIMCLK1 / sampling_frequency);

}

void dac_stopI(void) {
  dac_power_speaker(false); // Turn off audio.
  gptStopTimerI(&TIMER_DAC);
  dacStopConversionI(&DAC_USED);
  dac_state = STATE_STOPPED;
}

void dac_stop(void) {
	dac_power_speaker(false); // Turn off audio.
  gptStopTimer(&TIMER_DAC);
  dacStopConversion(&DAC_USED);
	dac_state = STATE_STOPPED;
}

void dac_power_speaker(bool on_off){
  if(on_off){
    palClearPad(GPIOD, GPIOD_AUDIO_PWR);
  }else{
    palSetPad(GPIOD, GPIOD_AUDIO_PWR);
  }
}

/**************************END PUBLIC FUNCTIONS***********************************/

