#include <ch.h>
#include <hal.h>
#include "audio_thread.h"

#define STATE_STOPPED     0
#define STATE_PLAYING     1
#define SINUS_BUFFER_SIZE 360

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
#if 0
static const dacsample_t sinus_buffer[SINUS_BUFFER_SIZE] = {
  2047, 2082, 2118, 2154, 2189, 2225, 2260, 2296, 2331, 2367, 2402, 2437,
  2472, 2507, 2542, 2576, 2611, 2645, 2679, 2713, 2747, 2780, 2813, 2846,
  2879, 2912, 2944, 2976, 3008, 3039, 3070, 3101, 3131, 3161, 3191, 3221,
  3250, 3278, 3307, 3335, 3362, 3389, 3416, 3443, 3468, 3494, 3519, 3544,
  3568, 3591, 3615, 3637, 3660, 3681, 3703, 3723, 3744, 3763, 3782, 3801,
  3819, 3837, 3854, 3870, 3886, 3902, 3917, 3931, 3944, 3958, 3970, 3982,
  3993, 4004, 4014, 4024, 4033, 4041, 4049, 4056, 4062, 4068, 4074, 4078,
  4082, 4086, 4089, 4091, 4092, 4093, 4094, 4093, 4092, 4091, 4089, 4086,
  4082, 4078, 4074, 4068, 4062, 4056, 4049, 4041, 4033, 4024, 4014, 4004,
  3993, 3982, 3970, 3958, 3944, 3931, 3917, 3902, 3886, 3870, 3854, 3837,
  3819, 3801, 3782, 3763, 3744, 3723, 3703, 3681, 3660, 3637, 3615, 3591,
  3568, 3544, 3519, 3494, 3468, 3443, 3416, 3389, 3362, 3335, 3307, 3278,
  3250, 3221, 3191, 3161, 3131, 3101, 3070, 3039, 3008, 2976, 2944, 2912,
  2879, 2846, 2813, 2780, 2747, 2713, 2679, 2645, 2611, 2576, 2542, 2507,
  2472, 2437, 2402, 2367, 2331, 2296, 2260, 2225, 2189, 2154, 2118, 2082,
  2047, 2012, 1976, 1940, 1905, 1869, 1834, 1798, 1763, 1727, 1692, 1657,
  1622, 1587, 1552, 1518, 1483, 1449, 1415, 1381, 1347, 1314, 1281, 1248,
  1215, 1182, 1150, 1118, 1086, 1055, 1024,  993,  963,  933,  903,  873,
   844,  816,  787,  759,  732,  705,  678,  651,  626,  600,  575,  550,
   526,  503,  479,  457,  434,  413,  391,  371,  350,  331,  312,  293,
   275,  257,  240,  224,  208,  192,  177,  163,  150,  136,  124,  112,
   101,   90,   80,   70,   61,   53,   45,   38,   32,   26,   20,   16,
    12,    8,    5,    3,    2,    1,    0,    1,    2,    3,    5,    8,
    12,   16,   20,   26,   32,   38,   45,   53,   61,   70,   80,   90,
   101,  112,  124,  136,  150,  163,  177,  192,  208,  224,  240,  257,
   275,  293,  312,  331,  350,  371,  391,  413,  434,  457,  479,  503,
   526,  550,  575,  600,  626,  651,  678,  705,  732,  759,  787,  816,
   844,  873,  903,  933,  963,  993, 1024, 1055, 1086, 1118, 1150, 1182,
  1215, 1248, 1281, 1314, 1347, 1381, 1415, 1449, 1483, 1518, 1552, 1587,
  1622, 1657, 1692, 1727, 1763, 1798, 1834, 1869, 1905, 1940, 1976, 2012
};

#else
static const dacsample_t sinus_buffer[SINUS_BUFFER_SIZE] = {
  2047/20, 2082/20, 2118/20, 2154/20, 2189/20, 2225/20, 2260/20, 2296/20, 2331/20, 2367/20, 2402/20, 2437/20,
  2472/20, 2507/20, 2542/20, 2576/20, 2611/20, 2645/20, 2679/20, 2713/20, 2747/20, 2780/20, 2813/20, 2846/20,
  2879/20, 2912/20, 2944/20, 2976/20, 3008/20, 3039/20, 3070/20, 3101/20, 3131/20, 3161/20, 3191/20, 3221/20,
  3250/20, 3278/20, 3307/20, 3335/20, 3362/20, 3389/20, 3416/20, 3443/20, 3468/20, 3494/20, 3519/20, 3544/20,
  3568/20, 3591/20, 3615/20, 3637/20, 3660/20, 3681/20, 3703/20, 3723/20, 3744/20, 3763/20, 3782/20, 3801/20,
  3819/20, 3837/20, 3854/20, 3870/20, 3886/20, 3902/20, 3917/20, 3931/20, 3944/20, 3958/20, 3970/20, 3982/20,
  3993/20, 4004/20, 4014/20, 4024/20, 4033/20, 4041/20, 4049/20, 4056/20, 4062/20, 4068/20, 4074/20, 4078/20,
  4082/20, 4086/20, 4089/20, 4091/20, 4092/20, 4093/20, 4094/20, 4093/20, 4092/20, 4091/20, 4089/20, 4086/20,
  4082/20, 4078/20, 4074/20, 4068/20, 4062/20, 4056/20, 4049/20, 4041/20, 4033/20, 4024/20, 4014/20, 4004/20,
  3993/20, 3982/20, 3970/20, 3958/20, 3944/20, 3931/20, 3917/20, 3902/20, 3886/20, 3870/20, 3854/20, 3837/20,
  3819/20, 3801/20, 3782/20, 3763/20, 3744/20, 3723/20, 3703/20, 3681/20, 3660/20, 3637/20, 3615/20, 3591/20,
  3568/20, 3544/20, 3519/20, 3494/20, 3468/20, 3443/20, 3416/20, 3389/20, 3362/20, 3335/20, 3307/20, 3278/20,
  3250/20, 3221/20, 3191/20, 3161/20, 3131/20, 3101/20, 3070/20, 3039/20, 3008/20, 2976/20, 2944/20, 2912/20,
  2879/20, 2846/20, 2813/20, 2780/20, 2747/20, 2713/20, 2679/20, 2645/20, 2611/20, 2576/20, 2542/20, 2507/20,
  2472/20, 2437/20, 2402/20, 2367/20, 2331/20, 2296/20, 2260/20, 2225/20, 2189/20, 2154/20, 2118/20, 2082/20,
  2047/20, 2012/20, 1976/20, 1940/20, 1905/20, 1869/20, 1834/20, 1798/20, 1763/20, 1727/20, 1692/20, 1657/20,
  1622/20, 1587/20, 1552/20, 1518/20, 1483/20, 1449/20, 1415/20, 1381/20, 1347/20, 1314/20, 1281/20, 1248/20,
  1215/20, 1182/20, 1150/20, 1118/20, 1086/20, 1055/20, 1024/20,  993/20,  963/20,  933/20,  903/20,  873/20,
   844/20,  816/20,  787/20,  759/20,  732/20,  705/20,  678/20,  651/20,  626/20,  600/20,  575/20,  550/20,
   526/20,  503/20,  479/20,  457/20,  434/20,  413/20,  391/20,  371/20,  350/20,  331/20,  312/20,  293/20,
   275/20,  257/20,  240/20,  224/20,  208/20,  192/20,  177/20,  163/20,  150/20,  136/20,  124/20,  112/20,
   101/20,   90/20,   80/20,   70/20,   61/20,   53/20,   45/20,   38/20,   32/20,   26/20,   20/20,   16/20,
    12/20,    8/20,    5/20,    3/20,    2/20,    1/20,    0/20,    1/20,    2/20,    3/20,    5/20,    8/20,
    12/20,   16/20,   20/20,   26/20,   32/20,   38/20,   45/20,   53/20,   61/20,   70/20,   80/20,   90/20,
   101/20,  112/20,  124/20,  136/20,  150/20,  163/20,  177/20,  192/20,  208/20,  224/20,  240/20,  257/20,
   275/20,  293/20,  312/20,  331/20,  350/20,  371/20,  391/20,  413/20,  434/20,  457/20,  479/20,  503/20,
   526/20,  550/20,  575/20,  600/20,  626/20,  651/20,  678/20,  705/20,  732/20,  759/20,  787/20,  816/20,
   844/20,  873/20,  903/20,  933/20,  963/20,  993/20, 1024/20, 1055/20, 1086/20, 1118/20, 1150/20, 1182/20,
  1215/20, 1248/20, 1281/20, 1314/20, 1347/20, 1381/20, 1415/20, 1449/20, 1483/20, 1518/20, 1552/20, 1587/20,
  1622/20, 1657/20, 1692/20, 1727/20, 1763/20, 1798/20, 1834/20, 1869/20, 1905/20, 1940/20, 1976/20, 2012/20,
};
#endif

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

