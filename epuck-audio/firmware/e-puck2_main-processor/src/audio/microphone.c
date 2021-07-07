#include <string.h>
#include <ch.h>
#include <hal.h>
#include "microphone.h"
#include "mp45dt02_processing.h"


static uint16_t mic_volume[4];
static int16_t mic_last[4];
static bool mic_buffer_ready = false;

// Return the last PCM sample of the given microphone from the last 10 ms of recorded audio data.
int16_t mic_get_last(uint8_t mic) {
	if(mic < 4) {
		return mic_last[mic];
	} else {
		return 0;
	}
}

// Return the last volume computed for the given microphone.
uint16_t mic_get_volume(uint8_t mic) {
	if(mic < 4) {
		return mic_volume[mic];
	} else {
		return 0;
	}
}

// This function is called every 10 ms of recorded audio data.
// It is called twice every 10 ms because the microphones are handled in pairs (left and right, front and back).
// For each microphone there are 160 samples.
// The function compute the volume for each microphone.
static void handlePCMdata(int16_t *data, uint16_t num_samples) {
	int16_t max_value[4]={INT16_MIN}, min_value[4]={INT16_MAX};

	mic_buffer_ready = true;

	for(uint16_t i=0; i<num_samples; i+=4) {

		if(data[i + MIC_LEFT] > max_value[MIC_LEFT]) {
			max_value[MIC_LEFT] = data[i + MIC_LEFT];
		}
		if(data[i + MIC_LEFT] < min_value[MIC_LEFT]) {
			min_value[MIC_LEFT] = data[i + MIC_LEFT];
		}

		if(data[i + MIC_RIGHT] > max_value[MIC_RIGHT]) {
			max_value[MIC_RIGHT] = data[i + MIC_RIGHT];
		}
		if(data[i + MIC_RIGHT] < min_value[MIC_RIGHT]) {
			min_value[MIC_RIGHT] = data[i + MIC_RIGHT];
		}

		if(data[i + MIC_FRONT] > max_value[MIC_FRONT]) {
			max_value[MIC_FRONT] = data[i + MIC_FRONT];
		}
		if(data[i + MIC_FRONT] < min_value[MIC_FRONT]) {
			min_value[MIC_FRONT] = data[i + MIC_FRONT];
		}
		
		if(data[i + MIC_BACK] > max_value[MIC_BACK]) {
			max_value[MIC_BACK] = data[i + MIC_BACK];
		}
		if(data[i + MIC_BACK] < min_value[MIC_BACK]) {
			min_value[MIC_BACK] = data[i + MIC_BACK];
		}
		
	}

	mic_volume[MIC_RIGHT] = max_value[MIC_RIGHT] - min_value[MIC_RIGHT];
	mic_volume[MIC_LEFT] = max_value[MIC_LEFT] - min_value[MIC_LEFT];
	mic_volume[MIC_BACK] = max_value[MIC_BACK] - min_value[MIC_BACK];
	mic_volume[MIC_FRONT] = max_value[MIC_FRONT] - min_value[MIC_FRONT];
	mic_last[MIC_RIGHT] = data[MIC_BUFFER_LEN-(4-MIC_RIGHT)];
	mic_last[MIC_LEFT] = data[MIC_BUFFER_LEN-(4-MIC_LEFT)];
	mic_last[MIC_BACK] = data[MIC_BUFFER_LEN-(4-MIC_BACK)];
	mic_last[MIC_FRONT] = data[MIC_BUFFER_LEN-(4-MIC_FRONT)];

	return;
}

void mic_start(mp45dt02FullBufferCb customFullbufferCb) {

	if(I2SD2.state != I2S_STOP) {
		return;
	}

	// *******************
	// TIMER CONFIGURATION
	// *******************
	// TIM9CH1 => input, the source is the I2S2 clock.
	// TIM9CH2 => output, this is the clock for microphones, 1/2 of input clock.

    rccEnableTIM9(FALSE);
    rccResetTIM9();

    STM32_TIM9->SR   = 0; // Clear eventual pending IRQs.
    STM32_TIM9->DIER = 0; // DMA-related DIER settings => DMA disabled.

    // Input channel configuration.
    STM32_TIM9->CCER &= ~STM32_TIM_CCER_CC1E; // Channel 1 capture disabled.
    STM32_TIM9->CCMR1 &= ~STM32_TIM_CCMR1_CC1S_MASK; // Reset channel selection bits.
    STM32_TIM9->CCMR1 |= STM32_TIM_CCMR1_CC1S(1); // CH1 Input on TI1.
    STM32_TIM9->CCMR1 &= ~STM32_TIM_CCMR1_IC1F_MASK; // No filter.
    STM32_TIM9->CCER &= ~(STM32_TIM_CCER_CC1P | STM32_TIM_CCER_CC1NP); // Rising edge, non-inverted.
    STM32_TIM9->CCMR1 &= ~STM32_TIM_CCMR1_IC1PSC_MASK; // No prescaler

    // Trigger configuration.
    STM32_TIM9->SMCR &= ~STM32_TIM_SMCR_TS_MASK; // Reset trigger selection bits.
    STM32_TIM9->SMCR |= STM32_TIM_SMCR_TS(5); // Input is TI1FP1.
    STM32_TIM9->SMCR &= ~STM32_TIM_SMCR_SMS_MASK; // Reset the slave mode bits.
    STM32_TIM9->SMCR |= STM32_TIM_SMCR_SMS(7); // External clock mode 1 => clock is TI1FP1.

    // Output channel configuration (pwm mode).
    STM32_TIM9->CR1 &= ~STM32_TIM_CR1_CKD_MASK; // No clock division.
    STM32_TIM9->ARR = 1; // Output clock halved.
    STM32_TIM9->PSC = 0; // No prescaler, counter clock frequency = fCK_PSC / (PSC[15:0] + 1).
    STM32_TIM9->EGR = STM32_TIM_EGR_UG; // Enable update event to reload preload register value immediately.
    STM32_TIM9->CCER &= ~STM32_TIM_CCER_CC2E; // Channel 2 output disabled.
    STM32_TIM9->CCMR1 &= ~STM32_TIM_CCMR1_CC2S_MASK; // Reset channel selection bits => channel configured as output.
    STM32_TIM9->CCMR1 &= ~STM32_TIM_CCMR1_OC2M_MASK; // Reset channel mode bits.
    STM32_TIM9->CCMR1 |= STM32_TIM_CCMR1_OC2M(6); // PWM1 mode.
    STM32_TIM9->CCER &= ~(STM32_TIM_CCER_CC2P | STM32_TIM_CCER_CC2NP); // Active high.
    STM32_TIM9->CCR[1] = 1; // Output clock halved.
    STM32_TIM9->CCMR1 |= STM32_TIM_CCMR1_OC2PE; // Enable preload at each update event for channel 2.
    STM32_TIM9->CCER &= ~STM32_TIM_CCMR1_OC2FE; // Disable fast mode.

    // Enable channels.
    STM32_TIM9->CCER |= STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC2E;
    STM32_TIM9->CR1 |= STM32_TIM_CR1_CEN;

    // ***************************
	// I2S2 AND I2S3 CONFIGURATION
    // ***************************
    mp45dt02Config micConfig;
    memset(&micConfig, 0, sizeof(micConfig));
    if (customFullbufferCb)
        micConfig.fullbufferCb = customFullbufferCb; // Custom callback called when the buffer is filled with 10 ms of PCM data.
    else
    	micConfig.fullbufferCb = handlePCMdata; // Callback called when the buffer is filled with 10 ms of PCM data.
    mp45dt02Init(&micConfig);


}

int16_t* mic_get_buffer_ptr(void) {
	return mp45dt02BufferPtr();
}

bool mic_buffer_is_ready(void) {
	return mic_buffer_ready;
}

void mic_buffer_ready_reset(void) {
	mic_buffer_ready = false;
}

uint16_t mic_buffer_get_size(void) {
	return MIC_BUFFER_LEN*2;
}
