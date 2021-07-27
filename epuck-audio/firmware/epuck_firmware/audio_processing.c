#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>


//INNER FUNCTION DECLARATIONS
uint16_t data_from_0_to_256_test(uint16_t start_data);
void fill_mics_with_fake_data(void);
uint16_t calculateBinFromFrequency(uint16_t frequency);

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
/* SOUND REMOTE CODE
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
*/
//static float micAll_cmplx_output[8 * FFT_SIZE];



#define MIN_VALUE_THRESHOLD	10000



/** data that contains
 * bin number -> timestamp -> fft -> bin values
 */
static float tx_buffer[DATA_SIZE];



#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing


#define OPERATING_BUZZER_FREQUENCY 64 //around 1000Hz through calibration I find that it has to be at 124.5, or Am I somehow displacing it by three (so six values in the code?)
#define BUZZER_FREQ_500 64
#define BUFFER_TEST_MOD //add an E to MOD to switch to buffer test mode
#define MIC_RATE 16000
#define TOTAL_SAMPLES 1024

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)





/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//go forward
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
		left_motor_set_speed(600);
		right_motor_set_speed(600);
	}
	//turn left
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	//turn right
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}
	//go backward
	else if(max_norm_index >= FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(-600);
	}
	else{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

extern uint16_t buzzerFreq;

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	volatile uint16_t binPosition;
	volatile uint16_t start_i_fbin;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		//TEST FUNCTION TO PUT NUMBERS going from 0 to 2048 in the buffers of the mics for testing
		//fill_mics_with_fake_data();

		//ME : select the right audio data :
		uint16_t i_array = 0;

		//generating the package to send
		tx_buffer[i_array++] = BIN_SIZE;
		
		binPosition  = calculateBinFromFrequency(buzzerFreq);

		start_i_fbin = (binPosition > BIN_SIZE / 2 ) ? 2 *( binPosition - BIN_SIZE / 2 ) : 0;

		for(uint8_t i_fbin = 0; i_fbin < BIN_SIZE * 2; i_fbin+=2){
			tx_buffer[i_array++] = micRight_cmplx_input[start_i_fbin + i_fbin];
			tx_buffer[i_array++] = micLeft_cmplx_input[start_i_fbin + i_fbin];
			tx_buffer[i_array++] = micBack_cmplx_input[start_i_fbin + i_fbin];
			tx_buffer[i_array++] = micFront_cmplx_input[start_i_fbin + i_fbin];
			tx_buffer[i_array++] = micRight_cmplx_input[start_i_fbin + i_fbin + 1];
			tx_buffer[i_array++] = micLeft_cmplx_input[start_i_fbin + i_fbin + 1];
			tx_buffer[i_array++] = micBack_cmplx_input[start_i_fbin + i_fbin + 1];
			tx_buffer[i_array++] = micFront_cmplx_input[start_i_fbin + i_fbin + 1];
		}

		for(uint8_t i_fbin = 0; i_fbin < BIN_SIZE; i_fbin++){
			tx_buffer[i_array++] = start_i_fbin / 2 + i_fbin;
		}
		
		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/

		/* SOUND REMOTE CODE
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		*/

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 3){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		/* SOUND REMOTE CODE
		sound_remote(micLeft_output);
		*/
	}
}

void wait_finish_record(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	/* SOUND REMOTE CODE
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}*/
	//ME :
	else if(name == ALL_OUTPUTS){
		return tx_buffer;
	}
	else{
		return NULL;
	}
}


//ME : function that chooses the highest peak in magnitude and sends the bins centered around this element
/*
 * data : the magnitude result of one of the mics of size FFT_SIZE
 */
void choose_bins_magnitude(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	//int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			//max_norm_index = i;
		}
	}
	//because magnitude is only of size FFT_SIZE and the complex FFT result is of size 2 FFT_SIZE
	//int16_t start = max_norm_index * 2 - HALF_BIN ;
	for(uint16_t i = 0 ; i < BIN_SIZE ; i++){
		//add the values of each of the complex results to the new array
	}


}

uint16_t calculateBinFromFrequency(uint16_t frequency){
	if(frequency > 0){
		return (uint32_t) frequency * (uint32_t) TOTAL_SAMPLES / MIC_RATE;
	}
	return 0;
}

//DEBUG FUNCTIONS

void fill_mics_with_fake_data(void){
	for(uint16_t i = 0 ; i < FFT_SIZE * 2 ; i++){
		micRight_cmplx_input[i] = i;
		micLeft_cmplx_input[i] = i;
		micBack_cmplx_input[i] = i;
		micFront_cmplx_input[i] = i;

	}
}

uint16_t data_from_0_to_256_test(uint16_t start_data){
	uint16_t double_bin = BIN_SIZE * 2;
	uint16_t micPos = start_data;
	uint16_t testValue = 0;

	//left mic
	for(uint16_t i = 0 ; i < double_bin ; i+= 2){
		tx_buffer[i + micPos] = testValue;
		testValue++;
		tx_buffer[i + 1 + micPos] = testValue;
		testValue++;
	}
	micPos += double_bin;

	//right mic
	for(uint16_t i = 0 ; i < double_bin ; i+= 2){
		tx_buffer[i + micPos] = testValue;
		testValue++;
		tx_buffer[i + 1 + micPos] = testValue;
		testValue++;
	}
	micPos += double_bin;

	//front mic
	for(uint16_t i = 0 ; i < double_bin ; i+= 2){
		tx_buffer[i + micPos] = testValue;
		testValue++;
		tx_buffer[i + 1 + micPos] = testValue;
		testValue++;
	}
	micPos += double_bin;

	//back mic
	for(uint16_t i = 0 ; i < double_bin ; i+= 2){
		tx_buffer[i + micPos] = testValue;
		testValue++;
		tx_buffer[i + 1 + micPos] = testValue;
		testValue++;
	}
	micPos += double_bin;
	return micPos;
}


