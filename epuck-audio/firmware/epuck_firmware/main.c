#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

#include <audio/audio_thread.h>

#define SCREEN_DEBUG 0

#define WHEEL_DISTANCE      5.35f    //cm TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

#define BUZZER_FMIN 1000
#define BUZZER_FMAX 5000
#define BUZZER_DF 125
#define SPEED 200

typedef enum states_enum_t {
	WAIT_START, RECORD, SEND, MOVE, MOVE_WAIT, MOVE_CONTINUOUS, NEXT_NOTE, WAIT_ACK,
} state_t;

state_t state = WAIT_START;

uint8_t move_started = 0;
uint8_t recording_on = 0;

uint16_t buzzerFreq = BUZZER_FMIN;

uint8_t c_in;

uint8_t buzzer_idx;

static void serial_start(void) {
	static SerialConfig ser_cfg = { 115200, 0, 0, 0, };

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void) {
	//General Purpose Timer configuration
	//timer 12 is a 16 bit timer so we can measure time
	//to about 65ms with a 1Mhz counter
	static const GPTConfig gpt12cfg = {
			1000000, /* 1MHz timer clock in order to measure uS.*/
			NULL, /* Timer callback.*/
			0, 0 };

	gptStart(&GPTD12, &gpt12cfg);
	//let the timer count to max value
	gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void) {

	halInit();
	chSysInit();
	mpu_init();

	//starts the serial communication
	serial_start();
	//starts the USB communication
	usb_start();
	//starts timer 12
	timer12_start();
	//inits the motors
	motors_init();
	//initialize the speaker
	dac_start();

	//send_tab is used to save the state of the buffer to send (double buffering)
	//to avoid modifications of the buffer while sending it
	static float send_tab[DATA_SIZE];
	uint32_t timestamp = 0;
	uint32_t tickstart = 0;

	//starts the microphones processing thread.
	//it calls the callback given in parameter when samples are ready
	mic_start(&processAudioData);

	// Welcome message
	chSequentialStreamWrite((BaseSequentialStream * ) &SD3, (uint8_t* )"STARTING MAIN EPUCK\n\r", 21);

	/* Infinite loop. */
	while (1) {
		// Detect end of move started in the state machine

		// update timer
		timestamp += GPTD12.tim->CNT;
		GPTD12.tim->CNT = 0;

		switch (state) {
		case WAIT_START:
			// heart beat like function to detect the active port on the computer
			SendStart((BaseSequentialStream *) &SD3);

			// Detect start sequence
			c_in = chSequentialStreamGet((BaseSequentialStream *) &SD3);

			switch(c_in){
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 0:
				buzzer_idx = c_in;

				if (buzzer_idx == 1) {
					dac_play(buzzerFreq);
				} else {
					dac_play(buzzer_idx * 1000);
					left_motor_set_speed(SPEED);
					right_motor_set_speed(SPEED);
				}
				state = RECORD;
				break;
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case '0':
				buzzer_idx = c_in - '0';

				if (buzzer_idx == 1) {
					dac_play(buzzerFreq);
				} else {
					dac_play(buzzer_idx * 1000);
					left_motor_set_speed(SPEED);
					right_motor_set_speed(SPEED);
				}
				state = RECORD;
				break;
			case 'm':
				state = MOVE;
				break;
			case 'o':
				state = MOVE_CONTINUOUS;
				break;
			default:
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				dac_stop();
			}

			break;
		case RECORD:
			//waits until a result must be sent to the computer
			wait_finish_record();

			//we copy the buffer to avoid conflicts
			arm_copy_f32(get_audio_buffer_ptr(ALL_OUTPUTS), send_tab, DATA_SIZE);

			state = SEND;
			break;
		case SEND:
			SendFrameToComputer((BaseSequentialStream *) &SD3, send_tab, DATA_SIZE, timestamp);

			state = WAIT_ACK;
			break;
		case WAIT_ACK:
			c_in = chSequentialStreamGet((BaseSequentialStream *) &SD3);

			switch(c_in){
			case 'a':
				if(buzzer_idx == 1){
					state = NEXT_NOTE;
				}else{
					state = RECORD;
				}
				break;
			case 'n':
				state = SEND;
				break;
			case 'x':
				state = WAIT_START;
				buzzerFreq = BUZZER_FMIN;
				break;
			case 'm':
				state = MOVE;
				break;
			case 'o':
				state = MOVE_CONTINUOUS;
				break;
			}

			break;
		case MOVE:
			dac_stop();

			left_motor_set_speed(SPEED);
			right_motor_set_speed(SPEED);
			state = MOVE_WAIT;
			tickstart = timestamp;
			break;
		case MOVE_WAIT:
			if(timestamp > (tickstart + 200000)){
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				state = WAIT_START;
			}
			break;
		case MOVE_CONTINUOUS:
			left_motor_set_speed(SPEED);
			right_motor_set_speed(SPEED);
			state = RECORD;
			break;
		case NEXT_NOTE:
			if (buzzerFreq <= BUZZER_FMAX) { //every second
				buzzerFreq += BUZZER_DF;
				dac_play(buzzerFreq);
				state = RECORD;
			} else if (buzzerFreq > BUZZER_FMAX) {
				buzzerFreq = BUZZER_FMIN;
				dac_stop();
				state = WAIT_START;
			}
			break;
		}
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
