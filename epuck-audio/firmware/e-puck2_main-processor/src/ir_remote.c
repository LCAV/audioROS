/*

File    : ir_remote.c
Author  : Eliot Ferragni
Date    : 8 january 2018
REV 1.0

Functions to read RC5 protocol over IR

Library taken from https://github.com/guyc/RC5
and adapted to work here
*/

#include <ch.h>
#include <hal.h>
#include "ir_remote.h"
#include "exti.h"
#include <main.h>
#include "motors.h"

#include "shell.h"
#include "chprintf.h"
#include "usbcfg.h"

#define DEFAULT_SPEED 600

static ir_remote_msg_t ir_remote_values;

// /***************************INTERNAL FUNCTIONS************************************/

#define MIN_SHORT  444
#define MAX_SHORT 1333
#define MIN_LONG  1334
#define MAX_LONG  2222

/* 
 * These step by two because it makes it
 * possible to use the values as bit-shift counters
 * when making state-machine transitions.  States
 * are encoded as 2 bits, so we step by 2.
 */
#define EVENT_SHORTSPACE  0
#define EVENT_SHORTPULSE  2
#define EVENT_LONGSPACE   4
#define EVENT_LONGPULSE   6
 
#define STATE_START1 0
#define STATE_MID1   1
#define STATE_MID0   2
#define STATE_START0 3

/*
 * definitions for parsing the bitstream into 
 * discrete parts.  14 bits are parsed as:
 * [S1][S2][T][A A A A A][C C C C C C]
 * Bits are transmitted MSbit first.
 */
#define S2_MASK       0x1000  // 1 bit
#define S2_SHIFT      12
#define TOGGLE_MASK   0x0800  // 1 bit 
#define TOGGLE_SHIFT  11
#define ADDRESS_MASK  0x7C0  //  5 bits
#define ADDRESS_SHIFT 6
#define COMMAND_MASK  0x003F //  low 6 bits
#define COMMAND_SHIFT 0


/* trans[] is a table of transitions, indexed by
 * the current state.  Each byte in the table 
 * represents a set of 4 possible next states,
 * packed as 4 x 2-bit values: 8 bits DDCCBBAA,
 * where AA are the low two bits, and 
 *   AA = short space transition
 *   BB = short pulse transition
 *   CC = long space transition
 *   DD = long pulse transition
 *
 * If a transition does not change the state, 
 * an error has occured and the state machine should
 * reset.
 *
 * The transition table is:
 * 00 00 00 01  from state 0: short space->1
 * 10 01 00 01  from state 1: short pulse->0, long pulse->2
 * 10 01 10 11  from state 2: short space->3, long space->1
 * 11 11 10 11  from state 3: short pulse->2
 */
const unsigned char trans[] = {0x01,
                               0x91,  
                               0x9B,  
                               0xFB};



 /**
 * @brief   Resets the counter of a GPT
 * 
 * @param gptp			Pointer to the GPT driver
 * 
 */
static void reset_counter_gpt(GPTDriver *gptp){
	gptp->tim->CNT = 0;
}

 /**
 * @brief   Resets a RC5 decoder state machine
 * 
 * @param ir			Structure of the RC5 decoder state machine.
 * 
 */
static void RC5_reset(RC5_t* ir)
{
    ir->state = STATE_MID1;
    ir->bits = 1;  // emit a 1 at start - see state machine graph
    ir->command = 1;
    ir->time0 = 0;
}

 /**
 * @brief   Decodes a RC5 event.
 * 
 * @param ir			Structure of the RC5 decoder state machine.
 * @param event			Event detected from the IR sensor
 * 
 */
static void RC5_decodeEvent(RC5_t* ir, unsigned char event)
{
    // find next state, 2 bits
    unsigned char newState = (trans[ir->state]>>event) & 0x3;
    if (newState==ir->state) {
        // no state change indicates error, reset
        RC5_reset(ir);
    } else {
        ir->state = newState;
        if (newState == STATE_MID0) {
            // always emit 0 when entering mid0 state
            ir->command = (ir->command<<1)+0;
            ir->bits++;
        } else if (newState == STATE_MID1) {
            // always emit 1 when entering mid1 state
            ir->command = (ir->command<<1)+1;
            ir->bits++;
        }
    }
}

 /**
 * @brief   Decodes a RC5 pulse.
 * 
 * @param ir			Structure of the RC5 decoder state machine.
 * @param signal		Value read from the IR sensor
 * @param period		Time in uS between the last read on this one
 * 
 */
static void RC5_decodePulse(RC5_t* ir, unsigned char signal, unsigned long period)
{
    if (period >= MIN_SHORT && period <= MAX_SHORT) {
        RC5_decodeEvent(ir, signal ? EVENT_SHORTPULSE : EVENT_SHORTSPACE);
    } else if (period >= MIN_LONG && period <= MAX_LONG) {
        RC5_decodeEvent(ir, signal ? EVENT_LONGPULSE : EVENT_LONGSPACE);
    } else {
        // time period out of range, reset
        RC5_reset(ir);
    }
}


 /**
 * @brief   One step of the RC5 decoder state machine.
 * 
 * @param ir			Structure of the RC5 decoder state machine.
 * @param message		Pointer to store the complete message
 * 
 * @return				True if an order is complete and can be read, False otherwise
 * 
 */
static bool RC5_read_message(RC5_t* ir, unsigned int *message)
{
    /* Note that the input value read is inverted from the theoretical signal,
       ie we get 1 while no signal present, pulled to 0 when a signal is detected.
       So when the value changes, the inverted value that we get from reading the pin
       is equal to the theoretical (uninverted) signal value of the time period that
       has just ended.
    */
    unsigned long value = palReadPad(GPIOD, GPIOD_REMOTE);
    
    if (value != ir->lastValue) {
        unsigned long time1 = gptGetCounterX(&GPTD11);
        unsigned long elapsed = time1-ir->time0;
        //ir->time0 = time1;
        ir->lastValue = value;
        reset_counter_gpt(&GPTD11);
        RC5_decodePulse(ir, value, elapsed);
    }
    
    if (ir->bits == 14) {
        *message = ir->command;
        ir->command = 0;
        ir->bits = 0;
        return true;
    } else {
        return false;
    }
}

 /**
 * @brief   Performs one step of the RC5 decoder state machine.
 * 
 * @param ir			Structure of the RC5 decoder state machine.
 * @param toggle		Pointer to store the toggle variable.
 * @param address		Pointer to store the address variable.
 * @param command		Pointer to store the command variable.
 * 
 * @return				True if an order is complete and can be read, False otherwise
 * 
 */
static bool RC5_read(RC5_t* ir, unsigned char *toggle, unsigned char *address, unsigned char *command)
{
    unsigned int message;
    if (RC5_read_message(ir, &message)) {
        *toggle  = (message & TOGGLE_MASK ) >> TOGGLE_SHIFT;
        *address = (message & ADDRESS_MASK) >> ADDRESS_SHIFT;
        
        // Support for extended RC5:
        // to get extended command, invert S2 and shift into command's 7th bit
        unsigned char extended;
        extended = (~message & S2_MASK) >> (S2_SHIFT - 7);
        *command = ((message & COMMAND_MASK) >> COMMAND_SHIFT) | extended;
        
        return true;
    } else {
        return false;
    }
}

//General Purpose Timer configuration	
static const GPTConfig gpt11cfg = {
	1000000,		/* 1MHz timer clock in order to measure uS.*/
	NULL,			/* Timer callback.*/
	0,
	0
};

 /**
 * @brief  	Thread which interprets the order received and executes it
 */
static THD_FUNCTION(remote_motion_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);
    systime_t time;
    messagebus_topic_t *ir_remote_topic = messagebus_find_topic_blocking(&bus, "/ir_remote");
    ir_remote_msg_t remote_msg;

    while(1) {

    	time = chVTGetSystemTime();

    	messagebus_topic_wait(ir_remote_topic, &remote_msg, sizeof(remote_msg));

		switch(remote_msg.data) {
			// Sometimes there are two cases for the same command because two different
			// remote controls are used; one of this do not contain "numbers".
			case 5: // stop motors
			case 12:
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				break;

			case 2: // Both motors forward.
			case 32:
				left_motor_set_speed(DEFAULT_SPEED);
				right_motor_set_speed(DEFAULT_SPEED);
				break;

			case 8: // Both motors backward.
			case 33:
				left_motor_set_speed(-DEFAULT_SPEED);
				right_motor_set_speed(-DEFAULT_SPEED);
				break;

			case 6: // Both motors right.
			case 16:
				left_motor_set_speed(DEFAULT_SPEED);
				right_motor_set_speed(-DEFAULT_SPEED);
				break;

			case 4: // Both motors left.
			case 17:
				left_motor_set_speed(-DEFAULT_SPEED);
				right_motor_set_speed(DEFAULT_SPEED);
				break;

			case 3: // Left motor forward.
				left_motor_set_speed(DEFAULT_SPEED);
                right_motor_set_speed(0);
				break;

			case 1: // Right motor forward.
                left_motor_set_speed(0);
				right_motor_set_speed(DEFAULT_SPEED);
				break;

			case 9: // Left motor backward.
				left_motor_set_speed(-DEFAULT_SPEED);
                right_motor_set_speed(0);
				break;

			case 7: // Right motor backward.
                left_motor_set_speed(0);
				right_motor_set_speed(-DEFAULT_SPEED);
				break;

		}

		chThdSleepUntilWindowed(time, time + MS2ST(200)); // Receive commands at most @ 5 Hz.
    }

}

 /**
 * @brief  	Thread which looks for an EXTI_EVENT_IR_REMOTE_INT and 
 * 			if one occurs, go to the next step of the state machine
 * 			responsible of the interpretation of the ir command
 */
static THD_FUNCTION(remote_cmd_recv_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    event_listener_t remote_int;
    /* Starts waiting for the external interrupts. */
    chEvtRegisterMaskWithFlags(&exti_events, &remote_int,
                               (eventmask_t)EXTI_EVENT_IR_REMOTE_INT,
                               (eventflags_t)EXTI_EVENT_IR_REMOTE_INT);

    RC5_t remote;
    RC5_reset(&remote);

    // Declares the topic on the bus.
    messagebus_topic_t ir_remote_topic;
    MUTEX_DECL(ir_remote_topic_lock);
    CONDVAR_DECL(ir_remote_topic_condvar);
    messagebus_topic_init(&ir_remote_topic, &ir_remote_topic_lock, &ir_remote_topic_condvar, &ir_remote_values, sizeof(ir_remote_values));
    messagebus_advertise_topic(&bus, &ir_remote_topic, "/ir_remote");

    while(1) {
    	/* Wait for a command to come. */
    	chEvtWaitAny(EXTI_EVENT_IR_REMOTE_INT);
    	//Clears the flag. Otherwise the event is always true
    	chEvtGetAndClearFlags(&remote_int);

    	//do one step of the state machine. return true if an order is complete
    	if (RC5_read(&remote, &ir_remote_values.toggle, &ir_remote_values.address, &ir_remote_values.data))
		{
        	messagebus_topic_publish(&ir_remote_topic, &ir_remote_values, sizeof(ir_remote_values));
			//chprintf((BaseSequentialStream *)&SDU1, "command = %d\n", (int)ir_remote_values.data);
		}
    }
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void ir_remote_start(void) {
	gptStart(&GPTD11, &gpt11cfg);
	gptStartContinuous(&GPTD11, 0xFFFF);

	static THD_WORKING_AREA(remote_motion_thd_wa, 128);
	chThdCreateStatic(remote_motion_thd_wa, sizeof(remote_motion_thd_wa), NORMALPRIO, remote_motion_thd, NULL);

	static THD_WORKING_AREA(remote_cmd_recv_thd_wa, 256);
	chThdCreateStatic(remote_cmd_recv_thd_wa, sizeof(remote_cmd_recv_thd_wa), NORMALPRIO, remote_cmd_recv_thd, NULL);
}

uint8_t ir_remote_get_toggle(void) {
	return ir_remote_values.toggle;
}

uint8_t ir_remote_get_address(void) {
	return ir_remote_values.address;
}

uint8_t ir_remote_get_data(void) {
	return ir_remote_values.data;
}

/**************************END PUBLIC FUNCTIONS***********************************/
