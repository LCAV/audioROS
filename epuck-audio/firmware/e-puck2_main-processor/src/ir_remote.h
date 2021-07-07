#ifndef IR_REMOTE_H
#define IR_REMOTE_H

#include <stdint.h>
#include <hal.h>

/** Message reprensenting a command received from the ir remote control. */
typedef struct {
	uint8_t toggle;
	uint8_t address;
	uint8_t data;
} ir_remote_msg_t;

//RC5 decoder state machine
typedef struct{
    unsigned char pin;
    unsigned char state;
    unsigned long time0;
    unsigned long lastValue;
    unsigned int bits;
    unsigned int command;
}RC5_t;



/**
 * \brief Starts the IR remote process.
 */
void ir_remote_start(void);

/**
 * \brief Return the last toggle bit.
 * \return toggle toggle bit of the signal
 */
uint8_t ir_remote_get_toggle(void);

/**
 * \brief Return the address of the last command.
 * \return address address part of the signal
 */
uint8_t ir_remote_get_address(void);

/**
 * \brief Return the data of the last command.
 * \return data_ir data part of the signal
 */
uint8_t ir_remote_get_data(void);

#endif /* IR_REMOTE_H */
