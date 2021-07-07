#include <hal.h>
#include "button.h"

#define PRESSED 1
#define RELEASED 0

static uint8_t button_state = RELEASED;

uint8_t button_get_state(void) {
	return button_state;
}

void button_set_state(uint8_t state) {
	button_state = state;
}

uint8_t button_is_pressed(void) {
	return button_state==PRESSED;
}
