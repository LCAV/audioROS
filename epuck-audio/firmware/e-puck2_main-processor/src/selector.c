#include <hal.h>
#include "selector.h"

int get_selector() {
	return palReadPad(GPIOC, GPIOC_SEL_0) + 2*palReadPad(GPIOC, GPIOC_SEL_1) + 4*palReadPad(GPIOC, GPIOC_SEL_2) + 8*palReadPad(GPIOD, GPIOD_SEL_3);
}

