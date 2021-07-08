#include <ch.h>
#include <hal.h>

void panic_handler(const char *reason)
{
    (void)reason;

	palClearPad(GPIOD, GPIOD_LED1);
	palClearPad(GPIOD, GPIOD_LED3);
	palClearPad(GPIOD, GPIOD_LED5);
	palClearPad(GPIOD, GPIOD_LED7);
	palClearPad(GPIOD, GPIOD_LED_FRONT);
	palClearPad(GPIOB, GPIOB_LED_BODY);
	
    while (true) {

    }
}
