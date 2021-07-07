#ifndef LEDS_H
#define LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

#define RGB_MAX_INTENSITY 100	//percentage

//List of the RED LEDs present on the e-puck 2
typedef enum {
	LED1,
	LED3,
	LED5,
	LED7,
	NUM_LED,
} led_name_t;

//List of the RGB LEDs present on the e-puck 2
typedef enum {
	LED2,
	LED4,
	LED6,
	LED8,
	NUM_RGB_LED,
} rgb_led_name_t;

//List of the LEDs present on each RGB LED
typedef enum {
	RED_LED,
	GREEN_LED,
	BLUE_LED,
	NUM_COLOR_LED,
} color_led_name_t;


/*! \brief Turn on/off the specified LED
 *
 * The e-puck2 has 4 red LEDs placed on front, right, back and left; these LEDs are directly controllable from the main processor (F407).
 * There are also 4 RGB LEDs placed at 45, 135, 225, 315 degrees; these LEDs are connected to the ESP32 and can be controlled thorugh SPI.
 * With this function, you can change the state of the 4 red LEDs, not the RGB LEDs.
 * \param led_number: LED1, LED3, LED5 or LED7 (LED1 is the front led, then continue clockwise)
 * \param value 0 (off), 1 (on) otherwise toggle the state
 * \warning if led_number is other than LED1-LED7, all leds are set to the indicated value.
 */
void set_led(led_name_t led_number, unsigned int value);

/*! \brief Turn off all the LEDs around the robot
 *
 * The e-puck2 has 4 red LEDs and 4 RGB LEDs placed on top of it. This function turn them all off.
 * \warning this function doesn't turn off "body LED" and "front LED".
 */
void clear_leds(void);

void set_body_led(unsigned int value); // value (0=off 1=on higher=inverse)
void set_front_led(unsigned int value); //value (0=off 1=on higher=inverse)

void get_all_rgb_state(uint8_t* values);
void toggle_rgb_led(rgb_led_name_t led_number, color_led_name_t led, uint8_t intensity);
void set_rgb_led(rgb_led_name_t led_number, uint8_t red_val, uint8_t green_val, uint8_t blue_val);


#ifdef __cplusplus
}
#endif

#endif
