#include "e_led.h"
#include "../leds.h"
//#include "e_agenda.h"

/*! \brief For the preprocessor. 
 * Comment if you want not to use the LED effects.
 */
#define LED_EFFECTS

/*
 * RGB intensity (percentage).
 */
#define RGB_INTENSITY 10

/*! \brief turn on/off the specified LED
 *
 * The e-puck has 8 red LEDs. With this function, you can
 * change the state of these LEDs.
 * \param led_number between 0 and 7
 * \param value 0 (off), 1 (on) otherwise change the state
 * \warning if led_number is other than 0-7, all leds are set
 * to the indicated value.
 */
void e_set_led(unsigned int led_number, unsigned int value)
{
	switch(led_number)
	{
		case 0:
			set_led(0, value);
			break;
		case 1: // Change only the red led of the RGB to have the same color as other "normal" leds.
			if(value >= 2) {
				toggle_rgb_led(0, RED_LED, RGB_INTENSITY);
			} else {
				set_rgb_led(0, value*RGB_INTENSITY, 0, 0);
			}
			break;
		case 2:
			set_led(1, value);
			break;
		case 3: // Change only the red led of the RGB to have the same color as other "normal" leds.
			if(value >= 2) {
				toggle_rgb_led(1, RED_LED, RGB_INTENSITY);
			} else {
				set_rgb_led(1, value*RGB_INTENSITY, 0, 0);
			}
			break;
		case 4:
			set_led(2, value);
			break;
		case 5: // Change only the red led of the RGB to have the same color as other "normal" leds.
			if(value >= 2) {
				toggle_rgb_led(2, RED_LED, RGB_INTENSITY);
			} else {
				set_rgb_led(2, value*RGB_INTENSITY, 0, 0);
			}
			break;
		case 6:
			set_led(3, value);
			break;
		case 7: // Change only the red led of the RGB to have the same color as other "normal" leds.
			if(value >= 2) {
				toggle_rgb_led(3, RED_LED, RGB_INTENSITY);
			} else {
				set_rgb_led(3, value*RGB_INTENSITY, 0, 0);
			}
			break;
		default:
			for(int i=0; i<8; i++) {
				e_set_led(i, value);
			}
			break;
	}
}

/*! \brief turn off the 8 LEDs
 *
 * The e-puck has 8 green LEDs. This function turn all off.
 * \warning this function doesn't turn off "body LED" and "front LED".
 */
void e_led_clear(void)
{
	clear_leds();
}

/*! \brief turn on/off the body LED
 *
 * The e-puck has a green LED that illuminate his body. With this function,
 * you can change the state of these LED.
 * \param value 0 (off), 1 (on) otherwise change the state
 */
void e_set_body_led(unsigned int value)
{
	set_body_led(value);
}

/*! \brief turn on/off the front LED
 *
 * The e-puck has a red LED in the front. With this function, you can
 * change the state of these LED.
 * \param value 0 (off), 1 (on) otherwise change the state
 */
void e_set_front_led(unsigned int value)
{
	set_front_led(value);
}

/** \brief Change the state of all LED
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led(void)
{
	set_led(4, 2);
	// Send command to esp32 to toggle the state of all leds...comando toggle dalla parte esp32?
}

/*! \brief Change the state of LED0
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led0(void)
{
	set_led(0, 2);
}

/*! \brief Change the state of LED1
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led1(void)
{
	// Send command to esp32 to toggle the led...comando toggle dalla parte esp32?
}

/*! \brief Change the state of LED2
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led2(void)
{
	set_led(1, 2);
}

/*! \brief Change the state of LED3
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led3(void)
{
	// Send command to esp32 to toggle the led...comando toggle dalla parte esp32?
}

/*! \brief Change the state of LED4
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led4(void)
{
	set_led(2, 2);
}

/*! \brief Change the state of LED5
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led5(void)
{
	// Send command to esp32 to toggle the led...comando toggle dalla parte esp32?
}

/*! \brief Change the state of LED6
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led6(void)
{
	set_led(3, 2);
}

/*! \brief Change the state of LED7
 *
 * Callback function for an agenda.
 * \sa AgendaType
 */
void e_blink_led7(void)
{
	// Send command to esp32 to toggle the led...comando toggle dalla parte esp32?
}

/*! \brief Start blinking all LED
 *
 * \param cycle	   the number of cycle we wait before launching \ref e_blink_led(void)
 * \sa e_blink_led, e_activate_agenda
 */
void e_start_led_blinking(int cycle)
{
  //e_activate_agenda(e_blink_led, cycle);
	(void)cycle;
}

/*! \brief Stop blinking all LED
 *
 * This function use \ref e_destroy_agenda(void (*func)(void))
 * \sa e_destroy_agenda
 */
void e_stop_led_blinking(void)
{
  //e_destroy_agenda(e_blink_led);
}

/*! \brief Change the blinking speed
 *
 * This function use \ref e_set_agenda_cycle(void (*func)(void), int cycle)
 * \param cycle	   the number of cycle we wait before launching \ref e_blink_led(void)"
 * \sa e_blink_led, e_set_agenda_cycle
 */
void e_set_blinking_cycle(int cycle)
{
//	if (cycle>=0)
//		e_set_agenda_cycle(e_blink_led, cycle);
	(void)cycle;
}

//###################################################
// Artistics led effects
//###################################################

#ifdef LED_EFFECTS

/*! \brief One led is on and turn clockwise */
void snake_led(void)
{
	static unsigned char no_led = 0;
	if(no_led == 0)
	{
		e_set_led(7, 0);
		e_set_led(no_led, 1);
		no_led++;
	}
	else if(no_led == 7)
	{
		e_set_led(no_led-1, 0);
		e_set_led(no_led, 1);
		no_led = 0;
	}
	else
	{
		e_set_led(no_led-1,0);
		e_set_led(no_led, 1);
		no_led++;
	}
}

/*! \brief The leds go on from the front to the back
 and go off from the front to the back, etc */
void flow_led(void)
{
	static unsigned char no_led = 0;
	switch(no_led)
	{
		case 0: e_set_led(0, 2); break;
		case 1: e_set_led(1, 2); e_set_led(7, 2); break;
		case 2: e_set_led(2, 2); e_set_led(6, 2); break;	
		case 3: e_set_led(3, 2); e_set_led(5, 2); break;
		case 4: e_set_led(4, 2); break;
	}
	if(no_led < 4)
		no_led++;
	else
		no_led = 0;
}

/*! \brief The K2000 effect */
void k2000_led(void)
{
	static unsigned char no_led = 0;
	static unsigned char right = 1;
	if(no_led == 0 && right) {
		no_led = 1; e_set_led(0, 0); e_set_led(no_led, 1); right = 0;
	}
	else if(no_led == 0 && !right) {
		no_led = 7; e_set_led(0, 0); e_set_led(no_led, 1); right = 1;
	}
	else if(no_led == 7) {
		no_led = 0; e_set_led(7, 0); e_set_led(no_led, 1);
	}
	else if(no_led == 1) {
		no_led = 0; e_set_led(1, 0); e_set_led(no_led, 1);
	}
}

/*! \brief The right LED are indicating the right side */
void right_led(void)
{
	static unsigned char no_led = 0;
	switch(no_led)
	{
		case 0: e_set_led(0, 2); e_set_led(4, 2); break;
		case 1: e_set_led(1, 2); e_set_led(3, 2); break;
		case 2: e_set_led(2, 2); break;	
		case 3: e_led_clear();
	}
	if(no_led < 3)
		no_led++;
	else
		no_led = 0;
}

/*! \brief The left LED are indicating the left side */
void left_led(void)
{
	static unsigned char no_led = 0;
	switch(no_led)
	{
		case 0: e_set_led(0, 2); e_set_led(4, 2); break;
		case 1: e_set_led(7, 2); e_set_led(5, 2); break;
		case 2: e_set_led(6, 2); break;	
		case 3: e_led_clear();
	}
	if(no_led < 3)
		no_led++;
	else
		no_led = 0;
}
#endif

