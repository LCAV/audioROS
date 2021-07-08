/**************************************************************************
* 			Defintition of all port of the e-puck 	 	                  *
*			Version 1.0 november 2005			                          *
*			Michael Bonani, Francesco Mondada, Davis Dadie                *
*									                                      *
**************************************************************************/
/********************************************************************************

			Defintition of all port of the e-puck
			Version 1.0 november 2005
			Michael Bonani, Francesco Mondada, Davis Dadie


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005-2007 Michael Bonani, Francesco Mondada, Davis Dadie

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Define all the usefull names corresponding of e-puck's hardware.
 * \author Code: Michael Bonani, Francesco Mondada, Davis Dadie \n Doc: Jonathan Besuchet
 */

/*! \mainpage e-puck standard library documentation
 * \image html logo.gif
 * \section intro_sec Introduction
 * This project has been started at the Ecole Polytechnique Federale de Lausanne as 
 * collaboration between the Autonomous Systems Lab, the Swarm-Intelligent Systems
 * group and the Laboratory of Intelligent System.
 * \n \n An educational robot:
 * The main goal of this project is to develop a miniature mobile robot for educational
 * purposes at university level. To achieve this goal the robot needs, in our opinion,
 * the following features:
 * - Good structure. The robot should have a clean mechanical structure, simple to
 * understand. The electronics, processor structure and software have to be a good
 * example of a clean modern system. 
 * - Flexibility. The robot should cover a large spectrum of educational activities and
 * should therefore have a large potential in its sensors, processing power and
 * extensions. Potential educational fields are, for instance, mobile robotics,
 * real-time programming, embedded systems, signal processing, image or sound feature
 * extraction, human-machine interaction or collective systems. 
 * - User friendly. The robot should be small and easy to exploit on a table next to a
 * computer. It should need minimal wiring, battery operation and optimal working comfort. 
 * - Good robustness and simple maintenance. The robot should resist to student use
 * and be simple and cheap to repair. 
 * - Cheap. The robot, for large use, should be cheap (450-550 euros) 
 * \section brief_sec Documentation organization
 * This documentation is divided in five sections (as you can see on the top of the page):
 * - Main Page: The startup page.
 * - Modules: An overview of all the modules that compose this library. Here you can see
 * all the files containing by each module and a detailed description of each module. Look
 * at these pages to have a better idea of what each module is doing.
 * - Data Structures: Here are listed all the C-struct of the library.
 * - Files: All the library's files listed by alphabetical order.
 * - Directories: The directories architectures of the library.
 * \section link_sec External links
 * - http://www.e-puck.org/                 The official site of the e-puck
 * - https://gna.org/projects/e-puck/       The developers area at gna
 * - http://lsro.epfl.ch/                   The site of the lab where the e-puck has been created
 * - http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45 The license
 */

/*! \defgroup motor_LED Ports, motors and LEDs
 * 
 * \section intro_sec_motors Introduction
 * This package contains all the resources you need to control the ports,
 * the motors, the LED and the IR receiver of the e-puck. 
 * 
 * \subsection intro_subsec1_ports Ports
 * The standard port's name of the p30F6014A microcontroller is not explicit in the
 * e-puck context, so we need to redefine these names to make them more user friendly.
 * \n This work is made in the file: e_epuck_ports.h.
 * 
 * \subsection intro_subsec2_motors Motors
 * The e-puck has two step by step motors called MOTOR1 (left) and MOTOR2 (right).
 * To control the changing phase's sequence of these motors we need to use timers.
 * Four possibilities are offered to you:
 * - standard: we use the timer4 for MOTOR2 and timer5 for MOTOR1. This solution is
 *   exploited by the file library\motor_led\e_motors.c.
 * - one timer standard: we use the timer3 for both MOTOR1 and MOTOR2. This solution
 *   is exploited by the file library\motor_led\e_motors_timer3.c
 * - advance one timer: we use the timer2 for both MOTOR1 and MOTOR2, but this time
 *   the mechanism work on the agenda method (see below or e_agenda.h
 *   for more information about agenda). This solution is exploited by the file
 *   library\motor_led\advance_one_timer\e_motors.c.
 * - fast agenda: we use the timer1,2,3 for both MOTOR1 and MOTOR2, but this time
 *   the mechanism work on the fast_agenda method (see below or e_agenda_fast.h
 *   for more information about fast_agenda). This solution is exploited by the file
 *   library\motor_led\advance_one_timer\fast_agenda\e_motors.c.
 *
 * \subsection intro_subsec3_led LED
 * The e-puck has 8 reds LEDs, a front LED and a body LED. All the functions needed
 * to control these LEDs are in the file library\motor_led\e_led.c. This file is
 * made for basics use. If you want blinking functions
 * you have to work with these following files: library\motor_led\advance_one_timer\e_led.c
 * or library\motor_led\advance_one_timer\fast_agenda\e_led.c. In the case you will
 * work with agenda solution (see below or e_agenda.h or e_agenda_fast.h for more 
 * information about agenda or fast agenda).
 *
 * \subsection intro_subsec4_ir IR remote
 * The e-puck has a IR receptor. To control this receptor look at this file: 
 * library\motor_led\advance_one_timer\e_remote_control.c.
 * \warning The IR remote uses the agenda solution, then it use timer2 (see below or
 * e_agenda.h for more information about agenda).
 *
 * \section timer_sect_timer Timer's problems
 * The p30F6014A microcontroller has five timers. The camera's package uses the
 * timer4 and the timer5, so we can't exploit them to make the motors work when we
 * want to use the camera. For this reason we can't use the standard solution above.
 * \warning If you are using the camera, you have to work with one of this three
 * solutions explained above:
 * - one timer standard
 * - advance one timer
 * - fast agenda
 *
 * \section agenda_sect_agenda Agenda solution
 * As we have seen, we can use the agenda solution to make the motors work.
 * \n \n So what is an agenda ?
 * \n An agenda is a structure which can launch a specific function (called callback
 * function) with a given intervals. The agenda structure is made to work as
 * chained list.
 * \n \n How it works ?
 * \n You create an agenda by specifying:
 * - the callback function you will call
 * - the delay between two calls
 * - the next element of the chained list
 *
 * On each timer overflow all the agenda chained list is scanned and if an agenda
 * in the list has reached the delay, then the callback function is called.
 * \sa e_agenda.c, e_agenda_fast.c
 * 
 * \author Doc: Jonathan Besuchet
 */

#ifndef _EPUCK_PORTS
#define _EPUCK_PORTS

#include <hal.h>

#define BATT_LOW 0 	// We don't have a pin indicating the battery level on e-puck2, thus give it a fixed value of 0.
					// This shouldn't be used, but is defined only for legacy software compilation.

#define SELECTOR0 palReadPad(GPIOC, GPIOC_SEL_0)
#define SELECTOR1 palReadPad(GPIOC, GPIOC_SEL_1)
#define SELECTOR2 palReadPad(GPIOC, GPIOC_SEL_2)
#define SELECTOR3 palReadPad(GPIOD, GPIOD_SEL_3)

#endif
