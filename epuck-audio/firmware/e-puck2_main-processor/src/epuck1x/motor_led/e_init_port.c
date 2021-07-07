/********************************************************************************

			Initialization of port of e-puck
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
 * \brief Initialize the ports on standard configuration.
 * \author Code: Michael Bonani, Francesco Mondada, Davis Dadie \n Doc: Jonathan Besuchet
 */

#include "e_init_port.h"


/*! \brief Initialize all ports (in/out)
 *
 * Call this method to set all the standards output
 * components (LEDs, IR, camera, motors, I2C, audio) on
 * their defaults values and set their corresponding PIN
 * to "output".
 * The method also set the corresponding PIN to "input"
 * for all the standards inputs components
 * (IR receiver, selector, camera, battery level).
 */
void e_init_port(void) {
	return;
}

unsigned char isEpuckVersion1_3(void) {
    return 1; // The features available on e-puck1.3 are still available on e-puck2.
}

