/********************************************************************************

			Control IR receiver module							
			December 2005: first version							
			Valentin Longchamp 


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Valentin Longchamp

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch
**********************************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Manage the IR receiver module (timer2)
 *
 * This module manage the IR receiver with the agenda solution (timer2).
 *
 * Alittle exemple to manage the IR remote (the body LED change his state when 
 * you press a button of the IR controller).
 * \code
 * #include <p30F6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <motor_led/advance_one_timer/e_remote_control.h>
 * #include <motor_led/advance_one_timer/e_agenda.h>
 * 
 * int main(void)
 * {
 * 	int ir_check;
 * 	int previous_check = 0;
 * 	e_init_port();
 * 	e_init_remote_control();
 * 	e_start_agendas_processing();
 * 	while(1)
 * 	{
 * 		ir_check = e_get_check();
 * 		if(ir_check != previous_check)
 * 			BODY_LED = BODY_LED^1;
 * 		previous_check = ir_check;
 * 	}
 * }
 * \endcode
 * \sa e_agenda.h
 * \author Code: Francesco Mondada, Lucas Meier \n Doc: Jonathan Besuchet
 */

#include "e_remote_control.h"
#include "../ir_remote.h"


/*! \brief Initialise the IR receiver ports */
void e_init_remote_control(void) // initialisation for IR interruptions on INT0
{
	return;
}

/*------ user calls ------*/

/** \brief Read the check bit
 * \return	check	check bit of the signal
 */
unsigned char e_get_check(void) {
	return ir_remote_get_toggle();
}

/** \brief Read the adress of the commande
 * \return	adress	adress part of the signal
 */
unsigned char e_get_address(void) {
	return ir_remote_get_address();
}

/** \brief Read the data of the command
 * \return	data	data part of the signal
 */
unsigned char e_get_data(void) {
	return ir_remote_get_data();
}
