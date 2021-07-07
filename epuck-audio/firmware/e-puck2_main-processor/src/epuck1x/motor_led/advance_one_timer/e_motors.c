/********************************************************************************

			Advance control motor of e-puck								
			December 2004: first version							
			Lucas Meier & Francesco Mondada 


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Francesco Mondada, Lucas Meier

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Manage the motors (with timer2)
 *
 * This module manage the motors with the agenda solution (timer2).
 *
 * A little exemple to use the motors with agenda (e-puck turn on himself)
 * \code
 * #include <p30F6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <motor_led/advance_one_timer/e_motors.h>
 * #include <motor_led/advance_one_timer/e_agenda.h>
 * 
 * int main(void)
 * {
 * 	e_init_port();
 * 	e_init_motors();
 * 	e_set_speed(-500, 500);
 * 	e_start_agendas_processing();
 * 	while(1) {}
 * }
 * \endcode
 * \sa e_agenda.h
 * \author Code: Francesco Mondada, Lucas Meier \n Doc: Jonathan Besuchet
 */
#include "e_motors.h"
#include "../motors.h"
#include <stdlib.h>

/* If powersave is enabled, the motor library will not leave 
 * the motor's phase powered when running at low speed, thus reducing the heat
 * dissipation and power consumption
 * 
 * TRESHV == the "virtual" speed when powersave is enabled, the phase will 
 * be keept on 1/TRESHV seconds.
 *
 * Powersave will only be enabled for speed < MAXV
 */


/*! \brief Initialize the motors's agendas
 *
 * This function initialize the agendas used by the motors. In fact
 * it call \ref e_activate_agenda(void (*func)(void), int cycle) function.
 * \sa e_activate_agenda
 */
void e_init_motors(void)
{
	return;
}

/*! \brief Manage the left motor speed
 *
 * This function manage the left motor speed by changing the MOTOR1
 * phases. The changing phases frequency (=> speed) is controled by
 * the agenda (throw the function \ref e_set_agenda_cycle(void (*func)(void), int cycle)).
 * \param motor_speed from -1000 to 1000 give the motor speed in steps/s,
 * positive value to go forward and negative to go backward.
 * \sa e_set_agenda_cycle
 */
void e_set_speed_left(int motor_speed)
{
	left_motor_set_speed((int16_t)motor_speed);
}

/*! \brief Manage the right motor speed
 *
 * This function manage the right motor speed by changing the MOTOR2
 * phases. The changing phases frequency (=> speed) is controled by
 * the agenda (throw the function \ref e_set_agenda_cycle(void (*func)(void), int cycle)).
 * \param motor_speed from -1000 to 1000 give the motor speed in steps/s,
 * positive value to go forward and negative to go backward.
 * \sa e_set_agenda_cycle
 */
void e_set_speed_right(int motor_speed)  // motor speed in percent
{
	right_motor_set_speed((int16_t)motor_speed);
}

/*! \brief Manage linear/angular speed
 *
 * This function manage the speed of the motors according to the 
 * desired linear and angular speed.
 * \param linear_speed	the speed in the axis of e-puck
 * \param angular_speed	the rotation speed (trigonometric)
 */
void e_set_speed(int linear_speed, int angular_speed)
{
	if(abs(linear_speed) + abs(angular_speed) > MOTOR_SPEED_LIMIT) {
		return;
	} else {
		left_motor_set_speed ((linear_speed - angular_speed));
		right_motor_set_speed((linear_speed + angular_speed));
	}
}

/*! \brief Give the number of left motor steps
 * \return The number of phases steps made since the left motor
 * is running.
 */
int e_get_steps_left(void)
{
	return left_motor_get_pos();
}

/*! \brief Set the number of left motor steps
 * \param set_steps The number of changed phases that you want set.
 */
void e_set_steps_left(int set_steps)
{
	left_motor_set_pos(set_steps);
	return;
}

/*! \brief Give the number of right motor steps
 * \return The number of phases steps made since the right motor
 * is running.
 */
int e_get_steps_right()
{
	return right_motor_get_pos();
}

/*! \brief Set the number of right motor steps
 * \param set_steps The number of changed phases that you want set.
 */
void e_set_steps_right(int set_steps)
{
	right_motor_set_pos(set_steps);
	return;
}

