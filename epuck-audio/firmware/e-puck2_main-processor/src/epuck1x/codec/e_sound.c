/********************************************************************************

			Sound module
			Version 1.0 August 2005 Michael Bonani


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2006-2007 Michael Bonani

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup sound
 * \brief Package to play basics sounds on the e-puck's speaker.
 * \n For more info look at this: \ref sound
 * \author Code: Michael Bonani \n Doc: Jonathan Besuchet
 */

#include "e_sound.h"
#include "../audio/audio_thread.h"

/*! \brief Initialize all you need to play sound on speaker
 * \warning You  must to call this function before playing a sound
 * (call it only one time).
 */
void e_init_sound(void)  // init sound module using si3000 codec
{
	return;
}

/*! \brief Play a sound
 * \param sound_nbr the begining of the sound
 * \param sound_length the length of the sound
 * \sa sound
 */
void e_play_sound(int sound_nbr, int sound_length)
{
	(void)sound_length;
	switch(sound_nbr) {
		case 0:
			dac_play(100);
			break;
		case 2116:
			dac_play(500);
			break;
		case 3878:
			dac_play(2000);
			break;
		case 7294:
			dac_play(4000);
			break;
		case 11028:
			dac_play(10000);
			break;
	}
}

/*! \brief Disable the sound
 * After that you can't play sound anymore, if you want to, you
 * have to call e_init_sound
 */
void e_close_sound(void)
{	
	dac_stop();
}
