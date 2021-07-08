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
 *
 * \author Code: Michael Bonani \n Doc: Jonathan Besuchet
 */

/*! \defgroup sound Sound
 * 
 * \section intro_sec Introduction
 * The e-puck has got a speaker on his top. This package is made to take 
 * advantage of it,
 * by playing little sample.
 *
 * \section organisation_sec Package organisation
 * The internals functions of this package are written in assembler,
 * because of speed. The sound you can play is in the file codec/e_const_sound.s
 *
 * The externals functions are located in the file codec/e_sound.c.
 * There are three functions: \ref e_init_sound(void), \ref e_play_sound(int sound_offset, int sound_length)
 * and \ref void e_close_sound(void). When you want to play a sound YOU HAVE TO call \ref e_init_sound(void)
 * at first, but only the first time.
 * \n To play a sound call \ref e_play_sound(int sound_offset, int sound_length). This function takes
 * two parameters, the first set the begining of the sound, the second set the length of the sound.
 * In fact it works like this:
 * - The "sound" which is in the file codec/e_const_sound.s is placed somewhere in
 * e-puck's memory.
 * - When you call the function \ref e_play_sound(int sound_offset, int sound_length), the words are sent
 * to the DCI one by one from the offset you have specified with the first parameter. The number
 * of words sent are specified with the second parameter.
 *
 * If you don't want to use the sound anymore call e_close_sound.
 * \warning If you call \ref void e_close_sound(void), you have to recall e\ref e_init_sound(void) to play
 * sound again.
 *
 * \section cmd_sec Sounds plage
 * In the file codec/e_const_sound.s there are 19044 words. Five sounds are
 * organized as following [begining, length]:
 * - [0, 2112]: "haa"
 * - [2116, 1760]: "spaah"
 * - [3878, 3412]: "ouah"
 * - [7294, 3732]: "yaouh"
 * - [11028, 8016]: "wouaaaaaaaah"
 *
 * Then if you want to play the "yaouh" sound from the begining to the end just
 * write this: e_play_sound(7294, 3732);
 * 
  * A little exemple which plays the five sounds one by one.
 * \code
 * #include <codec/e_sound.h>
 * #include <motor_led/e_init_port.h>
 * 
 * int main(void)
 * {
 * 	e_init_port();
 * 	e_init_sound();
 * 	while(1)
 * 	{
 * 		long i;
 * 		e_play_sound(0, 2112); //sound 1
 * 		for(i=0; i<4000000; i++) {asm("nop");}
 * 		e_play_sound(2116, 1760); //sound 2
 * 		for(i=0; i<4000000; i++) {asm("nop");}
 * 		e_play_sound(3878, 3412); //sound 3
 * 		for(i=0; i<4000000; i++) {asm("nop");}
 * 		e_play_sound(7294, 3732); //sound 4
 * 		for(i=0; i<4000000; i++) {asm("nop");}
 * 		e_play_sound(11028, 8016); //sound 5
 * 		for(i=0; i<4000000; i++) {asm("nop");}
 * 	}
 * }
 * \endcode
 *
 * \author Code: Michael Bonani \n Doc: Jonathan Besuchet
 */

#ifndef _SOUND
#define _SOUND

#ifdef __cplusplus
extern "C" {
#endif

/* functions */
void e_init_sound(void);
void e_play_sound(int sound_offset, int sound_length);
void e_close_sound(void);


/* DCI */
void e_init_dci_master(void);
void e_init_codec_slave(void);
void e_sub_dci_kickoff(int,int);

#ifdef __cplusplus
}
#endif

#endif

