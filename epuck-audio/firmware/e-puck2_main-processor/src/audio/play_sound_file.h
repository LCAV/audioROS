/*

File    : play_sound_file.h
Author  : Eliot Ferragni
Date    : 9 may 2018
REV 1.0

Functions and defines to play sounds from the sd card in the uncompressed WAV format in signed 16bits 44,1kHz mono

*/


#ifndef PLAY_SOUND_FILE
#define PLAY_SOUND_FILE

#include <ch.h>
#include <hal.h>


#define SF_ERROR 1
#define SF_OK    0

#define DEFAULT_VOLUME  10
#define VOLUME_MIN 		5	//under this, the sound is distorted.
#define VOLUME_MAX      50  //after this the sound begins to saturate.

typedef enum{
  SF_SIMPLE_PLAY = 0,	//plays the new sound file but if a file is already playing, then this order is ignored
  SF_WAIT_AND_CHANGE,	//waits (put the invocking thread in sleep) the end of the current file playing if any and plays the new one
  SF_FORCE_CHANGE,		//stops the current playing file if any and play the new one
}playSoundFileOption_t;


/**
 * @brief Starts the play_sound_file module			
 */
void playSoundFileStart(void);

/**
 * @brief 	Plays the sound file given in argument. Does nothing if the module has not been started with playSoundFileStart()
 * 			This function doesn't block the current thread. It uses it's self thread
 * 
 * @param pathToFile 	Path to the file on the sd card
 * @param option 		Behavior to change the sound file playing. (see play_melody_option_t)
 */
void playSoundFile(char* pathToFile, playSoundFileOption_t option);

/**
 * @brief Waits until the sound file playing has finished (put the invocking thread in sleep)
 * 		  Immediatly returns if nothing is playing
 */	
void waitSoundFileHasFinished(void);

/**
 * @brief 	Stops the sound file beeing played. Even if this function returns immediatly, 
 * 			the stopping of the playback is not immediate. It can take a very little time.
 * 			Use waitSoundFileHasFinished() to be sure the sound has stopped.
 */
void stopCurrentSoundFile(void);

/**
 * @brief Sets the volume output volume of the speaker. Only works for sound played with this library.
 * 
 * @param volume 	Desired volume between VOLUME_MIN and VOLUME_MAX. 
 * 					If less than VOLUME_MIN, then the volume is muted but the playback continues.
 */
void setSoundFileVolume(uint8_t volume);

#endif /* PLAY_SOUND_FILE */