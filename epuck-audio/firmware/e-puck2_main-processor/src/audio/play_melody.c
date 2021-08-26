/*

File    : play_melody.c
Author  : Eliot Ferragni
Date    : 4 january 2018
REV 1.0

Functions and defines to play little melodies on the speaker

Adapted from the code written by Dipto Pratyaksa
taken at https://www.princetronics.com/supermariothemesong/
*/


#include "play_melody.h"
#include "audio_thread.h"
#include "play_sound_file.h"


//conditional variable
static MUTEX_DECL(play_melody_lock);
static CONDVAR_DECL(play_melody_condvar);

//reference
static thread_reference_t play_melody_ref = NULL;

//variable to stop the playing if necessary
static bool play = true;

//Mission : Impossible main theme
static const uint16_t mission_impossible_melody[] = {
    NOTE_G4, 0, 0, NOTE_G4,
    0, 0, NOTE_AS4, 0 ,
    NOTE_C5, 0, NOTE_G4, 0,
    0, NOTE_G4, 0, 0,
    NOTE_F4, 0, NOTE_FS4, 0,

    NOTE_G4, 0, 0, NOTE_G4,
    0, 0, NOTE_AS4, 0 ,
    NOTE_C5, 0, NOTE_G4, 0,
    0, NOTE_G4, 0, 0,
    NOTE_F4, 0, NOTE_FS4, 0,

    NOTE_AS5, NOTE_G5, NOTE_D5,0,
    0,0,0,0,
    0,0, NOTE_AS5, NOTE_G5,
    NOTE_CS5,0, 0, 0,
    0, 0, 0, 0,
    NOTE_AS5, NOTE_G5, NOTE_C5,0,
    0, 0, 0, 0,
    0, 0, NOTE_AS4, NOTE_C5,
    0, 0, 0, 0,
    0, 0, 0, 0

};

//Mission : Impossible main tempo
static const float mission_impossible_tempo[] = {
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,

      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,

      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,

      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12,
      12, 12, 12, 12

};

//We are the champions melody
static const uint16_t champions_melody[] = {
    NOTE_F5,
    NOTE_E5, NOTE_F5,
    NOTE_E5,
    NOTE_C5,
    0, NOTE_A4,

    NOTE_D5,
    NOTE_A4,
    NOTE_A3, 0, NOTE_G3, 0,
    NOTE_F3,

    0, 0, NOTE_G3,
    NOTE_C5,
    NOTE_F5,

    NOTE_G5, NOTE_A5,
    NOTE_C6,
    NOTE_A5,
    NOTE_D5, NOTE_E5,

    NOTE_D5,
    0,  NOTE_D4,
    NOTE_C4, NOTE_AS3,
    NOTE_A3,

    0, 0,
    NOTE_D5,
    NOTE_C5,
    NOTE_D5,
    NOTE_C5,

    NOTE_AS4,
    NOTE_AS5,
    NOTE_A5,
    NOTE_AS5,
    NOTE_A5,
    NOTE_G5,

    NOTE_A5,
    NOTE_F5,
    NOTE_AS5,

    NOTE_A5,
    NOTE_F5,
      NOTE_AS5,

    NOTE_GS5,
    NOTE_F5,
    NOTE_AS5,

    NOTE_GS5,
    NOTE_F5,

    0,
    NOTE_DS5,  NOTE_C5,
    NOTE_F5
};

//we are the champions tempo
static const float champions_tempo[] = {
  2.25,
  9, 9,
  4.5,
  4.5,
  9, 9,

  4.5,
  4.5,
  18, 18, 18, 18,
  4.5,

  18, 18, 4.5,
  9,
  2.25,

  9, 9,
  4.5,
  4.5,
  9, 9,

  4.5,
  9, 9,
  9, 9,
  4.5,

  4.5, 4.5,
  3,
  4.5,
  9,
  3,

  3,
  3,
  4.5,
  9,
  3,
  3,

  3,
  4.5,
  9,

  3,
  4.5,
  9,

  3,
  4.5,
  9,

  3,
  3,

  2.25,
  9, 9,
  2.25

};

static const uint16_t russia_melody[] = {
  0, 0, NOTE_AS4,

  NOTE_DS5,
  NOTE_AS4, NOTE_C5,
  NOTE_D5,
  NOTE_G4,

  NOTE_C5,
  NOTE_AS4, NOTE_GS4,
  NOTE_AS4,
  NOTE_DS4, NOTE_DS4,

  NOTE_F4,
  NOTE_F4, NOTE_G4,
  NOTE_GS4,
  NOTE_GS4,NOTE_AS4,

  NOTE_C5,
  NOTE_D5,NOTE_DS5,
  NOTE_F5,


};

static const float russia_tempo[] = {
  9, 9, 9,

  3,
  4.5, 9,
  3,
  3,

  3,
  4.5, 9,
  3,
  4.5, 9,

  3,
  4.5, 9,
  3,
  4.5, 9,

  3,
  4.5, 9,
  1.5,
};

//Mario main theme melody
static const uint16_t mario_melody[] = {
  NOTE_E5, NOTE_E5, 0, NOTE_E5,
  0, NOTE_C5, NOTE_E5, 0,
  NOTE_G5, 0, 0, 0,
  NOTE_G4, 0, 0, 0,
 
  NOTE_C5, 0, 0, NOTE_G4,
  0, 0, NOTE_E4, 0,
  0, NOTE_A4, 0, NOTE_B4,
  0, NOTE_AS4, NOTE_A4, 0,
 
  NOTE_G4, NOTE_E5, NOTE_G5,
  NOTE_A5, 0, NOTE_F5, NOTE_G5,
  0, NOTE_E5, 0, NOTE_C5,
  NOTE_D5, NOTE_B4, 0, 0,
 
  NOTE_C5, 0, 0, NOTE_G4,
  0, 0, NOTE_E4, 0,
  0, NOTE_A4, 0, NOTE_B4,
  0, NOTE_AS4, NOTE_A4, 0,
 
  NOTE_G4, NOTE_E5, NOTE_G5,
  NOTE_A5, 0, NOTE_F5, NOTE_G5,
  0, NOTE_E5, 0, NOTE_C5,
  NOTE_D5, NOTE_B4, 0, 0
};

//Mario main theme tempo
static const float mario_tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};

//Underworld melody
static const uint16_t underworld_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_D4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};

//Underwolrd tempo
static const float underworld_tempo[] = {
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  6, 18, 18, 18,
  6, 6,
  6, 6,
  6, 6,
  18, 18, 18, 18, 18, 18,
  10, 10, 10,
  10, 10, 10,
  3, 3, 3
};

//mario start melody
static const uint16_t mario_start_melody[] =
{
    NOTE_C5,NOTE_E5,NOTE_G5,
    NOTE_C5,NOTE_E5,NOTE_G5,
    NOTE_E5,0,
    NOTE_C5,NOTE_DS5,NOTE_FS5,
    NOTE_C5,NOTE_DS5,NOTE_FS5,
    NOTE_DS5,0,
    NOTE_D5,NOTE_F5,NOTE_AS5,
    NOTE_D5,NOTE_F5,NOTE_AS5,
    NOTE_AS5,NOTE_AS5,NOTE_AS5,
    NOTE_C6,0
};
//mario start tempo
static const float mario_start_tempo[] =
{
    16,16,16,
    16,16,16,
    12,12,
    16,16,16,
    16,16,16,
    12,12,
    16,16,16,
    16,16,16,
    19,19,19,
    10,10
};

//Mario death melody
static const uint16_t mario_death_melody[] = {
  NOTE_B3, NOTE_F4, 0, NOTE_F4, NOTE_F4,
  NOTE_E4, NOTE_D4, NOTE_C4, 
  NOTE_E3, 0,NOTE_E3, NOTE_C3
};

//Mario death tempo
static const float mario_death_tempo[] = {
  12, 12, 12, 12, 9,
  9, 9, 12, 
  12, 16, 12, 12

};

//Mario flag pole melody
static const uint16_t mario_flagpole[] = {
  NOTE_G2, NOTE_C3, NOTE_E3, NOTE_G3 ,
  NOTE_C4, NOTE_E4, NOTE_G4, NOTE_E4,
  NOTE_GS2, NOTE_C3,
  NOTE_DS3, NOTE_GS3, NOTE_C4, NOTE_DS4,
  NOTE_GS4, NOTE_DS4, NOTE_AS2, NOTE_D3, NOTE_F3,
  NOTE_AS3, NOTE_D4, NOTE_F4, NOTE_AS4,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C5
};

//Mario flag pole tempo
static const float mario_flagpole_tempo[] = {
  15, 15, 15, 15, 15, 15,  4.5, 4.5, 15, 15,
  15, 15, 15, 15, 4.5, 4.5, 15, 15, 15,
  15, 15, 15, 4.5, 15, 15, 15, 3

};

//Walking melody
static const uint16_t walking_melody[] = {
  NOTE_F3, NOTE_A3, NOTE_C3, NOTE_E3, NOTE_F3,
  NOTE_A3, NOTE_C3, NOTE_D3, NOTE_E3,
};

// Walking melody tempo
static const float walking_melody_tempo[] = {
  6,9,6,9,6,
  9,12,12,12,
};

// Pirates of the Caribbean main theme melody
static const uint16_t pirate_melody[] = {
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
  NOTE_A4, NOTE_G4, NOTE_A4, 0,

  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
  NOTE_A4, NOTE_G4, NOTE_A4, 0,

  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
  NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
  NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
  NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,

  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
  NOTE_D5, NOTE_E5, NOTE_A4, 0,
  NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
  NOTE_C5, NOTE_A4, NOTE_B4, 0,

  NOTE_A4, NOTE_A4,
  //Repeat of first part
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
  NOTE_A4, NOTE_G4, NOTE_A4, 0,

  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
  NOTE_A4, NOTE_G4, NOTE_A4, 0,

  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
  NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
  NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
  NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,

  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
  NOTE_D5, NOTE_E5, NOTE_A4, 0,
  NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
  NOTE_C5, NOTE_A4, NOTE_B4, 0,
  //End of Repeat

  NOTE_E5, 0, 0, NOTE_F5, 0, 0,
  NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
  NOTE_D5, 0, 0, NOTE_C5, 0, 0,
  NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,

  NOTE_E5, 0, 0, NOTE_F5, 0, 0,
  NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
  NOTE_D5, 0, 0, NOTE_C5, 0, 0,
  NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4
};

// Pirates of the Caribbean main theme tempo
static const float pirate_tempo[] = {
  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 3, 10,

  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 3, 10,

  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 3, 10, 10,

  10, 10, 5, 10, 10,
  5, 10, 5, 10,
  10, 10, 5, 10, 10,
  10, 10, 3, 3,

  5, 10,
  //Rpeat of First Part
  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 3, 10,

  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 3, 10,

  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 10, 10, 10,

  10, 10, 5, 10, 10,
  5, 10, 10, 10,
  10, 10, 5, 10, 10,
  10, 10, 3, 3,
  //End of Repeat

  5, 10, 5, 5, 10, 5,
  10, 10, 10, 10, 10, 10, 10, 10, 5,
  5, 10, 5, 5, 10, 5,
  10, 10, 10, 10, 10, 2,

  5, 10, 5, 5, 10, 5,
  10, 10, 10, 10, 10, 10, 10, 10, 5,
  5, 10, 5, 5, 10, 5,
  10, 10, 10, 10, 10, 2,
};

//Simpson main theme melody
static const uint16_t simpson_melody[] = {
  NOTE_D3, 0, 0, NOTE_E3,
  0, NOTE_FS3, 0, NOTE_A3,
  NOTE_G3, 0, 0, NOTE_E3,
  0, NOTE_C3, 0,NOTE_A2,

  NOTE_FS2, NOTE_FS2,
  NOTE_FS2, NOTE_G2, 0, NOTE_FS3,
  NOTE_FS3, NOTE_FS3, NOTE_G3, NOTE_AS3,
  0, 0,NOTE_B3, 0,
  0, 0, 0, 0,
};

//Simpson main theme tempo
static const float simpson_tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};

//Star Wars melody
static const uint16_t starwars_melody[] = {
  NOTE_A4, NOTE_A4, NOTE_A4,
  NOTE_F4, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4, 
  0,
  NOTE_E5, NOTE_E5, NOTE_E5,
  NOTE_F5, NOTE_C5, NOTE_GS4, NOTE_F4, NOTE_C5, NOTE_A4,
};

//Star Wars tempo
static const float starwars_tempo[] = {
	3.6, 3.6, 3.6,
	4.8, 12, 3.6, 4.8, 12, 3.6,
	2,
	3.6, 3.6, 3.6,
	4.8, 12, 3.6, 4.8, 12, 3.6
};

//Sandstorms melody
static const uint16_t sandstorms_melody[] = {
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,
  NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, 0,
  NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, 0,
  NOTE_A4, NOTE_A4,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,
  NOTE_E5, NOTE_E5,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,
};


//Sandstorms tempo
static const float sandstorms_tempo[] = {
  24, 24, 24, 24, 24, 24,
  24, 24, 24, 24, 24, 24, 24, 24,
  24, 24, 24, 24, 24, 24, 24, 24,
  24, 24, 24, 24, 24, 24, 24, 24,
  24, 24,
  24, 24, 24, 24, 24, 24,
  24, 24, 24, 24, 24, 24, 24, 24,
  24, 24,
  24, 24, 24, 24, 24, 24,
  24, 24, 24, 24, 24, 24, 24, 24,
};

//seven nation army melody by White Stripes
static const uint16_t seven_nation_army_melody[] = {
  0,
  NOTE_D4 , NOTE_D4, 0, NOTE_F4, 0, NOTE_D4, 0, NOTE_C4 , NOTE_AS3, NOTE_A3, 0,
  NOTE_D4 , NOTE_D4, 0, NOTE_F4, 0, NOTE_D4, 0, NOTE_C4 , NOTE_AS3, NOTE_A3, 0,
  NOTE_D4 , NOTE_D4, 0, NOTE_F4, 0, NOTE_D4, 0, NOTE_C4 , NOTE_AS3, NOTE_A3, 0,

};

//seven nation army tempo
static const float seven_nation_army_tempo[] = {

  12,
  3,20,20,20,9,20,9,9,2.2,2.2,70,
  3,20,20,20,9,20,9,9,2.2,2.2,70,
  3,20,20,20,9,20,9,9,2.2,2.2,70,

};


static const melody_t melody[NB_SONGS] = {
  //MISSION_IMPOSSIBLE
  {
    .notes = mission_impossible_melody,
    .tempo = mission_impossible_tempo,
    .length = sizeof(mission_impossible_melody)/sizeof(uint16_t),

  },
  //WE_ARE_THE_CHAMPIONS
  {
    .notes = champions_melody,
    .tempo = champions_tempo,
    .length = sizeof(champions_melody)/sizeof(uint16_t),
  },
  //RUSSIA
  {
    .notes = russia_melody,
    .tempo = russia_tempo,
    .length = sizeof(russia_melody)/sizeof(uint16_t),
  },
	//MARIO
	{
		.notes = mario_melody,
		.tempo = mario_tempo,
		.length = sizeof(mario_melody)/sizeof(uint16_t),
	},
	//UNDERWORLD
	{
		.notes = underworld_melody,
		.tempo = underworld_tempo,
		.length = sizeof(underworld_melody)/sizeof(uint16_t),
	},
  //MARIO_START
  {
    .notes = mario_start_melody,
    .tempo = mario_start_tempo,
    .length = sizeof(mario_start_melody)/sizeof(uint16_t),
  },
  //MARIO_DEATH
  {
    .notes = mario_death_melody,
    .tempo = mario_death_tempo,
    .length = sizeof(mario_death_melody)/sizeof(uint16_t),
  },
  //MARIO_FLAG
  {
    .notes = mario_flagpole,
    .tempo = mario_flagpole_tempo,
    .length = sizeof(mario_flagpole)/sizeof(uint16_t),
  },
  //WALKING
  {
    .notes = walking_melody,
    .tempo = walking_melody_tempo,
    .length = sizeof(walking_melody)/sizeof(uint16_t),
  },
  //PIRATES_OF_THE_CARIBBEAN
  {
    .notes = pirate_melody,
    .tempo = pirate_tempo,
    .length = sizeof(pirate_melody)/sizeof(uint16_t),
  },
  //SIMPSON
  {
    .notes = simpson_melody,
    .tempo = simpson_tempo,
    .length = sizeof(simpson_melody)/sizeof(uint16_t),
  },
	//STARWARS
	{
		.notes = starwars_melody,
		.tempo = starwars_tempo,
		.length = sizeof(starwars_melody)/sizeof(uint16_t),
	},
   //SANDSTORMS
  {
    .notes = sandstorms_melody,
    .tempo = sandstorms_tempo,
    .length = sizeof(sandstorms_melody)/sizeof(uint16_t),
  },
  //SEVEN_NATION_ARMY
  {
    .notes = seven_nation_army_melody,
    .tempo = seven_nation_army_tempo,
    .length = sizeof(seven_nation_army_melody)/sizeof(uint16_t),

  },
};

void playNote(uint16_t note, uint16_t duration_ms) {

	if(note != 0){
		dac_play(note);
	}
	chThdSleepMilliseconds(duration_ms);
	dac_stop();
}

static THD_WORKING_AREA(waPlayMelodyThd, 128);
static THD_FUNCTION(PlayMelodyThd, arg) {

  chRegSetThreadName("PlayMelody Thd");

	(void)arg;

	static melody_t* song = NULL;

	while(1){
		//this thread is waiting until it receives a message
		chSysLock();
		song = (melody_t*) chThdSuspendS(&play_melody_ref);
		chSysUnlock();

		for (int thisNote = 0; thisNote < song->length; thisNote++) {

      if(!play){
        break;
      }

			// to calculate the note duration, take one second
			// divided by the note type.
			//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
			uint16_t noteDuration = (uint16_t)(1000 / song->tempo[thisNote]);

			playNote(song->notes[thisNote], noteDuration);

			// to distinguish the notes, set a minimum time between them.
			// the note's duration + 30% seems to work well:
			uint16_t pauseBetweenNotes = (uint16_t)(noteDuration * 1.30);
			chThdSleepMilliseconds(pauseBetweenNotes);

		}
    play = false;
    //signals to the threads waiting that the melody has finished
    chCondBroadcast(&play_melody_condvar);
	}
}

void playMelodyStart(void){

	//create the thread
	chThdCreateStatic(waPlayMelodyThd, sizeof(waPlayMelodyThd), NORMALPRIO, PlayMelodyThd, NULL);
}

void playMelody(song_selection_t choice, play_melody_option_t option, melody_t* external_melody){

  melody_t* song = NULL;

  //case of an external melody provided
  if(choice == EXTERNAL_SONG && external_melody != NULL){
    song = external_melody;
  }//case of an internal melody chosen
  else if(choice < EXTERNAL_SONG){
    song = &melody[choice];
  }//if the internal melody is not valid
  else{
    return;
  }

  //stops a eventual file being played to avoid conflict
  stopCurrentSoundFile();
  waitSoundFileHasFinished();

  //SIMPLE_PLAY case
  if(option == ML_SIMPLE_PLAY){
    //if the reference is NULL, then the thread is already running
    //when the reference becomes not NULL, it means the thread is waiting
    if(play_melody_ref != NULL){
      play = true;
      //tell the thread to play the song given
      chThdResume(&play_melody_ref, (msg_t) song);
    }
  }//FORCE_CHANGE or WAIT_AND_CHANGE cases
  else{
    if(option == ML_FORCE_CHANGE){
      stopCurrentMelody();
    }
    waitMelodyHasFinished();
    play = true;
    //tell the thread to play the song given
    chThdResume(&play_melody_ref, (msg_t) song);
  }
}

void stopCurrentMelody(void){
    play = false;
}

void waitMelodyHasFinished(void) {
  //if a melody is playing
  if(play_melody_ref == NULL){
    //waits until the current melody is finished
    chMtxLock(&play_melody_lock);
    chCondWait(&play_melody_condvar);
    chMtxUnlock(&play_melody_lock);
  }
}


