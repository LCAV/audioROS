/*

File    : play_sound_file.c
Author  : Eliot Ferragni
Date    : 9 may 2018
REV 1.0

Functions and defines to play sounds from the sd card in the uncompressed WAV format in signed 16bits 44,1kHz mono

*/


#include <fat.h>
#include <audio/play_sound_file.h>
#include <audio/audio_thread.h>
#include "play_melody.h"

#include <chprintf.h>

#define SIZE_SOUND_BUFFER_16bits    256
#define SIZE_WAV_HEADER_BYTES       44
#define OFFSET_INT16_TO_UINT16      32768

#define SAMPLING_44100HZ            44100

//buffers to handle the data to play
static uint16_t sound_buffer1[SIZE_SOUND_BUFFER_16bits];
static uint16_t sound_buffer2[SIZE_SOUND_BUFFER_16bits];

//conditional variable
static MUTEX_DECL(play_sound_file_lock);
static CONDVAR_DECL(play_sound_file_condvar);

//semaphore to swap the buffer at the good time
static BSEMAPHORE_DECL(change_buffer, true);

//reference to play a music
static thread_reference_t play_sound_file_ref = NULL;

//variable to handle the playing
static bool play = false;
static uint8_t sound_file_volume = DEFAULT_VOLUME;

//variables to handle the buffers
static uint16_t* buffer_to_read = NULL;
static uint32_t size_buffer_to_read = 0;

//////////////////////////////////////////INTERNAL FUNCTIONS////////////////////////////////////

/*
 * DAC end callback.
 * Called when the dac is at the middle of the output buffer and at the end
 * Used to swap the buffer to output
 */
static void end_cb(DACDriver *dacp, const dacsample_t *buffer, size_t n) {

    (void)dacp;
    (void)buffer;
    (void)n;

    //to quit the callback and do nothing if we are here because of the half transfert interrupt
    //we only want the transfert finished interrupt
    if(buffer == sound_buffer1 || buffer == sound_buffer2){
        return;
    }

    //swaps the buffers to read
    if(buffer_to_read == sound_buffer1){
        buffer_to_read = sound_buffer2;
    }else{
        buffer_to_read = sound_buffer1;
    }

    //changes the buffer only if we have remaining data to send
    chSysLockFromISR();
    if(size_buffer_to_read){
        //we need to be sure the number of sample is 1 or even. BUT NOT ODD !
        //so in case of we simply remove 1
        if(size_buffer_to_read > 1 && size_buffer_to_read%2){
            size_buffer_to_read -= 1;
        }
        dac_change_bufferI(buffer_to_read, size_buffer_to_read, SAMPLING_44100HZ);
        //if we have less than the specified amount of data, it means we are at the end of the file
        //thus we set the size to 0 to stop the dac at the next iteration.
        if(size_buffer_to_read<SIZE_SOUND_BUFFER_16bits){
            size_buffer_to_read = 0;
        }
        ///tell the thread PlaySoundFileThd it can begin to fill the other buffer
        chBSemSignalI(&change_buffer);
    }else{
        dac_stopI();
    }
    chSysUnlockFromISR();
}

/**
 * @brief Function to read a wav file stored on the sd card
 * 
 * @param pathToFile Path to the file to read.
 * @return [description]
 */
uint8_t playWAVFile(char *pathToFile){
    FIL file;   /* file object */
    FRESULT err;

    UINT bytesRead = 0;
    uint8_t first_time = 1;
    uint16_t* buffer_to_fill = NULL;
    uint16_t i = 0;

    
    //Attempt to mount the drive.
    if(!mountSDCard()){
        return SF_ERROR;
    }

    //opens the sound file
    err = f_open(&file, pathToFile, FA_READ);
    if (err != FR_OK) {
        return SF_ERROR;
    }

    //initializes the pointers to the first buffer
    buffer_to_read = sound_buffer1;
    buffer_to_fill = sound_buffer1;

    //flushes the header
    //for now we don't care to read the header.
    err = f_read(&file, buffer_to_fill, SIZE_WAV_HEADER_BYTES, &bytesRead);
    if (err != FR_OK) {
        f_close(&file);
        return SF_ERROR;
    }

    //loop to read te file until we have no more data to read
    do{
        //to stop the reading
        if(!play){
            //stops the reading immediatly
            dac_stop();
            break;
        }

        //reads the data from the sd card and store them in the correct buffer
        err = f_read(&file, buffer_to_fill, SIZE_SOUND_BUFFER_16bits*2, &bytesRead);
        if (err != FR_OK) {
            f_close(&file);
            return SF_ERROR;
        }

        //bytRead counts the bytes. We need to have the number of uint16 now
        size_buffer_to_read = bytesRead/2;

        //converts the numbers read into the format needed by the DAC
        //=> signed 16bits to unsigned 12bits with volume control
        for(i = 0 ; i < size_buffer_to_read ; i++){
            buffer_to_fill[i] = (uint16_t)(((int16_t)buffer_to_fill[i] + OFFSET_INT16_TO_UINT16) >> 4);
            buffer_to_fill[i] = (buffer_to_fill[i] * sound_file_volume)/100;  //applies the volume
        }

        //wait the reading of a buffer is complete before filling it
        //special case is at the begining. Because we need to start the DAC and we already know
        //we can fill the other buffer
        if(first_time){
            dac_play_buffer(buffer_to_read, size_buffer_to_read, SAMPLING_44100HZ, end_cb);
            first_time = 0;
        }else{
            chBSemWait(&change_buffer);
        }

        //swaps the buffers to fill
        if(buffer_to_fill == sound_buffer1){
            buffer_to_fill = sound_buffer2;
        }else{
            buffer_to_fill = sound_buffer1;
        }

    }while(size_buffer_to_read >= SIZE_SOUND_BUFFER_16bits);


    //closes the file.
    f_close(&file);

    return SF_OK;

}

static THD_WORKING_AREA(waPlaySoundFileThd, 1536);
static THD_FUNCTION(PlaySoundFileThd, arg) {

  chRegSetThreadName("PlaySoundFile Thd");

    (void)arg;

    char *pathToFile = NULL;

    while(1){
        //this thread is waiting until it receives a message
        chSysLock();
        pathToFile = (char*) chThdSuspendS(&play_sound_file_ref);
        chSysUnlock();

        playWAVFile(pathToFile);
        play = false;
        //little delay otherwise there is a playback error sometime resulting in a disgracefull noise
        chThdSleepMilliseconds(10);
        //signals to the threads waiting that the sound is finished
        chCondBroadcast(&play_sound_file_condvar);
    }
}


//////////////////////////////////////////PUBLIC FUNCTIONS////////////////////////////////////

void playSoundFileStart(void){

    //creates the thread
    chThdCreateStatic(waPlaySoundFileThd, sizeof(waPlaySoundFileThd), NORMALPRIO, PlaySoundFileThd, NULL);
}

void playSoundFile(char* pathToFile, playSoundFileOption_t option){

    if(pathToFile == NULL){
        return;
    }

    //stops a eventual melody being played to avoid conflict
    stopCurrentMelody();
    waitMelodyHasFinished();

    //SIMPLE_PLAY case
    if(option == SF_SIMPLE_PLAY){
        //if the reference is NULL, then the thread is already running
        //when the reference becomes not NULL, it means the thread is waiting
        if(play_sound_file_ref != NULL){
            play = true;
            //tells the thread to play the file given
            chThdResume(&play_sound_file_ref, (msg_t) pathToFile);
        }
    }//FORCE_CHANGE or WAIT_AND_CHANGE cases
    else{
        if(option == SF_FORCE_CHANGE){
            stopCurrentSoundFile();
        }
        waitSoundFileHasFinished();
        play = true;
        //tell the thread to play the file given
        chThdResume(&play_sound_file_ref, (msg_t) pathToFile);
    }
}

void stopCurrentSoundFile(void){
    play = false;
}

void waitSoundFileHasFinished(void) {
  //if a melody is playing
  if(play_sound_file_ref == NULL){
    //waits until the current melody is finished
    chMtxLock(&play_sound_file_lock);
    chCondWait(&play_sound_file_condvar);
    chMtxUnlock(&play_sound_file_lock);
  }
}

void setSoundFileVolume(uint8_t volume){
    if(volume > VOLUME_MAX){
        volume = VOLUME_MAX;
    }

    //powers OFF the speaker if we are below the minimum volume
    //because we have a lot of noise on the speaker when the volume is to low
    if(play){
        if(volume < VOLUME_MIN){
            dac_power_speaker(false);
        }else{
            dac_power_speaker(true);
        }
    }
    
    sound_file_volume = volume;
}






