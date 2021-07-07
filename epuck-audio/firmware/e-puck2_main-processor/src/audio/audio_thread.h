#ifndef AUDIO_THREAD_H
#define AUDIO_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

 /**
 * @brief   Starts the DAC module. Power of the audio amplifier and DAC peripheral
 */
void dac_start(void);

 /**
 * @brief   Plays the specified frequence on the speaker
 */
void dac_play(uint16_t freq);

/**
 * @brief Plays the buffer to play with the DAC module. Needs to already have started it using dac_start()
 * 
 * @param buf 	buffer to send with the DAC
 * @param size 	size of the buffer in elements
 * @param sampling_frequency frequency at which read the samples contained in the buffer
 */
void dac_play_buffer(uint16_t * buf, uint32_t size, uint32_t sampling_frequency, daccallback_t end_cb);

/**
 * @brief 	Changes the buffer to play with the DAC module. The DAC should already be playing something
 * 			using dac_play() or dac_play_buffer() before calling this function
 * 			
 * 			Called from an interrupt context 
 * 
 * @param buf 					buffer to send with the DAC
 * @param size 					size of the buffer in elements
 * @param sampling_frequency 	frequency at which read the samples contained in the buffer
 */
void dac_change_bufferI(uint16_t* buf, uint32_t size, uint32_t sampling_frequency);

/**
 * @brief   Stops the sound being played on the speaker (if any).
 * 			Called from an interrupt context
 */
void dac_stopI(void);

/**
 * @brief   Stops the sound being played on the speaker (if any)
 */
void dac_stop(void);
/**
 * @brief Powers ON or OFF the alimentation of the speaker
 * 
 * @param on_off true to power ON and false to power OFF
 */
void dac_power_speaker(bool on_off);

#ifdef __cplusplus
}
#endif

#endif
