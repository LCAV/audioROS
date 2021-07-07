#ifndef DCMI_CAMERA_H
#define DCMI_CAMERA_H

#include <stdint.h>
#include <hal.h>
#include "dcmi.h"
#include "po8030.h"

#define MAX_BUFF_SIZE 19200 //76800 // This means 2 color QQVGA images: (160x120x2)x2; or a single greyscale QVGA image: 320x240.

typedef enum {
	CAPTURE_ONE_SHOT = 0,
	CAPTURE_CONTINUOUS = 1
} capture_mode_t;

/**
 * @brief 		DCMI Driver initialization and image memory allocation.
 *
 * @return		The operation status.
 * @retval 0	if the function succeeded.
 * @retval -1	if memory cannot be allocated.
 */
int8_t dcmi_start(void);

/**
 * @brief   Configures the DCMI peripheral.
 * @details This function configures the DCMI peripheral but keeps it disabled; the DMA is configured and enabled.
 * @post    Upon either of the two buffers being filled, the configured callback
 *          (transfer_complete_cb) is invoked.
 *          At the end of each frame the configured callback
 *          (frame_end_cb) is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes equal to
 *          8 bits else it is organized as uint16_t arrays.
 *
 * @return		The operation status.
 * @retval 0	if the function succeeded.
 * @retval -1	if the image size cannot fit in memory.
 */
int8_t dcmi_prepare(void);

/**
 * @brief Deactivates the DCMI peripheral.
 * @details This function disables the DCMI and related interrupts; also the DMA is released.
 *
 */
void dcmi_unprepare(void);

/**
 * @brief 		Returns if an image is ready
 *
 * @return		Image ready
 * @retval 1	an image is ready
 * @retval 0	no image ready
 *
 */
uint8_t image_is_ready(void);

/**
 * @brief 		Put the thread invocking this function in sleep 
 * 				until an image is ready
 *
 */
void wait_image_ready(void);

/**
 * @brief 		Returns if double buffering is enabled.
 *
 *@return		double buffering state
 *@retval 1		double buffering enabled
 *@retval 0		double buffering disabled
 *
 */
uint8_t dcmi_double_buffering_enabled(void);

/**
* @brief   Enable double buffering and allocate memory for both buffers.
* @details This function splits the available memory in two and allocates MAX_BUFF_SIZE/2 bytes for each buffer; it need to be called before "dcmi_prepare".
*
* @return		The operation status.
* @retval 0		if the function succeeded.
* @retval -1	if memory cannot be allocated.
*
*/
int8_t dcmi_enable_double_buffering(void);

/**
* @brief   Disable double buffering.
* @details This function free the memory allocated for the second image buffer and allocates all the available memory (MAX_BUFF_SIZE bytes) for the first buffer; it need to be called after "dcmi_unprepare".
*
* @return		The operation status.
* @retval 0		if the function succeeded.
* @retval -1	if memory cannot be allocated.
*/
int8_t dcmi_disable_double_buffering(void);

/**
* @brief   Configures the capture mode (oneshot or continuous).
*
* @param mode	capture mode. See capture_mode_t
*
*/
void dcmi_set_capture_mode(capture_mode_t mode);

/**
* @brief   Get the pointer to the last filled image buffer.
*
* @return	the buffer pointer.
*
*/
uint8_t* dcmi_get_last_image_ptr(void);

/**
* @brief   Get the pointer to the first image buffer.
*
* @return	the buffer pointer.
*
*/
uint8_t* dcmi_get_first_buffer_ptr(void);

/**
* @brief   Get the pointer to the second image buffer.
*
* @return	the buffer pointer.
*
*/
uint8_t* dcmi_get_second_buffer_ptr(void);

/**
* @brief   Start capturing the images from the camera.
*
*/
void dcmi_capture_start(void);

/**
* @brief   Stop capturing the images from the camera. It is only needed in "continuous mode".
*
* @return              The operation status.
* @retval MSG_OK       if the function succeeded.
* @retval MSG_TIMEOUT  if a timeout occurred before operation end.
*
*/
msg_t dcmi_capture_stop(void);


#endif /* DCMI_CAMERA_H */
