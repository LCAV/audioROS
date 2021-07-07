#ifndef PO8030_H
#define PO8030_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PO8030_MAX_WIDTH 640
#define PO8030_MAX_HEIGHT 480

typedef enum {
    FORMAT_CBYCRY = 0x00,
    FORMAT_CRYCBY = 0x01,
    FORMAT_YCBYCR = 0x02,
    FORMAT_YCRYCB = 0x03,
    FORMAT_RGGB = 0x10,
    FORMAT_GBRG = 0x11,
    FORMAT_GRBG = 0x12,
    FORMAT_BGGR = 0x13,
    FORMAT_RGB565 = 0x30,
    FORMAT_RGB565_BYTE_SWAP = 0x31,
    FORMAT_BGR565 = 0x32,
    FORMAT_BGR565_BYTE_SWAP = 0x33,
    FORMAT_RGB444 = 0x36,
    FORMAT_RGB444_BYTE_SWAP = 0x37,
    FORMAT_DPC_BAYER = 0x41,
    FORMAT_YYYY = 0x44
} format_t;

typedef enum {
    SIZE_VGA = 0x00,
    SIZE_QVGA = 0x01,
    SIZE_QQVGA = 0x02
} image_size_t;

typedef enum {
    SUBSAMPLING_X1 = 0x20,
    SUBSAMPLING_X2 = 0x40,
    SUBSAMPLING_X4 = 0x80
} subsampling_t;

struct po8030_configuration {
	uint16_t 		width;
	uint16_t 		height;
	format_t 		curr_format;
	subsampling_t 	curr_subsampling_x;
	subsampling_t 	curr_subsampling_y;
};

/**
 * @brief       Initializes the clock generation for the po8030
 */
void po8030_start(void);

 /**
 * @brief   Configures some parameters of the camera
 * 
 * @param fmt           format of the image. See format_t
 * @param imgsize       size of the image. See image_size_t
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end or if wrong imgsize
 *
 */
int8_t po8030_config(format_t fmt, image_size_t imgsize);

 /**
 * @brief   Configures advanced setting of the camera
 * 
 * @param fmt           format of the image. See format_t
 * @param x1            x coordinate of the upper left corner of the zone to capture from the sensor
 * @param y1            y coordinate of the upper left corner of the zone to capture from the sensor
 * @param width         width of the image to capture
 * @param height        height of the image to capture
 * subsampling_x        subsampling in the x axis. See subsampling_t
 * subsampling_y        subsampling in the y axis. See subsampling_t
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 *
 */
int8_t po8030_advanced_config(  format_t fmt, unsigned int x1, unsigned int y1, 
                                unsigned int width, unsigned int height, 
                                subsampling_t subsampling_x, subsampling_t subsampling_y);

 /**
 * @brief   Sets the brigthness of the camera
 * 
 * @param value         Brightness. [7] = sign (positive if 0) and [6:0] the value. => from -128 to 127
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 * @retval others        see in the implementation for details
 *
 */
int8_t po8030_set_brightness(uint8_t value);

 /**
 * @brief   Sets the contrast of the camera
 * 
 * @param value         Contrast
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 *
 */
int8_t po8030_set_contrast(uint8_t value);

 /**
 * @brief   Sets mirroring for both vertical and horizontal orientations.
 * 
 * @param vertical      1 to enable vertical mirroring. 0 otherwise
 * @param horizontal    1 to enable horizontal mirroring. 0 otherwise
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 *
 */
int8_t po8030_set_mirror(uint8_t vertical, uint8_t horizontal);

 /**
 * @brief   Enables/disables auto white balance.
 * 
 * @param awb      1 to enable auto white balance. 0 otherwise
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 *
 */
int8_t po8030_set_awb(uint8_t awb);

 /**
 * @brief   Sets the white balance for the red, green and blue gains. 
 *          Writes the values to the camera but has no effect if auto white balance is enabled
 * 
 *          The gains to give are binary values with decimal point between bit6 and bit5.
 *          => bit7 is weighted by 2, bit6 by 1, bit5 by 1/2, bit4 by 1/4, etc.
 *          
 * @param r             red gain. Default is 0x5E.
 * @param g             green gain. Default is 0x40.
 * @param b             blue gain. Default is 0x5D.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 *
 */
int8_t po8030_set_rgb_gain(uint8_t r, uint8_t g, uint8_t b);

 /**
 * @brief   Enables/disables auto exposure.
 * 
 * @param ae            1 to enable auto exposure. 0 otherwise
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 *
 */
int8_t po8030_set_ae(uint8_t ae);

 /**
 * @brief   Sets integration time, aka the exposure.
 *          Total integration time is: (integral + fractional/256) line time. 
 * 
 * @param integral      unit is line time. Default is 0x0080 (128).
 * @param fractional    unit is 1/256 line time. Default is 0x00 (0).
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 *
 */
int8_t po8030_set_exposure(uint16_t integral, uint8_t fractional);

 /**
 * @brief   Returns the current image size in bytes.
 *
 * @return              The image size in bytes
 *
 */
uint32_t po8030_get_image_size(void);

/**
 * @brief   I2C related function. Returns the last I2C error
 *
 *@return              I2C error. See i2cflags_t 
 *
 */
i2cflags_t get_last_i2c_error(void);

#ifdef __cplusplus
}
#endif

#endif
