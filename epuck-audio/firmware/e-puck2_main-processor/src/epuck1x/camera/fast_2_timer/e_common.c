#include "e_poxxxx.h"
#include "../camera/dcmi_camera.h"
#include "../camera/po8030.h"
#include <stdlib.h>

int e_poxxxx_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
				unsigned int sensor_width,unsigned int sensor_height,
				unsigned int zoom_fact_width,unsigned int zoom_fact_height,  
				int color_mode) {

	format_t fmt;
	subsampling_t subx, suby;

	sensor_width = sensor_width/zoom_fact_width;
	sensor_height = sensor_height/zoom_fact_height;

	if(color_mode == GREY_SCALE_MODE) {
		fmt = FORMAT_YYYY;
	} else if(color_mode == RGB_565_MODE) {
		fmt = FORMAT_RGB565;
	} else {
		fmt = FORMAT_YCBYCR;
	}

	switch(zoom_fact_width) {
		case 1:
			subx = SUBSAMPLING_X1;
			break;
		case 2:
			subx = SUBSAMPLING_X2;
			break;
		case 4:
			subx = SUBSAMPLING_X4;
			break;
		default:
			subx = SUBSAMPLING_X4;
			zoom_fact_width = 4;
			break;
	}

	switch(zoom_fact_height) {
		case 1:
			suby = SUBSAMPLING_X1;
			break;
		case 2:
			suby = SUBSAMPLING_X2;
			break;
		case 4:
			suby = SUBSAMPLING_X4;
			break;
		default:
			suby = SUBSAMPLING_X4;
			zoom_fact_height = 4;
			break;
	}

	po8030_advanced_config(fmt, sensor_x1, sensor_y1, sensor_width*zoom_fact_width, sensor_height*zoom_fact_height, subx, suby);
	dcmi_disable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	return 0;
}


void e_poxxxx_set_mirror(int vertical, int horizontal) {
	(void)vertical;
	(void)horizontal;
	return;
}

void e_poxxxx_write_cam_registers(void) {
	return;
}

/**
 * Initalize the camera, return the version in hexa, 0x3030 or 0x6030
 */
int e_poxxxx_init_cam(void) {
	// The camera is already initialized.
	return 0x8030; // Only one camera mounted on e-puck2 so return its id.
}

/**
 * Return the camera orientation
 * \return 0: vertical 480x640, 1: horizontal 640x480, horizontal inverted, -1: unknown
 */
int e_poxxxx_get_orientation(void) {
	return 1;
}

/*! Enable/Disable AWB and AE 
 * \param awb 1 means AWB enabled, 0 means disabled
 * \param ae 1 means AE enabled, 0 means disabled
 */
void e_poxxxx_set_awb_ae(int awb, int ae) {
	(void)awb;
	(void)ae;
	return;
}

/*! Set the gains of the camera
 * \param red The red pixels' gain (0..255)
 * \param green The green pixels' gain (0..255)
 * \param blue The blue pixels' gain (0..255)
 * \warning Only meaningful if AWB is disabled 
 */
void e_poxxxx_set_rgb_gain(unsigned char r, unsigned char g, unsigned char b) {
	(void)r;
	(void)g;
	(void)b;
	return;
}

/*! Set exposure time 
 * \param t Exposure time, LSB is in 1/64 line time
 * \warning Only meaningful if AE is disabled 
 */
void e_poxxxx_set_exposure(unsigned long exp) {
	(void)exp;
	return;
}
unsigned int getCameraVersion(void) {
	return 0x8030;
}

/*! Launch a capture in the \a buf buffer
 * \param buf The buffer to write to
 * \sa e_poxxxx_config_cam and e_poxxxx_is_img_ready
 */
void e_poxxxx_launch_capture(char * buf) {
	(void)buf;
	dcmi_capture_start();
}

/*! Check if the current capture is finished
 * \return Zero if the current capture is in progress, non-zero if the capture is done.
 * \sa e_poxxxx_launch_capture
 */
int e_poxxxx_is_img_ready(void) {
	return image_is_ready();
}

/*! Waits until the current capture is finished
 * \sa e_poxxxx_launch_capture
 */
void e_poxxxx_wait_img_ready(void) {
	wait_image_ready();
}
