#include "math.h"
#include "e_acc.h"
#include "sensors/imu.h"
#include "epuck1x/motor_led/advance_one_timer/e_led.h"
#include <stdlib.h>

/*****************************************************
 * internal variables                                *
 *****************************************************/
static int angle_mem = 0;			//used in the display_angle function


/*****************************************************
 * user called function                               *
 *****************************************************/

int e_get_acc(unsigned int captor) {
	// Values adapted to be compatible with e-puck1.x:
    // - shift by 4 the value received from the sensor (16 bits) to get 12 bits value as with e-puck1.x
    // - decrease the resulting value by 1/4 to get about the same value for 1g (in e-puck1.x 1g = about 800)
	int32_t tempValue = 0;
	if(captor == 0) { // X axis => same orientation as with e-puck1.x.
		tempValue = get_acc(captor) - get_acc_offset(captor);
	} else if(captor == 1) { // Y axis => change sign to be compatible with e-puck1.x.
		tempValue = -(get_acc(captor) - get_acc_offset(captor));
	} else { // Z axis => same orientation as with e-puck1.x.
		tempValue = get_acc(captor) - get_acc_offset(captor) + GRAVITY; // Do not consider the gravity as an offset.
	}
	tempValue = (tempValue>>4) - (tempValue>>6);
	return (int)(-tempValue);	// Change the sign to have +1g when the robot is still on the plane and the axis points upwards and is perpendicular to the surface.
								// This is to be compatible with e-puck1.x.
}

int e_get_acc_filtered(unsigned int captor, unsigned int filter_size)
{
	// Values adapted to be compatible with e-puck1.x:
    // - shift by 4 the value received from the sensor (16 bits) to get 12 bits value as with e-puck1.x
    // - decrease the resulting value by 1/4 to get about the same value for 1g (in e-puck1.x 1g = about 800)
	// - add an offset of 2048 because it represents the value of 0g with e-puck1.x (only positive values)
	int32_t tempValue = 0;
	if(captor == 0) { // X axis => same orientation as with e-puck1.x.
		tempValue = -get_acc_filtered(captor, filter_size); // Change the sign to have +1g when the robot is still on the plane and the axis points upwards and is perpendicular to the surface.
	} else if(captor == 1) { // Y axis => change sign to be compatible with e-puck1.x.
		tempValue = get_acc_filtered(captor, filter_size); // Change the sign to have +1g when the robot is still on the plane and the axis points upwards and is perpendicular to the surface.
	} else { // Z axis => same orientation as with e-puck1.x.
		tempValue = -get_acc_filtered(captor, filter_size); // Change the sign to have +1g when the robot is still on the plane and the axis points upwards and is perpendicular to the surface.
	}
	tempValue = ((tempValue>>4) - (tempValue>>6)) + 2048;
	return (int)tempValue;
}

void e_acc_calibr(void)
{
	calibrate_acc();
}

TypeAccSpheric e_read_acc_spheric(void)
{
	TypeAccSpheric result;
	int32_t acc_x, acc_y, acc_z;
	acc_x = e_get_acc(0);
	acc_y = e_get_acc(1);
	acc_z = e_get_acc(2);

	// Calculate the absolute acceleration value.
	result.acceleration = sqrtf((float)((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)));
	result.inclination =  90.0 - atan2f((float)(acc_z), sqrtf( (float)((acc_x * acc_x) + (acc_y * acc_y) ))) * CST_RADIAN;
	if (result.inclination<5 || result.inclination>160) {
		result.orientation=0;
	} else {
		result.orientation = (atan2f((float)(acc_x), (float)(acc_y)) * CST_RADIAN) + 180.0;		// 180 is added to have 0 to 360 degrees range
	}
	return result;
}

float e_read_inclination(void)
{
	TypeAccSpheric result;
	int32_t acc_x, acc_y, acc_z;
	acc_x = e_get_acc(0);
	acc_y = e_get_acc(1);
	acc_z = e_get_acc(2);

	result.inclination =  90.0 - atan2f((float)(acc_z), sqrtf( (float)((acc_x * acc_x) + (acc_y * acc_y) ))) * CST_RADIAN;
	return result.inclination;
}

float e_read_orientation(void)
{
	TypeAccSpheric result;
	int32_t acc_x, acc_y, acc_z;
	acc_x = e_get_acc(0);
	acc_y = e_get_acc(1);
	acc_z = e_get_acc(2);

	result.inclination =  90.0 - atan2f((float)(acc_z), sqrtf( (float)((acc_x * acc_x) + (acc_y * acc_y) ))) * CST_RADIAN;
	if (result.inclination<5 || result.inclination>160) {
		result.orientation=0;
	} else {
		result.orientation = (atan2f((float)(acc_x), (float)(acc_y)) * CST_RADIAN) + 180.0;		// 180 is added to have 0 to 360 degrees range
	}
	return result.orientation;
}

float e_read_acc(void)
{ 
	TypeAccSpheric result;
	int32_t acc_x, acc_y, acc_z;
	acc_x = e_get_acc(0);
	acc_y = e_get_acc(1);
	acc_z = e_get_acc(2);

	result.acceleration = sqrtf((float)((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)));
	return result.acceleration;
}

TypeAccRaw e_read_acc_xyz(void)
{
	TypeAccRaw result;
	result.acc_x = e_get_acc(0);
	result.acc_y = e_get_acc(1);
	result.acc_z = e_get_acc(2);
	return result;
}

int e_read_acc_x(void)
{
	return (e_get_acc(0));
}

int e_read_acc_y(void)
{
	return (e_get_acc(1));
}

int e_read_acc_z(void)
{
	return (e_get_acc(2));
}

void e_display_angle(void)
{
	float angle = 0.0;

// 	To avoid oscillation the limite of variation is limited at
//  a fix value	wich is 1/9 of the LED resolution
	angle = e_read_orientation();
	if ( abs(angle_mem - angle) > 5.0)
	{
		e_led_clear();
			// table of selection
		if ((angle > (360.0 - 22.5)) |  (angle <= (0.0   + 22.5)) && angle != ANGLE_ERROR)
			e_set_led(0, 1);
		else if ( angle > (45.0 - 22.5)  && angle <= (45.0  + 22.5))
			e_set_led(7, 1);
		else if ( angle > (90.0 - 22.5)  && angle <= (90.0  + 22.5))
			e_set_led(6, 1);
		else if ( angle > (135.0 - 22.5) && angle <= (135.0 + 22.5))
			e_set_led(5, 1);
		else if ( angle > (180.0 - 22.5) && angle <= (180.0 + 22.5))
			e_set_led(4, 1);
		else if ( angle > (225.0 - 22.5) && angle <= (225.0 + 22.5))
			e_set_led(3, 1);
		else if ( angle > (270.0 - 22.5) && angle <= (270.0 + 22.5))
			e_set_led(2, 1);
		else if ( angle > (315.0 - 22.5) && angle <= (315.0 + 22.5))
			e_set_led(1, 1);
		angle_mem = angle;	// value to compare with the next one
	}
}

