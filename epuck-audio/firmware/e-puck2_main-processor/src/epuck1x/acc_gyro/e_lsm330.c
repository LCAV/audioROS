
/*! \file
 * \brief Manage LSM330 (accelerometer + gyro) registers
 * \author Stefano Morgani
 *
 * x, y axes representation on e-puck rev < 1.3 (analog accelerometer MMA7260) => default representation:
 *      FORWARD
 *          ^
 *          | (-y)
 *          * - > (+x) RIGHT
 *
 * x, y axes representation on e-puck rev 1.3 (i2c device LSM330):
 *      FORWARD
 *          ^
 *          | (-x)
 *          * - > (+y) RIGHT
 *
 * x, y axes representation on e-puck 2 (i2c device LSM330):
 *      FORWARD
 *          ^
 *          | (-y)
 *          * - > (-x) RIGHT
 * Thus the x and y axes are exchanged to maintain compatibility with first e-puck version.
 */

#include "e_lsm330.h"
#include "../sensors/imu.h"

#define RAW_TO_DPS (1.0/131.0) // From the datasheet of the MPU-9250

/*!\brief Configure and turn on the device.
 */
void initAccAndGyro(void) {
	return;
}

void getAllAxesGyroRaw(unsigned char *arr) {
	arr[0] = -get_gyro(1); // Exchange x and y and change sign to be compatible with e-puck1.3.
	arr[1] = get_gyro(0); // Exchange x and y to be compatible with e-puck1.3.
	arr[2] = get_gyro(2);
}

void getAllAxesGyro(signed int *x, signed int *y, signed int *z) {
    int16_t arr[3];
    get_gyro_all(arr);
    *x = -(arr[1] - get_gyro_offset(1)); // Exchange x and y and change sign to be compatible with e-puck1.3.
    *y = arr[0] - get_gyro_offset(0); // Exchange x and y to be compatible with e-puck1.3.
    *z = arr[2] - get_gyro_offset(2);
}

int getXAxisGyro(void) {
	return -(get_gyro(1) - get_gyro_offset(1)); // Exchange x and y and change sign to be compatible with e-puck1.3.
}

int getYAxisGyro(void) {
	return (get_gyro(0) - get_gyro_offset(0)); // Exchange x and y to be compatible with e-puck1.3.
}

int getZAxisGyro(void) {
	return (get_gyro(2) - get_gyro_offset(2));
}

signed char getTemperature(void) {
	return (signed char)get_temperature();
}

void calibrateGyroscope(int numSamples) {
	(void)numSamples;
	calibrate_gyro();
}

/*! \brief Raw value to degrees per second.
 * Take as input the 2's complement value received by the device for a certain axis
 * and return the degrees per second (knowing the gyroscope is configured with sensitivity
 * of +- 250 dps).
 */
float rawToDps(int value) {
    // The gyroscope returns 16 bits 2's complement values with a full-scale of +-250 dps;
    // from the datasheet with full scale = +- 250 dps, then angular rate sensitivity is (1/131) dps/digit
    return (float)value*RAW_TO_DPS;
}

/*! \brief Raw value to degrees per millisecond.
 * Take as input the 2's complement value received by the device for a certain axis
 * and return the degrees per millisecond (knowing the gyroscope is configured with sensitivity
 * of +- 250 dps).
 */
float rawToDpms(int value) {
    return (float)value*RAW_TO_DPS/1000.0;
}
