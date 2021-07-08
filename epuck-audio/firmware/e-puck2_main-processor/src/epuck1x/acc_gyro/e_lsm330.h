/*! \file
 * \brief LSM330 library header
 * \author Stefano Morgani
 */

#ifndef __LSM330_H__
#define __LSM330_H__

#ifdef __cplusplus
extern "C" {
#endif

void initAccAndGyro(void);

// do not remove the offsets computed during calibration
void getAllAxesGyroRaw(unsigned char *arr);

void getAllAxesGyro(signed int *x, signed int *y, signed int *z);

// degrees (two's complement)
signed char getTemperature(void);

int getXAxisGyro(void);

int getYAxisGyro(void);

int getZAxisGyro(void);

void calibrateGyroscope(int numSamples);

float rawToDps(int value);

float rawToDpms(int value);

#ifdef __cplusplus
}
#endif

#endif
