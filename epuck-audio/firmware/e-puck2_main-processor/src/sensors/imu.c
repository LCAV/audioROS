#include <ch.h>
#include <hal.h>
#include <main.h>
#include "usbcfg.h"
#include "chprintf.h"
#include "i2c_bus.h"
#include "imu.h"
#include "exti.h"

static imu_msg_t imu_values;

static uint8_t accAxisFilteringInProgress = 0;
static uint8_t accAxisFilteringState = 0;
static uint8_t accAxisSelected = 0;
static uint8_t accFilterSize = 0;
static uint8_t accCalibrationInProgress = 0;

static uint8_t gyroAxisFilteringInProgress = 0;
static uint8_t gyroAxisFilteringState = 0;
static uint8_t gyroAxisSelected = 0;
static uint8_t gyroFilterSize = 0;
static uint8_t gyroCalibrationInProgress = 0;

static thread_t *imuThd;
static bool imu_configured = false;

/***************************INTERNAL FUNCTIONS************************************/

 /**
 * @brief   Thread which updates the measures and publishes them
 */
static THD_WORKING_AREA(imu_reader_thd_wa, 512);
static THD_FUNCTION(imu_reader_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     event_listener_t imu_int;

     /* Starts waiting for the external interrupts. */
     chEvtRegisterMaskWithFlags(&exti_events, &imu_int,
                                (eventmask_t)EXTI_EVENT_IMU_INT,
                                (eventflags_t)EXTI_EVENT_IMU_INT);

     // Declares the topic on the bus.
     messagebus_topic_t imu_topic;
     MUTEX_DECL(imu_topic_lock);
     CONDVAR_DECL(imu_topic_condvar);
     messagebus_topic_init(&imu_topic, &imu_topic_lock, &imu_topic_condvar, &imu_values, sizeof(imu_values));
     messagebus_advertise_topic(&bus, &imu_topic, "/imu");

     uint8_t accCalibrationNumSamples = 0;
     int32_t accCalibrationSum = 0;
     uint8_t gyroCalibrationNumSamples = 0;
     int32_t gyroCalibrationSum = 0;
     systime_t time;

     while (chThdShouldTerminateX() == false) {
    	 time = chVTGetSystemTime();

      //    /* Waits for a measurement to come. */
      //    chEvtWaitAny(EXTI_EVENT_IMU_INT);
      //    //Clears the flag. Otherwise the event is always true
    	 // chEvtGetAndClearFlags(&imu_int);

    	if(imu_configured == true){
	 		/* Reads the incoming measurement. */
			mpu9250_read(	imu_values.gyro_rate, imu_values.acceleration, &imu_values.temperature, 
							imu_values.magnetometer, imu_values.gyro_raw, imu_values.acc_raw, 
							imu_values.gyro_offset, imu_values.acc_offset, &imu_values.status);
    	}


         /* Publishes it on the bus. */
         messagebus_topic_publish(&imu_topic, &imu_values, sizeof(imu_values));

         if(accAxisFilteringInProgress) {
         	switch(accAxisFilteringState) {
 				case 0:
 					imu_values.acc_offset[accAxisSelected] = 0;
 					accCalibrationSum = 0;
 					accCalibrationNumSamples = 0;
 					accAxisFilteringState = 1;
 					break;

 				case 1:
 					accCalibrationSum += imu_values.acc_raw[accAxisSelected];
 					accCalibrationNumSamples++;
 					if(accCalibrationNumSamples == accFilterSize) {
 						imu_values.acc_filtered[accAxisSelected] = accCalibrationSum/accFilterSize;
 						accAxisFilteringInProgress = 0;
 						if(accCalibrationInProgress == 1) {
 							imu_values.acc_offset[accAxisSelected] = imu_values.acc_filtered[accAxisSelected];
 							accCalibrationInProgress = 0;
 						}
 					}
 					break;
         	}
         }

         if(gyroAxisFilteringInProgress) {
         	switch(gyroAxisFilteringState) {
 				case 0:
 					imu_values.gyro_offset[gyroAxisSelected] = 0;
 					gyroCalibrationSum = 0;
 					gyroCalibrationNumSamples = 0;
 					gyroAxisFilteringState = 1;
 					break;

 				case 1:
 					gyroCalibrationSum += imu_values.gyro_raw[gyroAxisSelected];
 					gyroCalibrationNumSamples++;
 					if(gyroCalibrationNumSamples == gyroFilterSize) {
 						imu_values.gyro_filtered[gyroAxisSelected] = gyroCalibrationSum/gyroFilterSize;
 						gyroAxisFilteringInProgress = 0;
 						if(gyroCalibrationInProgress == 1) {
 							imu_values.gyro_offset[gyroAxisSelected] = imu_values.gyro_filtered[gyroAxisSelected];
 							gyroCalibrationInProgress = 0;
 						}
 					}
 					break;
         	}
         }

         chThdSleepUntilWindowed(time, time + MS2ST(4)); //reduced the sample rate to 250Hz

     }
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void imu_start(void)
{
	int8_t status = MSG_OK;

	if(imu_configured) {
		return;
	}

	i2c_start();

    status = mpu9250_setup(MPU9250_ACC_FULL_RANGE_2G
		                  | MPU9250_GYRO_FULL_RANGE_250DPS
		                  | MPU9250_SAMPLE_RATE_DIV(100));
		                  //| MPU60X0_LOW_PASS_FILTER_6)

    //not tested yet because the auxilliary I2C of the MPU-9250 is condamned due
    //to PCB correction on the e-puck2-F4, so the magnetometer cannot be read...
    // if(status == MSG_OK){
    // 	status = mpu9250_magnetometer_setup();
    // }

    if(status == MSG_OK){
    	imu_configured = true;
    }

    imuThd = chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa), NORMALPRIO, imu_reader_thd, NULL);
}

void imu_stop(void) {
    chThdTerminate(imuThd);
    chThdWait(imuThd);
    imuThd = NULL;
    imu_configured = false;
}

// Gets last axis value read from the sensor.
int16_t get_acc(uint8_t axis) {
	if(axis < 3) {
		return imu_values.acc_raw[axis];
	}
	return 0;
}

void get_acc_all(int16_t *values) {
	values[0] = imu_values.acc_raw[0];
	values[1] = imu_values.acc_raw[1];
	values[2] = imu_values.acc_raw[2];
}

// Returns an average of the last "filter_size" axis values read from the sensor.
int16_t get_acc_filtered(uint8_t axis, uint8_t filter_size) {
	if(axis < 3) {
		if(imu_configured == true){
			accAxisFilteringState = 0;
			accAxisFilteringInProgress = 1;
			accAxisSelected = axis;
			accFilterSize = filter_size;
			while(accAxisFilteringInProgress) {
				chThdSleepMilliseconds(20);
			}
		}
		return imu_values.acc_filtered[axis];
	}
	return 0;
}

// Saves an average of the last 50 samples for each axis, these values are the calibration/offset values.
void calibrate_acc(void) {
	if(imu_configured == true){
		accCalibrationInProgress = 1;
		get_acc_filtered(0, 50);
		accCalibrationInProgress = 1;
		get_acc_filtered(1, 50);
		accCalibrationInProgress = 1;
		get_acc_filtered(2, 50);
		accCalibrationInProgress = 0;
	}
}

// Returns the calibration value of the axis.
int16_t get_acc_offset(uint8_t axis) {
	if(axis < 3) {
		return imu_values.acc_offset[axis];
	}
	return 0;
}

float get_acceleration(uint8_t axis) {
	if(axis < 3) {
		return imu_values.acceleration[axis];
	}
	return 0;
}

int16_t get_gyro(uint8_t axis) {
	if(axis < 3) {
		return imu_values.gyro_raw[axis];
	}
	return 0;
}

void get_gyro_all(int16_t *values) {
	values[0] = imu_values.gyro_raw[0];
	values[1] = imu_values.gyro_raw[1];
	values[2] = imu_values.gyro_raw[2];
}

int16_t get_gyro_filtered(uint8_t axis, uint8_t filter_size) {
	if(axis < 3) {
		if(imu_configured == true){
			gyroAxisFilteringState = 0;
			gyroAxisFilteringInProgress = 1;
			gyroAxisSelected = axis;
			gyroFilterSize = filter_size;
			while(gyroAxisFilteringInProgress) {
				chThdSleepMilliseconds(20);
			}
		}
		return imu_values.gyro_filtered[axis];
	}
	return 0;
}

int16_t get_gyro_offset(uint8_t axis) {
	if(axis < 3) {
		return imu_values.gyro_offset[axis];
	}
	return 0;
}

// Saves an average of the last 50 samples for each axis, these values are the calibration/offset values.
void calibrate_gyro(void) {
	if(imu_configured == true){
		gyroCalibrationInProgress = 1;
		get_gyro_filtered(0, 50);
		gyroCalibrationInProgress = 1;
		get_gyro_filtered(1, 50);
		gyroCalibrationInProgress = 1;
		get_gyro_filtered(2, 50);
		gyroCalibrationInProgress = 0;
	}
	
}

float get_gyro_rate(uint8_t axis) {
	if(axis < 3) {
		return imu_values.gyro_rate[axis];
	}
	return 0;
}

float get_temperature(void) {
	return imu_values.temperature;
}

/**************************END PUBLIC FUNCTIONS***********************************/

