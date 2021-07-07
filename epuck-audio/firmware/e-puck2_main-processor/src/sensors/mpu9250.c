#include <stdint.h>
#include <math.h>
#include "mpu9250.h"
#include "../i2c_bus.h"

#define STANDARD_GRAVITY    9.80665f 
#define DEG2RAD(deg) (deg / 180 * M_PI)

#define RES_2G      2.0f
#define RES_250DPS  250.0f
#define RES_4800UT  4800.0f
#define MAX_INT16   32768.0f

#define RAW16BITS_TO_TESLA  (RES_4800UT/MAX_INT16)  //4800uT scale for int16 raw
#define ACC_RAW2G           (RES_2G / MAX_INT16)   //2G scale for int16 raw value
#define GYRO_RAW2DPS        (RES_250DPS / MAX_INT16)   //250DPS (degrees per second) scale for int16 raw value

static uint8_t imu_addr = MPU9250_ADDRESS_AD1_0;

/***************************INTERNAL FUNCTIONS************************************/

void mpu9250_change_addr(void) {
	if(imu_addr == MPU9250_ADDRESS_AD1_0) {
		imu_addr = MPU9250_ADDRESS_AD1_1;
	} else {
		imu_addr = MPU9250_ADDRESS_AD1_0;
	}
}

 /**
 * @brief   reads the id of the sensor
 * 
 * @param id            pointer to store the id of the sensor
 * 
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 */
int8_t mpu9250_read_id(uint8_t *id) {
    int8_t err = 0;
    if((err = read_reg(imu_addr, WHO_AM_I_MPU9250, id)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = read_reg(imu_addr, WHO_AM_I_MPU9250, id)) != MSG_OK) {
    		return err;
    	}
    }
    return MSG_OK;
}

 /**
 * @brief   reads a 16bit word from an 8bit buffer
 * 
 * @param buf           buffer to read
 * 
 * @return              The 16bit word read
 */
static int32_t read_word(const uint8_t *buf) // signed int16
{
    return (int16_t)((int8_t)buf[0]) << 8 | buf[1];
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

int8_t mpu9250_setup(int config) {
	int8_t err = 0;
	uint8_t regValue = 0;

    // Reset device.
    if((err = write_reg(imu_addr, PWR_MGMT_1, 0x80)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, PWR_MGMT_1, 0x80)) != MSG_OK) {
    		return err;
    	}
    }	
    chThdSleepMilliseconds(1);
    while(1) {
		if((err = read_reg(imu_addr, PWR_MGMT_1, &regValue)) != MSG_OK) {
			mpu9250_change_addr();
			if((err = read_reg(imu_addr, PWR_MGMT_1, &regValue)) != MSG_OK) {
				return err;
			}
		}
		if(!(regValue & 0x80)) {
			break;
		}
        chThdSleepMilliseconds(1);
    }
	
    // Gyro full scale.
    if((err = write_reg(imu_addr, GYRO_CONFIG, (config << 1) & 0x18)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, GYRO_CONFIG, (config << 1) & 0x18)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

    // Accelerometer full scale.
    if((err = write_reg(imu_addr, ACCEL_CONFIG, (config << 3) & 0x18)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, ACCEL_CONFIG, (config << 3) & 0x18)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

    // Sample rate divisor.
    if((err = write_reg(imu_addr, SMPLRT_DIV, (config >> 8) & 0xff)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, SMPLRT_DIV, (config >> 8) & 0xff)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

    // Enable interrupts: data ready.
    if((err = write_reg(imu_addr, INT_ENABLE, INTERRUPT_DATA_RDY)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, INT_ENABLE, INTERRUPT_DATA_RDY)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

//    // low pass filter config, FSYNC disabled
//    mpu60X0_reg_write(dev, MPU60X0_RA_CONFIG, (config >> 16) & 0x07);
//    chThdSleepMilliseconds(1);

    return err;
	
}

int8_t mpu9250_magnetometer_setup(void){
    int8_t err = 0;

     //enable bypass mode for I2C peripherals connected to the MPU9250.
     //(the magnetometer is connected to the auxilliary I2C)
     if((err = write_reg(imu_addr, INT_PIN_CFG, 0x02)) != MSG_OK) {
    	 mpu9250_change_addr();
    	 if((err = write_reg(imu_addr, INT_PIN_CFG, 0x02)) != MSG_OK) {
    		 return err;
    	 }
     }

     //set to continuous mode 1(8Hz) and 16bits resolution
     if((err = write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x12)) != MSG_OK) {
         return err;
     }

     //disable bypass mode for I2C peripherals connected to the MPU9250.
     //(the magnetometer is connected to the auxilliary I2C)
     if((err = write_reg(imu_addr, INT_PIN_CFG, 0x00)) != MSG_OK) {
    	 mpu9250_change_addr();
    	 if((err = write_reg(imu_addr, INT_PIN_CFG, 0x00)) != MSG_OK) {
    		 return err;
    	 }
     }

    //configure I2C_slave0 to read the magnetometer registers

    //configure the I2C Master
    // No other masters on the bus, unless there are (in which case switch this high bit)
    // Wait for external sensors to finish before data ready interrupt
    // No FIFO for Slave3 (which is actually about Slave3 and not the I2C Master)
    // Always issue a full stop, then a start when transitioning between slaves (instead of a restart)
    // Access the bus at 400kHz (see table in register map for other values)
    if((err = write_reg(imu_addr, I2C_MST_CTRL, 0x5D)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, I2C_MST_CTRL, 0x5D)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

    //enable the I2C Master of the MPU
    if((err = write_reg(imu_addr, USER_CTRL, 0x20)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, USER_CTRL, 0x20)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

    //configure the I2C slave adress in read mode
    if((err = write_reg(imu_addr, I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

    //configure the first register to read from the slave
    if((err = write_reg(imu_addr, I2C_SLV0_REG, AK8963_XOUT_L)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, I2C_SLV0_REG, AK8963_XOUT_L)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

    // Enable slave 0 interface
    // Do not swap bytes
    // Send the reg adress to read or write (normal I2C behavior)
    // Don't use even swap alignement
    // Read 7 bytes
    if((err = write_reg(imu_addr, I2C_SLV0_CTRL, 0x87)) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = write_reg(imu_addr, I2C_SLV0_CTRL, 0x87)) != MSG_OK) {
    		return err;
    	}
    }
    chThdSleepMilliseconds(1);

    return err;
}

bool mpu9250_ping(void) {
	uint8_t id = 0;
	mpu9250_read_id(&id);
	return id == 0x68;
}

int8_t mpu9250_read(float *gyro, float *acc, float *temp, float *magnet, int16_t *gyro_raw, int16_t *acc_raw, int16_t *gyro_offset, int16_t *acc_offset, uint8_t *status) {
	int8_t err = 0;

    static uint8_t buf[1 + 6 + 2 + 6 + 6 + 1]; // interrupt status, accel, temp, gyro, magnetometer, status magnetometer
    if((err = read_reg_multi(imu_addr, INT_STATUS, buf, sizeof(buf))) != MSG_OK) {
    	mpu9250_change_addr();
    	if((err = read_reg_multi(imu_addr, INT_STATUS, buf, sizeof(buf))) != MSG_OK) {
    		return err;
    	}
    }

    if(status) {
    	*status = buf[0];
    }
    if (acc) {
    	// Change the sign of all axes to have -1g when the robot is still on the plane and the axis points upwards and is perpendicular to the surface.
    	acc_raw[X_AXIS] = -read_word(&buf[1]);
    	acc_raw[Y_AXIS] = -read_word(&buf[3]);
    	acc_raw[Z_AXIS] = -read_word(&buf[5]);
        acc[X_AXIS] = (acc_raw[X_AXIS] - acc_offset[X_AXIS]) * STANDARD_GRAVITY * ACC_RAW2G;
        acc[Y_AXIS] = (acc_raw[Y_AXIS] - acc_offset[Y_AXIS]) * STANDARD_GRAVITY * ACC_RAW2G;
        //specific case for the z axis because it should not be zero but -1g
        //deletes the standard gravity to have only the offset
        acc[Z_AXIS] = (acc_raw[Z_AXIS] - acc_offset[Z_AXIS] - (MAX_INT16 / RES_2G)) * STANDARD_GRAVITY * ACC_RAW2G;
    }
    if (temp) {
        *temp = (float)((read_word(&buf[7]) - 21.0f) / 333.87f) + 21.0f; // Degrees.
    }
    if (gyro) {
    	gyro_raw[X_AXIS] = read_word(&buf[9]);
    	gyro_raw[Y_AXIS] = read_word(&buf[11]);
    	gyro_raw[Z_AXIS] = read_word(&buf[13]);
        gyro[X_AXIS] = (gyro_raw[X_AXIS] - gyro_offset[X_AXIS]) * DEG2RAD(GYRO_RAW2DPS);
        gyro[Y_AXIS] = (gyro_raw[Y_AXIS] - gyro_offset[Y_AXIS]) * DEG2RAD(GYRO_RAW2DPS);
        gyro[Z_AXIS] = (gyro_raw[Z_AXIS] - gyro_offset[Z_AXIS]) * DEG2RAD(GYRO_RAW2DPS);
    }

    if(magnet){
        magnet[X_AXIS] = ((int16_t)((int8_t)buf[16]) << 8 | buf[15]) * RAW16BITS_TO_TESLA;
        magnet[Y_AXIS] = ((int16_t)((int8_t)buf[18]) << 8 | buf[17]) * RAW16BITS_TO_TESLA;
        magnet[Z_AXIS] = ((int16_t)((int8_t)buf[20]) << 8 | buf[19]) * RAW16BITS_TO_TESLA;
    }

    return MSG_OK;
}

/**************************END PUBLIC FUNCTIONS***********************************/

