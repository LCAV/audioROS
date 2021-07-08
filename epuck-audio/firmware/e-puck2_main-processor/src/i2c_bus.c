#include <hal.h>
#include <ch.h>
#include "i2c_bus.h"

static i2cflags_t errors = 0;
static systime_t timeout = MS2ST(4); // 4 ms

void i2c_start(void) {

	if(I2CD1.state != I2C_STOP) {
		return;
	}

    /*
     * I2C configuration structure for camera, IMU and distance sensor.
     * Set it to 400kHz fast mode
     */
    static const I2CConfig i2c_cfg1 = {
        .op_mode = OPMODE_I2C,
        .clock_speed = 200000,
        .duty_cycle = FAST_DUTY_CYCLE_2
    };

    //simulate 16 clock pulses to unblock potential I2C periph blocked
    //take control of the pin
    palSetPadMode(GPIOB, GPIOB_SCL , PAL_MODE_OUTPUT_OPENDRAIN );
    //16 clock pulses
    for(uint8_t i = 0 ; i < 32 ; i++){
    	palTogglePad(GPIOB, GPIOB_SCL);
    	chThdSleepMilliseconds(1);
    }
    //make sure the output is high
    palSetPad(GPIOB, GPIOB_SCL);
    //give the control of the pin to the I2C machine
    palSetPadMode(GPIOB, GPIOB_SCL , PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    i2cStart(&I2CD1, &i2c_cfg1);
}

void i2c_stop(void) {
	i2cStop(&I2CD1);
}

i2cflags_t get_last_i2c_error(void) {
    return errors;
}

int8_t read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
	
	uint8_t txbuf[1] = {reg};
	uint8_t rxbuf[1] = {0};

	i2cAcquireBus(&I2CD1);
	if(I2CD1.state != I2C_STOP) {
		msg_t status = i2cMasterTransmitTimeout(&I2CD1, addr, txbuf, 1, rxbuf, 1, timeout);
		if (status != MSG_OK){
			errors = i2cGetErrors(&I2CD1);
			if(I2CD1.state == I2C_LOCKED){
				i2c_stop();
				i2c_start();
			}
			i2cReleaseBus(&I2CD1);
			return status;
		}
	}
	i2cReleaseBus(&I2CD1);

	*value = rxbuf[0];

    return MSG_OK;
}


int8_t write_reg(uint8_t addr, uint8_t reg, uint8_t value) {

	uint8_t txbuf[2] = {reg, value};
	uint8_t rxbuf[1] = {0};

	i2cAcquireBus(&I2CD1);
	if(I2CD1.state != I2C_STOP) {
		msg_t status = i2cMasterTransmitTimeout(&I2CD1, addr, txbuf, 2, rxbuf, 0, timeout);
		if (status != MSG_OK){
			errors = i2cGetErrors(&I2CD1);
			if(I2CD1.state == I2C_LOCKED){
				i2c_stop();
				i2c_start();
			}
			i2cReleaseBus(&I2CD1);
			return status;
		}
	}
	i2cReleaseBus(&I2CD1);

    return MSG_OK;
}

int8_t read_reg_multi(uint8_t addr, uint8_t reg, uint8_t *buf, int8_t len) {

	i2cAcquireBus(&I2CD1);
	if(I2CD1.state != I2C_STOP) {
		msg_t status = i2cMasterTransmitTimeout(&I2CD1, addr, &reg, 1, buf, len, timeout);
		if (status != MSG_OK){
			errors = i2cGetErrors(&I2CD1);
			if(I2CD1.state == I2C_LOCKED){
				i2c_stop();
				i2c_start();
			}
			i2cReleaseBus(&I2CD1);
			return status;
		}
	}
	i2cReleaseBus(&I2CD1);

	return MSG_OK;
}
