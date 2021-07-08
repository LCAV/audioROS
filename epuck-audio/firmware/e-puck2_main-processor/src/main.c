#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/po8030.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root, aseba_ns;

static THD_WORKING_AREA(selector_thd_wa, 2048);

static bool load_config(void)
{
    extern uint32_t _config_start;

    return config_load(&parameter_root, &_config_start);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static THD_FUNCTION(selector_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    uint8_t stop_loop = 0;
    systime_t time;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;
    int16_t leftSpeed = 0, rightSpeed = 0;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    uint8_t toEsp32 = 'c', fromEsp32 = 0;
    int16_t len = 0;

    uint8_t hw_test_state = 0;
    uint8_t *img_buff_ptr;
    uint16_t r = 0, g = 0, b = 0;
    uint8_t rgb_state = 0, rgb_counter = 0;
    uint16_t melody_state = 0, melody_counter = 0;

    uint8_t magneto_state = 0;

	uint8_t rab_addr = 0x20;
	uint8_t rab_state = 0;
	int8_t i2c_err = 0;
	uint8_t regValue[2] = {0};
	uint16_t rab_data = 0;
	double rab_bearing = 0.0;
	uint16_t rab_range = 0;
	uint16_t rab_sensor = 0;

	calibrate_acc();
	calibrate_gyro();

    while(stop_loop == 0) {
    	time = chVTGetSystemTime();

		switch(get_selector()) {
			case 0: // Aseba.
				aseba_vm_start();
				stop_loop = 1;
				break;

			case 1: // Shell.
				shell_start();
				stop_loop = 1;
				break;

			case 2: // Read proximity sensors.
				messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

				if (SDU1.config->usbp->state != USB_ACTIVE) { // Skip printing if port not opened.
					continue;
				}

				// Sensors info print: each line contains data related to a single sensor.
		        for (uint8_t i = 0; i < sizeof(prox_values.ambient)/sizeof(prox_values.ambient[0]); i++) {
		        //for (uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
		        	chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.ambient[i]);
		        	chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.reflected[i]);
		        	chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[i]);
		        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
		        }
		        chprintf((BaseSequentialStream *)&SDU1, "\r\n");

				chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
				break;

			case 3: // Asercom protocol v2 (BT).
				run_asercom2();
				stop_loop = 1;
				break;

			case 4: // Range and bearing - receiver.
				switch(rab_state) {
					case 0:
						write_reg(rab_addr, 12, 150);	// Set range.
						write_reg(rab_addr, 17, 0);		// Onboard calculation.
						write_reg(rab_addr, 16, 0);		// Store light conditions.
						rab_state = 1;
						break;

					case 1:
					    if((i2c_err = read_reg(rab_addr, 0, &regValue[0])) != MSG_OK) {
					    	chprintf((BaseSequentialStream *)&SDU1, "err\r\n");
					        break;
					    }
					    if(regValue[0] != 0) {
					    	read_reg(rab_addr, 1, &regValue[0]);
							read_reg(rab_addr, 2, &regValue[1]);
							rab_data = (((uint16_t)regValue[0])<<8) + regValue[1];

					    	read_reg(rab_addr, 3, &regValue[0]);
							read_reg(rab_addr, 4, &regValue[1]);
							rab_bearing = ((double)((((uint16_t)regValue[0])<<8) + regValue[1])) * 0.0001;

					    	read_reg(rab_addr, 5, &regValue[0]);
							read_reg(rab_addr, 6, &regValue[1]);
							rab_range = (((uint16_t)regValue[0])<<8) + regValue[1];

							read_reg(rab_addr, 9, &regValue[0]);
							rab_sensor = regValue[0];

		    	if (SDU1.config->usbp->state != USB_ACTIVE) { // Skip printing if port not opened.
								break;
							}

							chprintf((BaseSequentialStream *)&SDU1, "%d %3.2f %d %d\r\n", rab_data, (rab_bearing*180.0/M_PI), rab_range, rab_sensor);
		    	}
						break;
				}
				chThdSleepUntilWindowed(time, time + MS2ST(20)); // Refresh @ 50 Hz.
				break;

			case 5: // Range and bearing - transmitter.
				switch(rab_state) {
					case 0:
						write_reg(rab_addr, 12, 150); // Set range.
						if((i2c_err = read_reg(rab_addr, 12, &regValue[0])) == MSG_OK) {
							chprintf((BaseSequentialStream *)&SDU1, "set range to %d\r\n", regValue[0]);
						}
						write_reg(rab_addr, 17, 0); // Onboard calculation.
						if((i2c_err = read_reg(rab_addr, 12, &regValue[0])) == MSG_OK) {
							chprintf((BaseSequentialStream *)&SDU1, "onboard calculation enabled = %d\r\n", regValue[0]);
						}
						write_reg(rab_addr, 16, 0); // Store light conditions.
						rab_state = 1;
						break;

					case 1:
						write_reg(rab_addr, 13, 0xAA);
						write_reg(rab_addr, 14, 0xFF);
						break;
				}
				chThdSleepUntilWindowed(time, time + MS2ST(20)); // Refresh @ 50 Hz.
				break;

			case 6: // ESP32 UART communication test.
				sdPut(&SD3, toEsp32);
				len = sdReadTimeout(&SD3, &fromEsp32, 1, MS2ST(50));
				if(len > 0) {
					sdPut(&SDU1, fromEsp32);
				}
				chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
				break;

			case 7:
				communication_start((BaseSequentialStream *)&SDU1);
				 while (1) {
			        chThdSleepMilliseconds(1000);
			    }
				break;

			case 8: // Asercom protocol v2 (USB).
				run_asercom2();
				stop_loop = 1;
				break;

			case 9: // Asercom protocol.
				run_asercom();
				stop_loop = 1;
				break;

			case 10: // Gumstix extension.
				imu_stop();
				VL53L0X_stop();
				i2c_stop();
				stop_loop = 1;
				break;

			case 11: // Simple obstacle avoidance + some animation.
				messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
				leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
				rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);

	            switch(rgb_state) {
					case 0: // Red.
						set_rgb_led(0, 10, 0, 0);
						set_rgb_led(1, 10, 0, 0);
						set_rgb_led(2, 10, 0, 0);
						set_rgb_led(3, 10, 0, 0);
						break;
					case 1: // Green.
						set_rgb_led(0, 0, 10, 0);
						set_rgb_led(1, 0, 10, 0);
						set_rgb_led(2, 0, 10, 0);
						set_rgb_led(3, 0, 10, 0);
						break;
					case 2: // Blue.
						set_rgb_led(0, 0, 0, 10);
						set_rgb_led(1, 0, 0, 10);
						set_rgb_led(2, 0, 0, 10);
						set_rgb_led(3, 0, 0, 10);
						break;
	            }
				rgb_counter++;
				if(rgb_counter == 100) {
					rgb_counter = 0;
					rgb_state = (rgb_state+1)%3;
					set_body_led(2);
					set_front_led(2);
				}

				melody_counter++;
				if(melody_counter == 2000) {
					melody_counter = 0;
					melody_state = (melody_state+1)%NB_SONGS;
					playMelody(melody_state, ML_SIMPLE_PLAY, NULL);
				}

				chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
				break;

			case 12: // Hardware test.
				switch(hw_test_state) {
					case 0: // Init hardware.
						// Calibrate proximity.
						calibrate_ir();

						// Test audio.
						playMelody(MARIO, ML_SIMPLE_PLAY, NULL);

						// Test motors at low speed.
						left_motor_set_speed(150);
						right_motor_set_speed(150);

						// Init camera.
						po8030_advanced_config(FORMAT_RGB565, 240, 160, 160, 160, SUBSAMPLING_X4, SUBSAMPLING_X4);
						dcmi_disable_double_buffering();
						dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
						dcmi_prepare();

						// Calibrate IMU.
						calibrate_acc();
						calibrate_gyro();

						// Test all leds.
						set_body_led(1);
						set_front_led(1);
						set_led(4,1);
						set_rgb_led(0, 10, 0, 0);
						set_rgb_led(1, 10, 0, 0);
						set_rgb_led(2, 10, 0, 0);
						set_rgb_led(3, 10, 0, 0);

						hw_test_state = 1;
						break;

					case 1: // Test.
						chThdSleepUntilWindowed(time, time + MS2ST(50)); // Refresh @ 20 Hz.

			            switch(rgb_state) {
							case 0: // Red.
								set_rgb_led(0, 10, 0, 0);
								set_rgb_led(1, 10, 0, 0);
								set_rgb_led(2, 10, 0, 0);
								set_rgb_led(3, 10, 0, 0);
								break;
							case 1: // Green.
								set_rgb_led(0, 0, 10, 0);
								set_rgb_led(1, 0, 10, 0);
								set_rgb_led(2, 0, 10, 0);
								set_rgb_led(3, 0, 10, 0);
								break;
							case 2: // Blue.
								set_rgb_led(0, 0, 0, 10);
								set_rgb_led(1, 0, 0, 10);
								set_rgb_led(2, 0, 0, 10);
								set_rgb_led(3, 0, 0, 10);
								break;
			            }
						rgb_counter++;
						if(rgb_counter == 20) {
							rgb_counter = 0;
							rgb_state = (rgb_state+1)%3;
						}

						if (SDU1.config->usbp->state != USB_ACTIVE) { // Skip printing if port not opened.
							continue;
						}

						messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
						messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

						// Read IMU.
						chprintf((BaseSequentialStream *)&SDU1, "IMU\r\n");
				    	chprintf((BaseSequentialStream *)&SDU1, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n\n", imu_values.acc_raw[0], imu_values.acc_raw[1], imu_values.acc_raw[2], imu_values.gyro_raw[0], imu_values.gyro_raw[1], imu_values.gyro_raw[2]);

						// Read selector position.
				    	chprintf((BaseSequentialStream *)&SDU1, "SELECTOR\r\n");
				    	chprintf((BaseSequentialStream *)&SDU1, "%d\r\n\n", get_selector());

						// Read IR receiver.
				    	chprintf((BaseSequentialStream *)&SDU1, "IR RECEIVER\r\n");
				    	chprintf((BaseSequentialStream *)&SDU1, "check : 0x%x, address : 0x%x, data : 0x%x\r\n\n", ir_remote_get_toggle(), ir_remote_get_address(), ir_remote_get_data());

						// Read proximity sensors.
				    	chprintf((BaseSequentialStream *)&SDU1, "PROXIMITY\r\n");
				    	chprintf((BaseSequentialStream *)&SDU1, "%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d\r\n\n", prox_values.delta[0], prox_values.delta[1], prox_values.delta[2], prox_values.delta[3], prox_values.delta[4], prox_values.delta[5], prox_values.delta[6], prox_values.delta[7]);
				    	chprintf((BaseSequentialStream *)&SDU1, "AMBIENT\r\n");
				    	chprintf((BaseSequentialStream *)&SDU1, "%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d\r\n\n", prox_values.ambient[0], prox_values.ambient[1], prox_values.ambient[2], prox_values.ambient[3], prox_values.ambient[4], prox_values.ambient[5], prox_values.ambient[6], prox_values.ambient[7]);

						// Read microphones.
				    	chprintf((BaseSequentialStream *)&SDU1, "MICROPHONES\r\n");
				    	chprintf((BaseSequentialStream *)&SDU1, "%4d,%4d,%4d,%4d\r\n\n", mic_get_volume(0), mic_get_volume(1), mic_get_volume(2), mic_get_volume(3));

				    	// Read distance sensor.
				    	chprintf((BaseSequentialStream *)&SDU1, "DISTANCE SENSOR\r\n");
				    	chprintf((BaseSequentialStream *)&SDU1, "%d\r\n\n", VL53L0X_get_dist_mm());

						// Read camera.
				    	dcmi_capture_start();
						wait_image_ready();
						img_buff_ptr = dcmi_get_last_image_ptr();
						r = (int)img_buff_ptr[0]&0xF8;
			            g = (int)(img_buff_ptr[0]&0x07)<<5 | (img_buff_ptr[1]&0xE0)>>3;
			            b = (int)(img_buff_ptr[1]&0x1F)<<3;
			            chprintf((BaseSequentialStream *)&SDU1, "CAMERA\r\n");
			            chprintf((BaseSequentialStream *)&SDU1, "R=%3d, G=%3d, B=%3d\r\n\n", r, g, b);

			            printUcUsage((BaseSequentialStream *)&SDU1);

			            chThdSleepMilliseconds(100);
						break;
				}
				break;

			case 13: // Reflect the orientation on the LEDs around the robot.
				e_display_angle();
				chThdSleepMilliseconds(50);
				break;

			case 14: // Read magnetometer sensors values.
				switch(magneto_state) {
					case 0:
						mpu9250_magnetometer_setup();
						magneto_state = 1;
						break;

					case 1:
				    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
				    	if (SDU1.config->usbp->state != USB_ACTIVE) { // Skip printing if port not opened.
				    		continue;
				    	}
				    	chprintf((BaseSequentialStream *)&SDU1, "%Mx=%f My=%f Mz=%f\r\n", imu_values.magnetometer[0], imu_values.magnetometer[1], imu_values.magnetometer[2]);
				    	chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
						break;
				}
				break;

			case 15:
				chprintf((BaseSequentialStream *)&SD3, "battery=%d, %f V \r\n", get_battery_raw(), get_battery_voltage());
				chThdSleepUntilWindowed(time, time + MS2ST(500)); // Refresh @ 2 Hz.
				break;
		}
    }
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    parameter_namespace_declare(&parameter_root, NULL, NULL);

    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
	dcmi_start();
	po8030_start();
	motors_init();
	proximity_start();
	battery_level_start();
	dac_start();
	exti_start();
	imu_start();
	ir_remote_start();
	spi_comm_start();
	VL53L0X_start();
	serial_start();
	mic_start(NULL);
	sdio_start();
	playMelodyStart();
	playSoundFileStart();

	// Initialise Aseba system, declaring parameters
    parameter_namespace_declare(&aseba_ns, &parameter_root, "aseba");
    aseba_declare_parameters(&aseba_ns);

    /* Load parameter tree from flash. */
    load_config();

    /* Start AsebaCAN. Must be after config was loaded because the CAN id
     * cannot be changed at runtime. */
    aseba_vm_init();
    aseba_can_start(&vmState);

    chThdCreateStatic(selector_thd_wa, sizeof(selector_thd_wa), NORMALPRIO, selector_thd, NULL);

    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
