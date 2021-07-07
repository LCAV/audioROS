#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <hal.h>

#define MOTOR_SPEED_LIMIT 1100 // [step/s]

 /**
 * @brief   Sets the speed of the left motor
 * 
 * @param speed     speed desired in step/s
 */
void left_motor_set_speed(int speed);

 /**
 * @brief   Sets the speed of the right motor
 * 
 * @param speed     speed desired in step/s
 */
void right_motor_set_speed(int speed);

 /**
 * @brief   Reads the position counter of the left motor
 * 
 * @return          position counter of the left motor in step
 */
int32_t left_motor_get_pos(void);

 /**
 * @brief   Reads the position counter of the right motor
 * 
 * @return          position counter of the right motor in step
 */
int32_t right_motor_get_pos(void);

/**
 * @brief 	sets the position counter of the left motor to the given value
 * 
 * @param counter_value 	value to store in the position counter of the motor in step
 */
void left_motor_set_pos(int32_t counter_value);

/**
 * @brief 	sets the position counter of the right motor to the given value
 * 
 * @param counter_value value to store in the position counter of the motor in step
 */
void right_motor_set_pos(int32_t counter_value);

 /**
 * @brief   Initializes the control of the motors.
 */
void motors_init(void);

#endif /* MOTOR_H */
