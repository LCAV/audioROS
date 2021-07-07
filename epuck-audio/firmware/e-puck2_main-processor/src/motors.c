#include <ch.h>
#include <hal.h>
#include "motors.h"
#include "leds.h"

#define MOTOR_TIMER_FREQ 100000 // [Hz]
#define THRESV 650 // This is the speed under which the power save feature is active.

static const uint8_t step_halt[4] = {0, 0, 0, 0};
//table of the differents steps of to control the motors
//it corresponds to microsteps.
//in the reality two microsteps correspond to 1 step of the motor.
static const uint8_t step_table[8][4] = {
    {1, 0, 1, 0},
	{0, 0, 1, 0},
    {0, 1, 1, 0},
	{0, 1, 0, 0},
    {0, 1, 0, 1},
	{0, 0, 0, 1},
    {1, 0, 0, 1},
	{1, 0, 0, 0},
};

struct stepper_motor_s {
    enum {
        HALT=0,
        FORWARD=1,
        BACKWARD=2
    } direction;
    uint8_t step_index;
    int32_t count;  //in microsteps
    void (*update)(const uint8_t *out);
    void (*enable_power_save)(void);
    void (*disable_power_save)(void);
    PWMDriver *timer;
};

struct stepper_motor_s right_motor;
struct stepper_motor_s left_motor;

/***************************INTERNAL FUNCTIONS************************************/

 /**
 * @brief   Updates the right motor state
 *
 * @param[in] out       pointer to the table containing the next state
 *
 */
static void right_motor_update(const uint8_t *out)
{
    /* right motor */
    out[0] ? palSetPad(GPIOE, GPIOE_MOT_R_IN1) : palClearPad(GPIOE, GPIOE_MOT_R_IN1);
    out[1] ? palSetPad(GPIOE, GPIOE_MOT_R_IN2) : palClearPad(GPIOE, GPIOE_MOT_R_IN2);
    out[2] ? palSetPad(GPIOE, GPIOE_MOT_R_IN3) : palClearPad(GPIOE, GPIOE_MOT_R_IN3);
    out[3] ? palSetPad(GPIOE, GPIOE_MOT_R_IN4) : palClearPad(GPIOE, GPIOE_MOT_R_IN4);
}

 /**
 * @brief   Updates the left motor state
 *
 * @param[in] out       pointer to the table containing the next state
 *
 */
static void left_motor_update(const uint8_t *out)
{
    /* left motor */
    out[0] ? palSetPad(GPIOE, GPIOE_MOT_L_IN1) : palClearPad(GPIOE, GPIOE_MOT_L_IN1);
    out[1] ? palSetPad(GPIOE, GPIOE_MOT_L_IN2) : palClearPad(GPIOE, GPIOE_MOT_L_IN2);
    out[2] ? palSetPad(GPIOE, GPIOE_MOT_L_IN3) : palClearPad(GPIOE, GPIOE_MOT_L_IN3);
    out[3] ? palSetPad(GPIOE, GPIOE_MOT_L_IN4) : palClearPad(GPIOE, GPIOE_MOT_L_IN4);
}

 /**
 * @brief   Callback that updates the state of the right motor
 *
 * @param[in] gptp       pointer of the PWM driver (not used)
 *
 */
static void right_motor_timer_callback(PWMDriver *gptp)
{
    (void) gptp;
    uint8_t i;
    if (right_motor.direction == BACKWARD) {
        i = (right_motor.step_index + 1) & 7;
        right_motor.update(step_table[i]);
        right_motor.count -= 1;
        right_motor.step_index = i;
    } else if (right_motor.direction == FORWARD) {
        i = (right_motor.step_index - 1) & 7;
        right_motor.update(step_table[i]);
        right_motor.count += 1;
        right_motor.step_index = i;
    } else {
        right_motor.update(step_halt);
    }
}

 /**
 * @brief   Callback that updates the state of the left motor
 *
 * @param[in] gptp       pointer of the PWM driver (not used)
 *
 */
static void left_motor_timer_callback(PWMDriver *gptp)
{
    (void) gptp;
    uint8_t i;
    if (left_motor.direction == FORWARD) { // Inverted for the two motors
        i = (left_motor.step_index + 1) & 7;
        left_motor.update(step_table[i]);
        left_motor.count += 1;
        left_motor.step_index = i;
    } else if (left_motor.direction == BACKWARD) {
        i = (left_motor.step_index - 1) & 7;
        left_motor.update(step_table[i]);
        left_motor.count -= 1;
        left_motor.step_index = i;
    } else {
        left_motor.update(step_halt);
    }
}

 /**
 * @brief   Callback that turns off the power of the right motor
 *          after a certain time to save energy. Called if power_save is enabled
 *
 * @param[in] pwmp       pointer of the PWM driver (not used)
 *
 */
static void right_motor_pwm_ch1_cb(PWMDriver *pwmp) {
	(void)pwmp;
    palClearPad(GPIOE, GPIOE_MOT_R_IN1);
    palClearPad(GPIOE, GPIOE_MOT_R_IN2);
    palClearPad(GPIOE, GPIOE_MOT_R_IN3);
    palClearPad(GPIOE, GPIOE_MOT_R_IN4);
}

 /**
 * @brief   Callback that turns off the power of the left motor
 *          after a certain time to save energy. Called if power_save is enabled
 *
 * @param[in] pwmp       pointer of the PWM driver (not used)
 *
 */
static void left_motor_pwm_ch1_cb(PWMDriver *pwmp) {
	(void)pwmp;
    palClearPad(GPIOE, GPIOE_MOT_L_IN1);
    palClearPad(GPIOE, GPIOE_MOT_L_IN2);
    palClearPad(GPIOE, GPIOE_MOT_L_IN3);
    palClearPad(GPIOE, GPIOE_MOT_L_IN4);
}

 /**
 * @brief   Enables the power save feature of the right motor
 */
void right_motor_enable_power_save(void) {
    //Enable channel 1 to set duty cycle for power save.
    pwmEnableChannel(&PWMD3, 0, (pwmcnt_t) (MOTOR_TIMER_FREQ/THRESV)); 
    //Channel 1 interrupt enable to handle motor shutdown.
	pwmEnableChannelNotification(&PWMD3, 0); 
}

 /**
 * @brief   Enables the power save feature of the left motor
 */
void left_motor_enable_power_save(void) {
    //Enable channel 1 to set duty cycle for power save.
    pwmEnableChannel(&PWMD4, 0, (pwmcnt_t) (MOTOR_TIMER_FREQ/THRESV)); 
    //Channel 1 interrupt enable to handle motor shutdown.
	pwmEnableChannelNotification(&PWMD4, 0); 
}

 /**
 * @brief   Disables the power save feature of the right motor
 */
void right_motor_disable_power_save(void) {
	pwmDisableChannel(&PWMD3, 0);
}

 /**
 * @brief   Disables the power save feature of the left motor
 */
void left_motor_disable_power_save(void) {
	pwmDisableChannel(&PWMD4, 0);
}

 /**
 * @brief   Sets the speed of the chosen motor
 * 
 * @param m         pointer to the motor. See stepper_motor_s
 * @param speed     speed desired in step/s
 */
void motor_set_speed(struct stepper_motor_s *m, int speed)
{
    /* limit motor speed */
    if (speed > MOTOR_SPEED_LIMIT) {
        speed = MOTOR_SPEED_LIMIT;
    } else if (speed < -MOTOR_SPEED_LIMIT) {
        speed = -MOTOR_SPEED_LIMIT;
    }
    //twice the speed because we are doing microsteps, 
    //which doubles the steps necessary to do one real step of the motor
    speed *=2;

    uint16_t interval;
    if (speed == 0) {
        m->direction = HALT;
        //Resolves a problem when the motors take about 650ms to restart
        interval = 1000;    //so the motors get updated at 100Hz when not used
        m->disable_power_save();
    } else {
        if (speed > 0) {
            m->direction = FORWARD;
        } else {
            m->direction = BACKWARD;
            speed = -speed;
        }
        interval = MOTOR_TIMER_FREQ / speed;

        if(speed < THRESV) {
        	m->enable_power_save();
        } else {
        	m->disable_power_save();
        }
    }

    /* change motor step interval */
    pwmChangePeriod(m->timer, interval);
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void left_motor_set_speed(int speed) {
    motor_set_speed(&left_motor, speed);
}

void right_motor_set_speed(int speed) {
	motor_set_speed(&right_motor, speed);
}

int32_t left_motor_get_pos(void) {
    return left_motor.count/2;  //to return the real number of steps
}

int32_t right_motor_get_pos(void) {
	return right_motor.count/2; //to return the real number of steps
}

void left_motor_set_pos(int32_t counter_value){
    left_motor.count = counter_value*2; //converts steps to microsteps
}

void right_motor_set_pos(int32_t counter_value){
    right_motor.count = counter_value*2; //converts steps to microsteps
}

void motors_init(void)
{
    /* motor struct init */
    right_motor.direction = HALT;
    right_motor.step_index = 0;
    right_motor.count = 0;
    right_motor.update = right_motor_update;
    right_motor.enable_power_save = right_motor_enable_power_save;
    right_motor.disable_power_save = right_motor_disable_power_save;
    right_motor.timer = &PWMD3;

    left_motor.direction = HALT;
    left_motor.step_index = 0;
    left_motor.count = 0;
    left_motor.update = left_motor_update;
    left_motor.enable_power_save = left_motor_enable_power_save;
    left_motor.disable_power_save = left_motor_disable_power_save;
    left_motor.timer = &PWMD4;

    /* motor init halted*/
    right_motor_update(step_halt);
    left_motor_update(step_halt);

    /* timer init */
    static const PWMConfig pwmcfg_right_motor = {
        .frequency = MOTOR_TIMER_FREQ,
        .period = 0xFFFF,
        .cr2 = 0,
        .callback = right_motor_timer_callback,
        .channels = {
            // Channel 1 is used to handling motor power save feature.
            {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = right_motor_pwm_ch1_cb},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
        },
    };
    pwmStart(&PWMD3, &pwmcfg_right_motor);
    pwmEnablePeriodicNotification(&PWMD3); // PWM general interrupt at the beginning of the period to handle motor steps.

    static const PWMConfig pwmcfg_left_motor = {
        .frequency = MOTOR_TIMER_FREQ,
        .period = 0xFFFF,
        .cr2 = 0,
        .callback = left_motor_timer_callback,
        .channels = {
            // Channel 1 is used to handling motor power save feature.
            {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = left_motor_pwm_ch1_cb},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
        },
    };
    pwmStart(&PWMD4, &pwmcfg_left_motor);
    pwmEnablePeriodicNotification(&PWMD4); // PWM general interrupt at the beginning of the period to handle motor steps.

}

/**************************END PUBLIC FUNCTIONS***********************************/
