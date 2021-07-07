#include "ch.h"
#include "hal.h"
#include "exti.h"

event_source_t exti_events;

/***************************INTERNAL FUNCTIONS************************************/

 /**
 * @brief   Callback called when an exti occurs.
 *          Broadcasts on the exti_events the flag triggered
 */
static void gpio_exti_callback(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    if (channel == GPIOD_REMOTE) {  // Channel IR remote control.
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_IR_REMOTE_INT);
        chSysUnlockFromISR();
    } else if(channel == GPIOD_IMU_INT) {
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_IMU_INT);
        chSysUnlockFromISR();
    }
}

//Exti configuration
static const EXTConfig extcfg = {
    {
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD, gpio_exti_callback}, // IR remote control
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD, gpio_exti_callback}, // IMU interrupt
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL}
    }
};

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void exti_start(void) {
	chEvtObjectInit(&exti_events);
	extStart(&EXTD1, &extcfg);
}

void exti_disable_ir_remote(void) {
	extChannelDisableI(&EXTD1, 3);
}

void exti_enable_ir_remote(void) {
	extChannelEnableI(&EXTD1, 3);
}

/**************************END PUBLIC FUNCTIONS***********************************/
