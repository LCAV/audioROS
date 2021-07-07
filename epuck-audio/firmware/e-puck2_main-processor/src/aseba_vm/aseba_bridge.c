#include "hal.h"
#include "aseba_bridge.h"
#include "aseba_can_interface.h"

#include "transport/can/can-net.h"
#include "common/consts.h"
#include "chprintf.h"

static bool is_bridge = false;

static void aseba_bridge_uart_to_can(void *p);
static void aseba_bridge_can_to_uart(void *p);
static void led_thread(void *p);

typedef union {
    uint8_t u8[2];
    uint16_t u16;
} uint16_8_t;

void aseba_bridge(BaseSequentialStream *stream)
{
    is_bridge = true;
    static THD_WORKING_AREA(uart_to_can_wa, 1024);
    chThdCreateStatic(uart_to_can_wa, sizeof(uart_to_can_wa), NORMALPRIO,
                      aseba_bridge_uart_to_can, (void *)stream);

    static THD_WORKING_AREA(can_to_uart_wa, 1024);
    chThdCreateStatic(can_to_uart_wa, sizeof(can_to_uart_wa), NORMALPRIO,
                      aseba_bridge_can_to_uart, (void *)stream);

    static THD_WORKING_AREA(led_thread_wa, 256);
    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO,
                      led_thread, NULL);
}

static void aseba_bridge_uart_to_can(void *p)
{
    chRegSetThreadName("aseba uart -> can");
    BaseSequentialStream *stream = (BaseSequentialStream *)p;

    uint16_8_t source, length;
    uint8_t data[ASEBA_MAX_PACKET_SIZE];

    while (true) {
        chSequentialStreamRead(stream, length.u8, sizeof(length));
        chSequentialStreamRead(stream, source.u8, sizeof(source));
        chSequentialStreamRead(stream, data, length.u16 + 2);

        palTogglePad(GPIOD, GPIOD_LED3);

        aseba_can_lock();
        AsebaCanSendSpecificSource(data, length.u16 + 2, source.u16);
        aseba_can_unlock();
    }
}

static void aseba_bridge_can_to_uart(void *p)
{
    chRegSetThreadName("aseba can -> uart");
    BaseSequentialStream *stream = (BaseSequentialStream *)p;
    uint16_8_t source, length;
    uint8_t data[ASEBA_MAX_PACKET_SIZE];

    while (true) {
        length.u16 = AsebaCanRecv(data,
                                  ASEBA_MAX_INNER_PACKET_SIZE,
                                  &source.u16);

        if (length.u16 > 0) {
            palTogglePad(GPIOD, GPIOD_LED1);
            /* Aseba transmits length minus the type. */
            length.u16 -= 2;

            chSequentialStreamWrite(stream, length.u8, sizeof(source));
            chSequentialStreamWrite(stream, source.u8, sizeof(source));
            chSequentialStreamWrite(stream, data, length.u16 + 2);
        }
        chThdSleepMilliseconds(1);
    }
}

static void led_thread(void *p)
{
    (void) p;
    while (true) {
        palSetPad(GPIOD, GPIOD_LED1);
        chThdSleepMilliseconds(300);
        palClearPad(GPIOD, GPIOD_LED1);
        chThdSleepMilliseconds(300);
    }
}

bool aseba_is_bridge(void)
{
    return is_bridge;
}
