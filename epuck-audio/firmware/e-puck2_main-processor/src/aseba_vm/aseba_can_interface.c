#include "ch.h"
#include "hal.h"

#include "transport/can/can-net.h"
#include "vm/vm.h"

#include "aseba_can_interface.h"

#define ASEBA_CAN_SEND_QUEUE_SIZE       1024
#define ASEBA_CAN_RECEIVE_QUEUE_SIZE    1024



CanFrame aseba_can_send_queue[ASEBA_CAN_SEND_QUEUE_SIZE];
CanFrame aseba_can_receive_queue[ASEBA_CAN_RECEIVE_QUEUE_SIZE];

static THD_WORKING_AREA(can_rx_thread_wa, 256);
static THD_FUNCTION(can_rx_thread, arg)
{
    (void)arg;
    chRegSetThreadName("CAN rx");
    while (1) {
        CANRxFrame rxf;
        CanFrame aseba_can_frame;
        msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, MS2ST(1000));
        if (m != MSG_OK) {
            continue;
        }
        if (rxf.IDE) {
            continue; // no extended id frames
        }
        if (rxf.RTR) {
            continue; // no remote transmission request frames
        }
        aseba_can_frame.id = rxf.SID;
        aseba_can_frame.len = rxf.DLC;
        int i;
        for (i = 0; i < aseba_can_frame.len; i++) {
            aseba_can_frame.data[i] = rxf.data8[i];
        }
        AsebaCanFrameReceived(&aseba_can_frame);
    }
}

void can_init(void)
{

    static const CANConfig can1_config = {
        .mcr = (1 << 6)  /* Automatic bus-off management enabled. */
               | (1 << 2), /* Message are prioritized by order of arrival. */

        /* APB Clock is 42 Mhz
           42MHz / 2 / (1tq + 12tq + 8tq) = 1MHz => 1Mbit */
        .btr = (1 << 0)  /* Baudrate prescaler (10 bits) */
               | (11 << 16)/* Time segment 1 (3 bits) */
               | (7 << 20) /* Time segment 2 (3 bits) */
               | (0 << 24) /* Resync jump width (2 bits) */
    };

    canStart(&CAND1, &can1_config);
}

void aseba_can_rx_dropped(void)
{
}

void aseba_can_tx_dropped(void)
{
}

void aseba_can_send_frame(const CanFrame *frame)
{
    aseba_can_lock();
    CANTxFrame txf;
    txf.DLC = frame->len;
    txf.RTR = 0;
    txf.IDE = 0;
    txf.SID = frame->id;

    int i;
    for (i = 0; i < frame->len; i++) {
        txf.data8[i] = frame->data[i];
    }

    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(100));
    chThdSleepMilliseconds(1);

    AsebaCanFrameSent();
    aseba_can_unlock();
}

// Returns true if there is enough space to send the frame
int aseba_can_is_frame_room(void)
{
    return can_lld_is_tx_empty(&CAND1, CAN_ANY_MAILBOX);
}

void aseba_can_start(AsebaVMState *vm_state)
{
    can_init();
    chThdCreateStatic(can_rx_thread_wa,
                      sizeof(can_rx_thread_wa),
                      NORMALPRIO + 1,
                      can_rx_thread,
                      NULL);
    AsebaCanInit(vm_state->nodeId, aseba_can_send_frame, aseba_can_is_frame_room,
                 aseba_can_rx_dropped, aseba_can_tx_dropped,
                 aseba_can_send_queue, ASEBA_CAN_SEND_QUEUE_SIZE,
                 aseba_can_receive_queue, ASEBA_CAN_RECEIVE_QUEUE_SIZE);
}

static MUTEX_DECL(can_lock);

void aseba_can_lock(void)
{
    chMtxLock(&can_lock);
}

void aseba_can_unlock(void)
{
    chMtxUnlock(&can_lock);
}
