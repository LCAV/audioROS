#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"
#include <string.h>
#include <stdio.h>

#include "common/types.h"
#include "common/consts.h"
#include "transport/buffer/vm-buffer.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_node.h"
#include "aseba_vm/aseba_bridge.h"
#include "flash/flash.h"

void update_aseba_variables_read(void);
void update_aseba_variables_write(void);
sint16 aseba_float_to_int(float var, float max);

unsigned int events_flags = 0;
static uint16 vmBytecode[VM_BYTECODE_SIZE];
static sint16 vmStack[VM_STACK_SIZE];

static parameter_t nodeId_param;

AsebaVMState vmState = {
    .nodeId = 0, /* changed by aseba_vm_init() */

    .bytecodeSize = VM_BYTECODE_SIZE,
    .bytecode = vmBytecode,

    .variablesSize = sizeof(vmVariables) / sizeof(sint16),
    .variables = (sint16*)&vmVariables,

    .stackSize = VM_STACK_SIZE,
    .stack = vmStack,
    .flags = 0, .pc = 0, .sp = 0,
    .breakpoints = {0}, .breakpointsCount = 0,
};


static THD_FUNCTION(aseba_vm_thd, arg)
{
    (void)arg;

    chRegSetThreadName("aseba");

    AsebaVMSetupEvent(&vmState, ASEBA_EVENT_INIT);

    while (TRUE) {
        // Don't spin too fast to avoid consuming all CPU time
        chThdYield();

        // Sync Aseba with the state of the system
        aseba_read_variables_from_system(&vmState);

        // Run VM for some time
        AsebaVMRun(&vmState, 1000);
        AsebaProcessIncomingEvents(&vmState);

        // Sync the system with the state of Aseba
        aseba_write_variables_to_system(&vmState);

        // Do not process events in step by step mode
        if (AsebaMaskIsSet(vmState.flags, ASEBA_VM_STEP_BY_STEP_MASK)) {
            continue;
        }

        // If we are not executing an event, there is nothing to do
        if (AsebaMaskIsSet(vmState.flags, ASEBA_VM_EVENT_ACTIVE_MASK)) {
            continue;
        }

        int event = ffs(events_flags) - 1;

        // If a local event is pending, then execute it.
        if (event != -1) {

            CLEAR_EVENT(event);

            vmVariables.source = vmState.nodeId;

            AsebaVMSetupEvent(&vmState, ASEBA_EVENT_LOCAL_EVENTS_START - event);
        }
    }
}

void aseba_vm_init(void)
{
    uint16_t bytecode_size;

    vmState.nodeId = parameter_integer_get(&nodeId_param);

    AsebaVMInit(&vmState);

    extern uint8_t _aseba_bytecode_start;
    uint8_t *pos = &_aseba_bytecode_start;

    memcpy(&bytecode_size, pos, sizeof(bytecode_size));

    pos += sizeof(uint16_t);

    // Check if the bytecode page was erased
    if (bytecode_size != 0xffff) {
        memcpy(vmState.bytecode, pos, bytecode_size);
    }

    chThdSleepMilliseconds(500);
}

void aseba_declare_parameters(parameter_namespace_t *aseba_ns)
{
    parameter_integer_declare_with_default(&nodeId_param, aseba_ns, "id", 42);

    aseba_variables_init(aseba_ns);
}

void aseba_vm_start(void)
{
    static THD_WORKING_AREA(aseba_vm_thd_wa, 1024);
    chThdCreateStatic(aseba_vm_thd_wa, sizeof(aseba_vm_thd_wa), LOWPRIO, aseba_vm_thd, NULL);
}

void AsebaIdle(void)
{
    chThdYield();
}

void AsebaPutVmToSleep(AsebaVMState *vm)
{
    (void) vm;
    chThdSleepMilliseconds(1000);
}

void AsebaResetIntoBootloader(AsebaVMState *vm)
{
    (void) vm;
    NVIC_SystemReset();
}

void AsebaNativeFunction(AsebaVMState *vm, uint16 id)
{
    if (id < nativeFunctions_length) {
        nativeFunctions[id](vm);
    } else {
        AsebaVMEmitNodeSpecificError(vm, "Invalid native function.");
    }
}

const AsebaNativeFunctionDescription * const * AsebaGetNativeFunctionsDescriptions(AsebaVMState *vm)
{
    (void) vm;
    return nativeFunctionsDescription;
}


const AsebaVMDescription* AsebaGetVMDescription(AsebaVMState *vm)
{
    (void) vm;
    return &vmDescription;
}

const AsebaLocalEventDescription * AsebaGetLocalEventsDescriptions(AsebaVMState *vm)
{
    (void) vm;
    return localEvents;
}


uint16 AsebaShouldDropPacket(uint16 source, const uint8* data)
{
    /* Accept all packets in bridge mode. */
    if (aseba_is_bridge()) {
        return 0;
    }

    return AsebaVMShouldDropPacket(&vmState, source, data);
}

void AsebaWriteBytecode(AsebaVMState *vm)
{
    extern uint8_t _aseba_bytecode_start;

    flash_unlock();
    flash_sector_erase(&_aseba_bytecode_start);
    flash_write(&_aseba_bytecode_start, &vm->bytecodeSize, sizeof(uint16));
    flash_write(&_aseba_bytecode_start + sizeof(uint16), vm->bytecode, vm->bytecodeSize);
    flash_lock();
}
