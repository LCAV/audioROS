#ifndef ASEBA_NODE_H
#define ASEBA_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common/types.h"
#include "vm/vm.h"
#include "parameter/parameter.h"

/** Number of opcodes in an aseba bytecode script.
 *
 * @note Should be a multiple of (766 + 768). */
#define VM_BYTECODE_SIZE (766 + 768)
#define VM_STACK_SIZE 128

/*
 * In your code, put "SET_EVENT(EVENT_NUMBER)" when you want to trigger an
 * event. This macro is interrupt-safe, you can call it anywhere you want.
 *
 * FIXME: On STM32 This is *not* IRQ safe.
 */
#define SET_EVENT(event) (events_flags |= (1 << event))
#define CLEAR_EVENT(event) (events_flags &= ~(1 << event))
#define IS_EVENT(event) (events_flags & (1 << event))

extern unsigned int events_flags;
extern AsebaVMState vmState;

void aseba_vm_start(void);
void aseba_vm_init(void);

/** Declares all the parameters used by the Aseba subsystem. */
void aseba_declare_parameters(parameter_namespace_t *aseba_ns);

#ifdef __cplusplus
}
#endif

#endif
