#ifndef ASEBA_CAN_INTERFACE_H
#define ASEBA_CAN_INTERFACE_H

#include "vm/vm.h"

#ifdef __cplusplus
extern "C" {
#endif

void aseba_can_start(AsebaVMState *vm_state);

void aseba_can_lock(void);
void aseba_can_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* ASEBA_CAN_INTERFACE_H */
