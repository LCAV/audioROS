#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <ch.h>

#include "cmp_mem_access/cmp_mem_access.h"

struct dispatcher_entry_s {
    const char *id;
    int (*cb)(cmp_ctx_t *cmp, void *arg);
    void *arg;
};

void communication_start(BaseSequentialStream *out);


#endif /* COMMUNICATION_H */