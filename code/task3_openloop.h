#ifndef __TASK3_OPENLOOP_H__
#define __TASK3_OPENLOOP_H__

#include "headfile.h"

/* Fixed-time trajectory experiment for Task 3. */
void task3_openloop_init(void);
void task3_openloop_update(uint16_t dt_ms);

#endif
