#ifndef __TASK3_ASYM_MOVE_H__
#define __TASK3_ASYM_MOVE_H__

#include "headfile.h"

/* Non-symmetric fine-motion experiment around the mechanical balance point. */
void task3_asym_move_init(void);
void task3_asym_move_update(uint16_t dt_ms);

#endif
