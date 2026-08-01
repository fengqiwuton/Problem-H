#ifndef __BALANCE_CALIBRATION_H__
#define __BALANCE_CALIBRATION_H__

#include "headfile.h"

/* Manual balance-point calibration for the water-pipe linkage.
 * PB1 raises the commanded servo position; PA6 lowers it. */
void balance_calibration_init(void);
void balance_calibration_update(uint16_t dt_ms);

#endif
