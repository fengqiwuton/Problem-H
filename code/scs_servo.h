#ifndef __SCS_SERVO_H__
#define __SCS_SERVO_H__

#include "headfile.h"

/* SCS串行总线舵机驱动 (飞特SC09等)
 * 协议: FF FF ID Len Cmd Params... Checksum
 * 接线: PB10(UART3_TX) → 舵机信号线
 *       舵机需要独立供电(5~8.4V)
 */

void scs_init(uint8_t servo_id);
void scs_write_pos(uint8_t id, uint16_t pos, uint16_t time, uint16_t speed);
void scs_torque_enable(uint8_t id, uint8_t enable);

/* 工作范围 */
#define SCS_POS_MIN      453
#define SCS_POS_MAX      760
#define SCS_POS_CENTER   670

#endif
