#ifndef __SUPER_H
#define __SUPER_H

#include "stdint.h"
#include "stdbool.h"

void wave_hight_init(void);
float recv_data(void);
void DebugUartInit(void);
void receive_pid_data(uint8_t data[96],uint8_t *flag);

#endif
