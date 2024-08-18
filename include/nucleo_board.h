#ifndef NUCLEO_BOARD_H
#define NUCLEO_BOARD_H

#include "stdint.h"

void NB_Init(void);
void NB_User_LED_Blinking(uint8_t repetitions, uint16_t period_ms);

#endif 