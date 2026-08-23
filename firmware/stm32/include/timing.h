#ifndef HIL_STM32_TIMING_H
#define HIL_STM32_TIMING_H

#include <stdint.h>

void timing_init(void);
uint32_t timing_now_cycles(void);
uint32_t timing_cycle_delta(uint32_t start, uint32_t end);
uint32_t timing_cycles_to_us(uint32_t cycles);

#endif
