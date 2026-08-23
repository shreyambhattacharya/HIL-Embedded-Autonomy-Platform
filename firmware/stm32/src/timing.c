#include "timing.h"

#include <stdint.h>

#include "FreeRTOSConfig.h"

#define CORE_DEBUG_DEMCR (*(volatile uint32_t *)0xE000EDFCUL)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004UL)
#define CORE_DEBUG_DEMCR_TRCENA (1UL << 24U)
#define DWT_CTRL_CYCCNTENA (1UL << 0U)

void timing_init(void)
{
  CORE_DEBUG_DEMCR |= CORE_DEBUG_DEMCR_TRCENA;
  DWT_CYCCNT = 0U;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

uint32_t timing_now_cycles(void)
{
  return DWT_CYCCNT;
}

uint32_t timing_cycle_delta(uint32_t start, uint32_t end)
{
  return end - start;
}

uint32_t timing_cycles_to_us(uint32_t cycles)
{
  const uint32_t cycles_per_us = configCPU_CLOCK_HZ / 1000000U;
  return cycles_per_us == 0U ? 0U : cycles / cycles_per_us;
}
