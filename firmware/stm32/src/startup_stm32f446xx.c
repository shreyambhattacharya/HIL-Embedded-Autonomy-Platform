#include <stdint.h>

typedef void (*isr_handler_t)(void);

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern int main(void);
extern void SystemInit(void);
extern void USART2_IRQHandler(void);

void Reset_Handler(void);
void Default_Handler(void);

void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

/* GNU range designator fills unused external IRQ slots with the safe handler. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Woverride-init"
__attribute__((section(".isr_vector"), used))
const uintptr_t vector_table[82] = {
  [0] = (uintptr_t)&_estack,
  [1] = (uintptr_t)Reset_Handler,
  [2] = (uintptr_t)NMI_Handler,
  [3] = (uintptr_t)HardFault_Handler,
  [4] = (uintptr_t)MemManage_Handler,
  [5] = (uintptr_t)BusFault_Handler,
  [6] = (uintptr_t)UsageFault_Handler,
  [11] = (uintptr_t)SVC_Handler,
  [12] = (uintptr_t)DebugMon_Handler,
  [14] = (uintptr_t)PendSV_Handler,
  [15] = (uintptr_t)SysTick_Handler,
  [16 ... 81] = (uintptr_t)Default_Handler,
  [16 + 38] = (uintptr_t)USART2_IRQHandler,
};

#pragma GCC diagnostic pop

void Reset_Handler(void)
{
  uint32_t *source = &_sidata;
  for (uint32_t *destination = &_sdata; destination < &_edata; ++destination, ++source) {
    *destination = *source;
  }
  for (uint32_t *destination = &_sbss; destination < &_ebss; ++destination) {
    *destination = 0U;
  }
  SystemInit();
  (void)main();
  for (;;) {
  }
}

void Default_Handler(void)
{
  for (;;) {
  }
}
