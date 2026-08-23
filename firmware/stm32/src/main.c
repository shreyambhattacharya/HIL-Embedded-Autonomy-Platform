#include "stm32f446xx.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hil_control.h"
#include "hil_protocol.h"
#include "hil_safety.h"
#include "timing.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
#include "task.h"

#define RX_STREAM_CAPACITY 512U
#define TX_QUEUE_LENGTH 8U
#ifndef UART_BAUD_RATE
#define UART_BAUD_RATE 115200U
#endif
#define CONTROL_PERIOD_MS 10U
#define CONTROL_PERIOD_US 10000U
#define COMMAND_TIMEOUT_TICKS pdMS_TO_TICKS(100U)
#define FEEDBACK_TIMEOUT_TICKS pdMS_TO_TICKS(50U)
#define IWDG_PRESCALER_CODE 4U
#define IWDG_RELOAD 249U
#define WATCHDOG_TEST_HOLD_MS 3000U
#ifndef HIL_TEST_WATCHDOG
#define HIL_TEST_WATCHDOG 0
#endif

typedef struct {
  float linear_m_s;
  float yaw_rad_s;
  float left_feedback_rad_s;
  float right_feedback_rad_s;
  float left_effort_nm;
  float right_effort_nm;
  TickType_t last_command_tick;
  TickType_t last_feedback_tick;
  uint32_t boot_id;
  uint8_t state;
  uint8_t fault;
  bool link_seen;
  bool have_command;
  bool have_feedback;
  hil_safety_state_t safety;
  uint8_t reset_cause;
} controller_context_t;
typedef struct {
  uint32_t sample_count;
  uint32_t execution_min_us;
  uint32_t execution_max_us;
  uint64_t execution_sum_us;
  uint32_t period_min_us;
  uint32_t period_max_us;
  uint64_t period_sum_us;
  uint32_t period_sample_count;
  uint32_t previous_activation_cycles;
  bool have_previous_activation;
  uint32_t deadline_misses;
  volatile uint32_t control_progress;
  volatile uint32_t rx_stream_drops;
  volatile uint32_t tx_queue_drops;
  volatile uint32_t uart_overruns;
} runtime_metrics_t;

static controller_context_t controller = {
  .state = HIL_STATE_WAIT_LINK,
};
static StaticStreamBuffer_t rx_stream_struct;
static uint8_t rx_stream_storage[RX_STREAM_CAPACITY];
static StreamBufferHandle_t rx_stream;
static StaticQueue_t tx_queue_struct;
static uint8_t tx_queue_storage[TX_QUEUE_LENGTH * sizeof(hil_protocol_frame_t)];
static QueueHandle_t tx_queue;
static StaticTask_t rx_task_struct;
static StaticTask_t control_task_struct;
static StaticTask_t tx_task_struct;
static StackType_t rx_task_stack[384];
static StackType_t control_task_stack[384];
static StackType_t tx_task_stack[384];
static StaticTask_t idle_task_struct;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
static hil_protocol_decoder_t decoder;
static uint32_t tx_sequence = 0U;
static uint32_t boot_counter __attribute__((section(".noinit")));
static StaticTask_t supervisor_task_struct;
static StackType_t supervisor_task_stack[256];
static TaskHandle_t rx_task_handle;
static TaskHandle_t control_task_handle;
static TaskHandle_t tx_task_handle;
static TaskHandle_t supervisor_task_handle;
static runtime_metrics_t runtime_metrics;

static void put_u32(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value & 0xffU);
  out[1] = (uint8_t)((value >> 8U) & 0xffU);
  out[2] = (uint8_t)((value >> 16U) & 0xffU);
  out[3] = (uint8_t)((value >> 24U) & 0xffU);
}
static uint32_t uptime_ms(void)
{
  return (uint32_t)xTaskGetTickCount();
}
static void sync_safety_legacy(void)
{
  controller.state = controller.safety.state;
  controller.fault = controller.safety.reason;
  controller.link_seen = controller.safety.link_seen;
  controller.have_command = controller.safety.have_command;
  controller.have_feedback = controller.safety.have_feedback;
}

static uint8_t capture_reset_cause(void)
{
  const uint32_t flags = RCC->CSR;
  uint8_t cause = HIL_RESET_CAUSE_UNKNOWN;
  if ((flags & RCC_CSR_IWDGRSTF) != 0U) {
    cause = HIL_RESET_CAUSE_IWDG;
  } else if ((flags & RCC_CSR_SFTRSTF) != 0U) {
    cause = HIL_RESET_CAUSE_SOFTWARE;
  } else if ((flags & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) != 0U) {
    cause = HIL_RESET_CAUSE_POWER_ON;
  }
  RCC->CSR |= RCC_CSR_RMVF;
  return cause;
}

static void iwdg_start(void)
{
  IWDG->KR = 0x5555U;
  IWDG->PR = IWDG_PRESCALER_CODE;
  IWDG->RLR = IWDG_RELOAD;
  IWDG->KR = 0xAAAAU;
  IWDG->KR = 0xCCCCU;
}

static void iwdg_refresh(void)
{
  IWDG->KR = 0xAAAAU;
}

static uint16_t saturate_u16(UBaseType_t value)
{
  return value > (UBaseType_t)UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

static uint32_t mean_u64(uint64_t sum, uint32_t count)
{
  return count == 0U ? 0U : (uint32_t)(sum / (uint64_t)count);
}

void SystemInit(void)
{
  RCC->CR |= (1UL << 0U);
  while ((RCC->CR & (1UL << 1U)) == 0U) {
  }
  RCC->CFGR = 0U;
}

static void led_set(bool on)
{
  GPIOA->BSRR = on ? (1UL << 5U) : (1UL << (5U + 16U));
}

static void gpio_uart_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  (void)RCC->AHB1ENR;

  GPIOA->MODER &= ~((3UL << (2U * 2U)) | (3UL << (3U * 2U)) | (3UL << (5U * 2U)));
  GPIOA->MODER |= (2UL << (2U * 2U)) | (2UL << (3U * 2U)) | (1UL << (5U * 2U));
  GPIOA->OTYPER &= ~((1UL << 2U) | (1UL << 3U) | (1UL << 5U));
  GPIOA->OSPEEDR |= (3UL << (2U * 2U)) | (3UL << (3U * 2U)) | (3UL << (5U * 2U));
  GPIOA->PUPDR &= ~((3UL << (2U * 2U)) | (3UL << (3U * 2U)));
  GPIOA->PUPDR |= (1UL << (2U * 2U)) | (1UL << (3U * 2U));
  GPIOA->AFR[0] &= ~((0xFUL << (2U * 4U)) | (0xFUL << (3U * 4U)));
  GPIOA->AFR[0] |= (7UL << (2U * 4U)) | (7UL << (3U * 4U));
  led_set(false);

  USART2->BRR = (configCPU_CLOCK_HZ + (UART_BAUD_RATE / 2U)) / UART_BAUD_RATE;
  USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
  USART2->CR2 = 0U;
  USART2->CR3 = 0U;
  NVIC_IPR[38U] = (uint8_t)configMAX_SYSCALL_INTERRUPT_PRIORITY;
  NVIC_ISER1 |= (1UL << (38U - 32U));
}

static void uart_write_byte(uint8_t byte)
{
  while ((USART2->SR & USART_SR_TXE) == 0U) {
  }
  USART2->DR = byte;
}

static void uart_write_frame(hil_protocol_frame_t frame)
{
  frame.protocol_version = HIL_PROTOCOL_VERSION;
  frame.sequence = tx_sequence++;
  frame.sender_tick_ms = uptime_ms();
  uint8_t encoded[HIL_PROTOCOL_MAX_ENCODED_FRAME];
  size_t encoded_length = 0U;
  if (!hil_protocol_encode_frame(&frame, encoded, sizeof(encoded), &encoded_length)) {
    taskENTER_CRITICAL();
    hil_safety_internal_error(&controller.safety);
    sync_safety_legacy();
    taskEXIT_CRITICAL();
    return;
  }
  for (size_t i = 0U; i < encoded_length; ++i) {
    uart_write_byte(encoded[i]);
  }
}

static void queue_frame(const hil_protocol_frame_t *frame)
{
  if (xQueueSend(tx_queue, frame, 0U) != pdTRUE) {
    ++runtime_metrics.tx_queue_drops;
  }
}

static void queue_ack(uint8_t command_type, uint32_t transaction, uint8_t result)
{
  hil_protocol_frame_t frame;
  if (hil_protocol_pack_ack(&frame, command_type, transaction, result)) {
    queue_frame(&frame);
  }
}

static void queue_pong(uint32_t token)
{
  hil_protocol_frame_t frame;
  if (hil_protocol_pack_ping(&frame, token)) {
    frame.message_type = HIL_MSG_PONG;
    queue_frame(&frame);
  }
}

static void process_hello(const hil_protocol_frame_t *frame)
{
  if (frame->payload_length < 10U || frame->payload[0] != HIL_ROLE_LINUX_BRIDGE ||
    frame->payload[1] != HIL_PROTOCOL_VERSION) {
    taskENTER_CRITICAL();
    controller.safety.state = HIL_STATE_WAIT_LINK;
    controller.safety.reason = HIL_SAFETY_PROTOCOL_INCOMPATIBLE;
    sync_safety_legacy();
    taskEXIT_CRITICAL();
    return;
  }
  taskENTER_CRITICAL();
  hil_safety_on_hello(&controller.safety);
  sync_safety_legacy();
  taskEXIT_CRITICAL();
}

static void process_mode(const hil_protocol_frame_t *frame)
{
  uint8_t mode = 0U;
  uint32_t transaction = 0U;
  if (!hil_protocol_unpack_mode(frame, &mode, &transaction)) {
    taskENTER_CRITICAL();
    hil_safety_internal_error(&controller.safety);
    sync_safety_legacy();
    taskEXIT_CRITICAL();
    return;
  }
  const TickType_t now = xTaskGetTickCount();
  const bool fresh = controller.link_seen && controller.have_command && controller.have_feedback &&
    (now - controller.last_command_tick <= COMMAND_TIMEOUT_TICKS) &&
    (now - controller.last_feedback_tick <= FEEDBACK_TIMEOUT_TICKS);
  const bool accepted = hil_safety_request_mode(&controller.safety, mode, uptime_ms(), 100U, 50U);
  sync_safety_legacy();
  if (mode == HIL_MODE_DISARM) {
    taskENTER_CRITICAL();
    controller.state = HIL_STATE_DISARMED;
    controller.left_effort_nm = 0.0F;
    controller.right_effort_nm = 0.0F;
    taskEXIT_CRITICAL();
    sync_safety_legacy();
    queue_ack(HIL_MSG_MODE_COMMAND, transaction, HIL_ACK_OK);
  } else if (mode == HIL_MODE_ARM && fresh && accepted && controller.fault == 0U) {
    taskENTER_CRITICAL();
    controller.state = HIL_STATE_ACTIVE;
    taskEXIT_CRITICAL();
    sync_safety_legacy();
    queue_ack(HIL_MSG_MODE_COMMAND, transaction, HIL_ACK_OK);
  } else {
    taskENTER_CRITICAL();
    controller.state = HIL_STATE_SAFE;
    taskEXIT_CRITICAL();
    sync_safety_legacy();
    queue_ack(HIL_MSG_MODE_COMMAND, transaction, HIL_ACK_REJECTED);
  }
}

static void process_frame(const hil_protocol_frame_t *frame)
{
  if (frame->message_type == HIL_MSG_HELLO) {
    process_hello(frame);
    return;
  }
  if (frame->message_type == HIL_MSG_CONTROL_COMMAND) {
    float linear = 0.0F;
    float yaw = 0.0F;
    if (hil_protocol_unpack_control(frame, &linear, &yaw) && hil_control_finite(linear) && hil_control_finite(yaw)) {
      taskENTER_CRITICAL();
      controller.linear_m_s = linear;
      controller.yaw_rad_s = yaw;
      controller.last_command_tick = xTaskGetTickCount();
      controller.have_command = true;
      hil_safety_note_command(&controller.safety, uptime_ms());
      sync_safety_legacy();
      taskEXIT_CRITICAL();
    } else {
      taskENTER_CRITICAL();
      hil_safety_internal_error(&controller.safety);
      sync_safety_legacy();
      taskEXIT_CRITICAL();
    }
    return;
  }
  if (frame->message_type == HIL_MSG_WHEEL_FEEDBACK) {
    float left = 0.0F;
    float right = 0.0F;
    if (hil_protocol_unpack_wheel_pair(frame, &left, &right) &&
      hil_control_finite(left) && hil_control_finite(right)) {
      taskENTER_CRITICAL();
      controller.left_feedback_rad_s = left;
      controller.right_feedback_rad_s = right;
      controller.last_feedback_tick = xTaskGetTickCount();
      controller.have_feedback = true;
      hil_safety_note_feedback(&controller.safety, uptime_ms());
      sync_safety_legacy();
      taskEXIT_CRITICAL();
    } else {
      taskENTER_CRITICAL();
      hil_safety_internal_error(&controller.safety);
      sync_safety_legacy();
      taskEXIT_CRITICAL();
    }
    return;
  }
  if (frame->message_type == HIL_MSG_MODE_COMMAND) {
    process_mode(frame);
    return;
  }
  if (frame->message_type == HIL_MSG_PING) {
    uint32_t token = 0U;
    if (hil_protocol_unpack_ping(frame, &token)) {
      queue_pong(token);
    }
  }
}

void USART2_IRQHandler(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  while ((USART2->SR & USART_SR_RXNE) != 0U) {
    const uint8_t byte = (uint8_t)(USART2->DR & 0xffU);
    if (xStreamBufferSendFromISR(
        rx_stream, &byte, 1U, &higher_priority_task_woken) != 1U) {
      ++runtime_metrics.rx_stream_drops;
    }
  }
  if ((USART2->SR & USART_SR_ORE) != 0U) {
    (void)USART2->DR;
    ++runtime_metrics.uart_overruns;
  }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void uart_rx_task(void *argument)
{
  (void)argument;
  for (;;) {
    uint8_t byte = 0U;
    if (xStreamBufferReceive(rx_stream, &byte, 1U, portMAX_DELAY) == 1U) {
      hil_protocol_frame_t frame;
      const hil_protocol_result_t result = hil_protocol_decoder_feed(&decoder, byte, &frame);
      if (result == HIL_PROTOCOL_FRAME_READY) {
        process_frame(&frame);
      }
    }
  }
}

static void control_task(void *argument)
{
  (void)argument;
  TickType_t previous = xTaskGetTickCount();
  const hil_control_wheel_pair_t zero = {0.0F, 0.0F};
  for (;;) {
    vTaskDelayUntil(&previous, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    const uint32_t activation_cycles = timing_now_cycles();
    const uint32_t now_ms = uptime_ms();
    taskENTER_CRITICAL();
    hil_safety_evaluate(&controller.safety, now_ms, 100U, 50U);
    sync_safety_legacy();
    const bool fresh = hil_safety_is_fresh(&controller.safety, now_ms, 100U, 50U);
    const uint8_t state = controller.safety.state;
    const float linear = controller.linear_m_s;
    const float yaw = controller.yaw_rad_s;
    const float left_feedback = controller.left_feedback_rad_s;
    const float right_feedback = controller.right_feedback_rad_s;
    taskEXIT_CRITICAL();

    hil_control_wheel_pair_t effort = zero;
    if (state == HIL_STATE_ACTIVE && fresh) {
      const hil_control_wheel_pair_t targets = hil_control_body_to_wheels(linear, yaw, 0.18F, 0.86F);
      effort.left_rad_s = hil_control_limited_p_effort(targets.left_rad_s, left_feedback, 0.75F, 1.5F);
      effort.right_rad_s = hil_control_limited_p_effort(targets.right_rad_s, right_feedback, 0.75F, 1.5F);
    }
    taskENTER_CRITICAL();
    controller.left_effort_nm = effort.left_rad_s;
    controller.right_effort_nm = effort.right_rad_s;
    taskEXIT_CRITICAL();

    const uint32_t completion_cycles = timing_now_cycles();
    const uint32_t execution_us = timing_cycles_to_us(
      timing_cycle_delta(activation_cycles, completion_cycles));
    uint32_t period_us = 0U;
    if (runtime_metrics.have_previous_activation) {
      period_us = timing_cycles_to_us(
        timing_cycle_delta(runtime_metrics.previous_activation_cycles, activation_cycles));
    }
    runtime_metrics.previous_activation_cycles = activation_cycles;
    runtime_metrics.have_previous_activation = true;

    taskENTER_CRITICAL();
    ++runtime_metrics.sample_count;
    if (runtime_metrics.sample_count == 1U || execution_us < runtime_metrics.execution_min_us) {
      runtime_metrics.execution_min_us = execution_us;
    }
    if (runtime_metrics.sample_count == 1U || execution_us > runtime_metrics.execution_max_us) {
      runtime_metrics.execution_max_us = execution_us;
    }
    runtime_metrics.execution_sum_us += execution_us;
    if (period_us > 0U) {
      ++runtime_metrics.period_sample_count;
      if (runtime_metrics.period_sample_count == 1U || period_us < runtime_metrics.period_min_us) {
        runtime_metrics.period_min_us = period_us;
      }
      if (runtime_metrics.period_sample_count == 1U || period_us > runtime_metrics.period_max_us) {
        runtime_metrics.period_max_us = period_us;
      }
      runtime_metrics.period_sum_us += period_us;
    }
    if (execution_us >= CONTROL_PERIOD_US || period_us > (CONTROL_PERIOD_US + 1000U)) {
      ++runtime_metrics.deadline_misses;
    }
    ++runtime_metrics.control_progress;
    taskEXIT_CRITICAL();
  }
}

static void supervisor_task(void *argument)
{
  (void)argument;
  uint32_t last_progress = 0U;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100U));
    taskENTER_CRITICAL();
    const uint32_t progress = runtime_metrics.control_progress;
    taskEXIT_CRITICAL();
#if HIL_TEST_WATCHDOG
    if (uptime_ms() >= WATCHDOG_TEST_HOLD_MS) {
      continue;
    }
#endif
    if (progress != last_progress) {
      iwdg_refresh();
      last_progress = progress;
    }
  }
}

static void send_hello(void)
{
  hil_protocol_frame_t frame = {0};
  frame.message_type = HIL_MSG_HELLO;
  frame.payload_length = 10U;
  frame.payload[0] = HIL_ROLE_STM32;
  frame.payload[1] = HIL_PROTOCOL_VERSION;
  put_u32(&frame.payload[2], controller.boot_id);
  put_u32(&frame.payload[6], HIL_HELLO_CAP_TIMING_STATUS | HIL_HELLO_CAP_HARDWARE_WATCHDOG);
  uart_write_frame(frame);
}

static void send_heartbeat(void)
{
  hil_protocol_frame_t frame = {0};
  frame.message_type = HIL_MSG_HEARTBEAT;
  frame.payload_length = 5U;
  put_u32(&frame.payload[0], uptime_ms());
  frame.payload[4] = controller.state;
  uart_write_frame(frame);
}

static void send_status(void)
{
  hil_protocol_frame_t frame = {0};
  hil_protocol_counters_t counters;
  uint8_t state;
  uint8_t reason;
  uint8_t reset_cause;
  uint32_t boot_id;
  uint32_t rx_drops;
  uint32_t tx_drops;
  uint32_t uart_overruns;
  frame.message_type = HIL_MSG_STATUS;
  frame.payload_length = HIL_STATUS_PAYLOAD_SIZE;
  taskENTER_CRITICAL();
  state = controller.safety.state;
  reason = controller.safety.reason;
  reset_cause = controller.reset_cause;
  boot_id = controller.boot_id;
  counters = decoder.counters;
  rx_drops = runtime_metrics.rx_stream_drops;
  tx_drops = runtime_metrics.tx_queue_drops;
  uart_overruns = runtime_metrics.uart_overruns;
  taskEXIT_CRITICAL();
  frame.payload[0] = state;
  frame.payload[1] = reason;
  frame.payload[2] = reset_cause;
  frame.payload[3] = 0U;
  put_u32(&frame.payload[4], boot_id);
  put_u32(&frame.payload[8], uptime_ms());
  put_u32(&frame.payload[12], counters.valid_frames);
  put_u32(&frame.payload[16], counters.crc_failures);
  put_u32(&frame.payload[20], counters.decode_failures);
  put_u32(&frame.payload[24], counters.length_failures);
  put_u32(&frame.payload[28], counters.version_failures);
  put_u32(&frame.payload[32], counters.duplicate_frames);
  put_u32(&frame.payload[36], counters.stale_frames);
  put_u32(&frame.payload[40], counters.sequence_gaps);
  put_u32(&frame.payload[44], rx_drops);
  put_u32(&frame.payload[48], tx_drops);
  put_u32(&frame.payload[52], uart_overruns);
  uart_write_frame(frame);
}

static void send_timing_status(void)
{
  hil_timing_status_t status = {0};
  taskENTER_CRITICAL();
  status.state = controller.safety.state;
  status.safety_reason = controller.safety.reason;
  status.reset_cause = controller.reset_cause;
  status.boot_id = controller.boot_id;
  status.uptime_ms = uptime_ms();
  status.sample_count = runtime_metrics.sample_count;
  status.execution_min_us = runtime_metrics.execution_min_us;
  status.execution_mean_us = mean_u64(runtime_metrics.execution_sum_us, runtime_metrics.sample_count);
  status.execution_max_us = runtime_metrics.execution_max_us;
  status.period_min_us = runtime_metrics.period_min_us;
  status.period_mean_us = mean_u64(runtime_metrics.period_sum_us, runtime_metrics.period_sample_count);
  status.period_max_us = runtime_metrics.period_max_us;
  status.deadline_misses = runtime_metrics.deadline_misses;
  status.rx_stream_drops = runtime_metrics.rx_stream_drops;
  status.tx_queue_drops = runtime_metrics.tx_queue_drops;
  status.uart_overruns = runtime_metrics.uart_overruns;
  status.rx_stack_high_water_words = saturate_u16(uxTaskGetStackHighWaterMark(rx_task_handle));
  status.control_stack_high_water_words = saturate_u16(uxTaskGetStackHighWaterMark(control_task_handle));
  status.tx_stack_high_water_words = saturate_u16(uxTaskGetStackHighWaterMark(tx_task_handle));
  taskEXIT_CRITICAL();

  hil_protocol_frame_t frame = {0};
  if (hil_protocol_pack_timing_status(&frame, &status)) {
    uart_write_frame(frame);
  }
}

static void tx_task(void *argument)
{
  (void)argument;
  TickType_t previous = xTaskGetTickCount();
  uint32_t effort_divider = 0U;
  uint32_t heartbeat_divider = 0U;
  uint32_t status_divider = 0U;
  uint32_t timing_divider = 0U;
  send_hello();
  for (;;) {
    vTaskDelayUntil(&previous, pdMS_TO_TICKS(1U));
    hil_protocol_frame_t pending;
    while (xQueueReceive(tx_queue, &pending, 0U) == pdTRUE) {
      uart_write_frame(pending);
    }
    if (++effort_divider >= 10U) {
      effort_divider = 0U;
      hil_protocol_frame_t effort_frame = {0};
      float left = 0.0F;
      float right = 0.0F;
      taskENTER_CRITICAL();
      left = controller.left_effort_nm;
      right = controller.right_effort_nm;
      taskEXIT_CRITICAL();
      if (hil_protocol_pack_wheel_pair(&effort_frame, HIL_MSG_WHEEL_EFFORT, left, right)) {
        uart_write_frame(effort_frame);
      }
    }
    if (++heartbeat_divider >= 100U) {
      heartbeat_divider = 0U;
      send_heartbeat();
    }
    if (++status_divider >= 100U) {
      status_divider = 0U;
      send_status();
    }
    if (++timing_divider >= 1000U) {
      timing_divider = 0U;
      send_timing_status();
    }
    const bool active = controller.state == HIL_STATE_ACTIVE;
    const bool safe = controller.state == HIL_STATE_SAFE;
    if (active) {
      led_set(true);
    } else if (safe) {
      led_set((uptime_ms() / 250U) % 2U == 0U);
    } else {
      led_set(false);
    }
  }
}

void vApplicationGetIdleTaskMemory(
  StaticTask_t **task_buffer, StackType_t **stack_buffer, uint32_t *stack_size)
{
  *task_buffer = &idle_task_struct;
  *stack_buffer = idle_task_stack;
  *stack_size = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
  (void)task;
  (void)name;
  led_set(true);
  for (;;) {
  }
}

void vApplicationAssert(void)
{
  __asm volatile ("cpsid i");
  for (;;) {
  }
}

int main(void)
{
  timing_init();
  controller.reset_cause = capture_reset_cause();
  hil_safety_init(&controller.safety);
  sync_safety_legacy();
  controller.boot_id = ++boot_counter;
  if (controller.boot_id == 0U) {
    controller.boot_id = ++boot_counter;
  }
  hil_protocol_decoder_init(&decoder);
  rx_stream = xStreamBufferCreateStatic(RX_STREAM_CAPACITY, 1U, rx_stream_storage, &rx_stream_struct);
  tx_queue = xQueueCreateStatic(TX_QUEUE_LENGTH, sizeof(hil_protocol_frame_t), tx_queue_storage, &tx_queue_struct);
  configASSERT(rx_stream != NULL);
  configASSERT(tx_queue != NULL);
  gpio_uart_init();
  rx_task_handle = xTaskCreateStatic(
      uart_rx_task, "uart_rx", 384U, NULL, 3U, rx_task_stack, &rx_task_struct);
  configASSERT(rx_task_handle != NULL);
  control_task_handle = xTaskCreateStatic(
      control_task, "control", 384U, NULL, 4U, control_task_stack, &control_task_struct);
  configASSERT(control_task_handle != NULL);
  tx_task_handle = xTaskCreateStatic(
      tx_task, "tx", 384U, NULL, 2U, tx_task_stack, &tx_task_struct);
  configASSERT(tx_task_handle != NULL);
  supervisor_task_handle = xTaskCreateStatic(
      supervisor_task, "supervisor", 256U, NULL, 3U, supervisor_task_stack, &supervisor_task_struct);
  configASSERT(supervisor_task_handle != NULL);
  iwdg_start();
  vTaskStartScheduler();
  for (;;) {
  }
}
