#include "stm32f446xx.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hil_control.h"
#include "hil_protocol.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
#include "task.h"

#define RX_STREAM_CAPACITY 512U
#define TX_QUEUE_LENGTH 8U
#define UART_BAUD_RATE 115200U
#define COMMAND_TIMEOUT_TICKS pdMS_TO_TICKS(100U)
#define FEEDBACK_TIMEOUT_TICKS pdMS_TO_TICKS(50U)

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
} controller_context_t;

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
    controller.fault = 4U;
    return;
  }
  for (size_t i = 0U; i < encoded_length; ++i) {
    uart_write_byte(encoded[i]);
  }
}

static void queue_frame(const hil_protocol_frame_t *frame)
{
  (void)xQueueSend(tx_queue, frame, 0U);
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
    controller.fault = 5U;
    return;
  }
  taskENTER_CRITICAL();
  controller.link_seen = true;
  if (controller.state == HIL_STATE_WAIT_LINK) {
    controller.state = HIL_STATE_DISARMED;
  }
  taskEXIT_CRITICAL();
}

static void process_mode(const hil_protocol_frame_t *frame)
{
  uint8_t mode = 0U;
  uint32_t transaction = 0U;
  if (!hil_protocol_unpack_mode(frame, &mode, &transaction)) {
    controller.fault = 6U;
    return;
  }
  const TickType_t now = xTaskGetTickCount();
  const bool fresh = controller.link_seen && controller.have_command && controller.have_feedback &&
    (now - controller.last_command_tick <= COMMAND_TIMEOUT_TICKS) &&
    (now - controller.last_feedback_tick <= FEEDBACK_TIMEOUT_TICKS);
  if (mode == HIL_MODE_DISARM) {
    taskENTER_CRITICAL();
    controller.state = HIL_STATE_DISARMED;
    controller.left_effort_nm = 0.0F;
    controller.right_effort_nm = 0.0F;
    taskEXIT_CRITICAL();
    queue_ack(HIL_MSG_MODE_COMMAND, transaction, HIL_ACK_OK);
  } else if (mode == HIL_MODE_ARM && fresh && controller.fault == 0U) {
    taskENTER_CRITICAL();
    controller.state = HIL_STATE_ACTIVE;
    taskEXIT_CRITICAL();
    queue_ack(HIL_MSG_MODE_COMMAND, transaction, HIL_ACK_OK);
  } else {
    taskENTER_CRITICAL();
    controller.state = HIL_STATE_SAFE;
    taskEXIT_CRITICAL();
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
      taskEXIT_CRITICAL();
    } else {
      controller.fault = 7U;
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
      taskEXIT_CRITICAL();
    } else {
      controller.fault = 8U;
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
    (void)xStreamBufferSendFromISR(
      rx_stream, &byte, 1U, &higher_priority_task_woken);
  }
  if ((USART2->SR & USART_SR_ORE) != 0U) {
    (void)USART2->DR;
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
    vTaskDelayUntil(&previous, pdMS_TO_TICKS(10U));
    const TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL();
    const bool fresh = controller.have_command && controller.have_feedback &&
      (now - controller.last_command_tick <= COMMAND_TIMEOUT_TICKS) &&
      (now - controller.last_feedback_tick <= FEEDBACK_TIMEOUT_TICKS);
    const uint8_t state = controller.state;
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
    } else if (state == HIL_STATE_ACTIVE && !fresh) {
      taskENTER_CRITICAL();
      controller.state = HIL_STATE_SAFE;
      controller.fault = controller.have_command ? 2U : 1U;
      taskEXIT_CRITICAL();
    }
    taskENTER_CRITICAL();
    controller.left_effort_nm = effort.left_rad_s;
    controller.right_effort_nm = effort.right_rad_s;
    taskEXIT_CRITICAL();
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
  put_u32(&frame.payload[6], 1U);
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
  frame.message_type = HIL_MSG_STATUS;
  frame.payload_length = 26U;
  taskENTER_CRITICAL();
  frame.payload[0] = controller.state;
  frame.payload[1] = controller.fault;
  put_u32(&frame.payload[2], controller.boot_id);
  taskEXIT_CRITICAL();
  put_u32(&frame.payload[6], uptime_ms());
  put_u32(&frame.payload[10], decoder.counters.valid_frames);
  put_u32(&frame.payload[14], decoder.counters.crc_failures);
  put_u32(&frame.payload[18], decoder.counters.decode_failures);
  put_u32(&frame.payload[22], decoder.counters.sequence_gaps);
  uart_write_frame(frame);
}

static void tx_task(void *argument)
{
  (void)argument;
  TickType_t previous = xTaskGetTickCount();
  uint32_t effort_divider = 0U;
  uint32_t heartbeat_divider = 0U;
  uint32_t status_divider = 0U;
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
  configASSERT(xTaskCreateStatic(
      uart_rx_task, "uart_rx", 384U, NULL, 3U, rx_task_stack, &rx_task_struct) != NULL);
  configASSERT(xTaskCreateStatic(
      control_task, "control", 384U, NULL, 4U, control_task_stack, &control_task_struct) != NULL);
  configASSERT(xTaskCreateStatic(
      tx_task, "tx", 384U, NULL, 2U, tx_task_stack, &tx_task_struct) != NULL);
  vTaskStartScheduler();
  for (;;) {
  }
}
