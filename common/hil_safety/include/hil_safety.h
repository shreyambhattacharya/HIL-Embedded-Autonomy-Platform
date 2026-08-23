#ifndef HIL_SAFETY_H
#define HIL_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "hil_protocol.h"

typedef struct {
  uint8_t state;
  uint8_t reason;
  bool link_seen;
  bool have_command;
  bool have_feedback;
  uint32_t last_command_ms;
  uint32_t last_feedback_ms;
} hil_safety_state_t;

void hil_safety_init(hil_safety_state_t *state);
void hil_safety_on_hello(hil_safety_state_t *state);
void hil_safety_note_command(hil_safety_state_t *state, uint32_t now_ms);
void hil_safety_note_feedback(hil_safety_state_t *state, uint32_t now_ms);
bool hil_safety_is_fresh(
  const hil_safety_state_t *state, uint32_t now_ms,
  uint32_t command_timeout_ms, uint32_t feedback_timeout_ms);
bool hil_safety_request_mode(
  hil_safety_state_t *state, uint8_t mode, uint32_t now_ms,
  uint32_t command_timeout_ms, uint32_t feedback_timeout_ms);
void hil_safety_evaluate(
  hil_safety_state_t *state, uint32_t now_ms,
  uint32_t command_timeout_ms, uint32_t feedback_timeout_ms);
void hil_safety_internal_error(hil_safety_state_t *state);

#endif
