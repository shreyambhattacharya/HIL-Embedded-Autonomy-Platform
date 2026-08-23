#include "hil_safety.h"

#include <stddef.h>

void hil_safety_init(hil_safety_state_t *state)
{
  if (state == NULL) {
    return;
  }
  *state = (hil_safety_state_t){
    .state = HIL_STATE_WAIT_LINK,
    .reason = HIL_SAFETY_NONE,
  };
}

void hil_safety_on_hello(hil_safety_state_t *state)
{
  if (state == NULL) {
    return;
  }
  state->link_seen = true;
  state->have_command = false;
  state->have_feedback = false;
  state->last_command_ms = 0U;
  state->last_feedback_ms = 0U;
  state->state = HIL_STATE_DISARMED;
  state->reason = HIL_SAFETY_NONE;
}

void hil_safety_note_command(hil_safety_state_t *state, uint32_t now_ms)
{
  if (state != NULL) {
    state->have_command = true;
    state->last_command_ms = now_ms;
  }
}

void hil_safety_note_feedback(hil_safety_state_t *state, uint32_t now_ms)
{
  if (state != NULL) {
    state->have_feedback = true;
    state->last_feedback_ms = now_ms;
  }
}

bool hil_safety_is_fresh(
  const hil_safety_state_t *state, uint32_t now_ms,
  uint32_t command_timeout_ms, uint32_t feedback_timeout_ms)
{
  if (state == NULL || !state->link_seen || !state->have_command || !state->have_feedback) {
    return false;
  }
  return (uint32_t)(now_ms - state->last_command_ms) <= command_timeout_ms &&
    (uint32_t)(now_ms - state->last_feedback_ms) <= feedback_timeout_ms;
}

bool hil_safety_request_mode(
  hil_safety_state_t *state, uint8_t mode, uint32_t now_ms,
  uint32_t command_timeout_ms, uint32_t feedback_timeout_ms)
{
  if (state == NULL) {
    return false;
  }
  if (mode == HIL_MODE_DISARM) {
    state->state = HIL_STATE_DISARMED;
    state->reason = HIL_SAFETY_MANUAL_DISARM;
    return true;
  }
  if (mode != HIL_MODE_ARM || !state->link_seen) {
    state->state = HIL_STATE_SAFE;
    state->reason = HIL_SAFETY_PROTOCOL_INCOMPATIBLE;
    return false;
  }
  if (!state->have_command ||
    (uint32_t)(now_ms - state->last_command_ms) > command_timeout_ms) {
    state->state = HIL_STATE_SAFE;
    state->reason = HIL_SAFETY_COMMAND_STALE;
    return false;
  }
  if (!state->have_feedback ||
    (uint32_t)(now_ms - state->last_feedback_ms) > feedback_timeout_ms) {
    state->state = HIL_STATE_SAFE;
    state->reason = HIL_SAFETY_FEEDBACK_STALE;
    return false;
  }
  state->state = HIL_STATE_ACTIVE;
  state->reason = HIL_SAFETY_NONE;
  return true;
}

void hil_safety_evaluate(
  hil_safety_state_t *state, uint32_t now_ms,
  uint32_t command_timeout_ms, uint32_t feedback_timeout_ms)
{
  if (state == NULL || state->state != HIL_STATE_ACTIVE) {
    return;
  }
  if (!state->have_command ||
    (uint32_t)(now_ms - state->last_command_ms) > command_timeout_ms) {
    state->state = HIL_STATE_SAFE;
    state->reason = HIL_SAFETY_COMMAND_STALE;
  } else if (!state->have_feedback ||
    (uint32_t)(now_ms - state->last_feedback_ms) > feedback_timeout_ms) {
    state->state = HIL_STATE_SAFE;
    state->reason = HIL_SAFETY_FEEDBACK_STALE;
  }
}

void hil_safety_internal_error(hil_safety_state_t *state)
{
  if (state != NULL) {
    state->state = HIL_STATE_FAULT;
    state->reason = HIL_SAFETY_INTERNAL_ERROR;
  }
}
