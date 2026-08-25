#include "hil_safety.h"

#include <assert.h>
#include <stdint.h>

static void fresh_state(hil_safety_state_t *state)
{
  hil_safety_init(state);
  hil_safety_on_hello(state);
  hil_safety_note_command(state, 100U);
  hil_safety_note_feedback(state, 100U);
}

int main(void)
{
  hil_safety_state_t state;
  hil_safety_init(&state);
  assert(state.state == HIL_STATE_WAIT_LINK);
  assert(!hil_safety_request_mode(&state, HIL_MODE_ARM, 0U, 100U, 50U));
  assert(state.state == HIL_STATE_SAFE);
  assert(state.reason == HIL_SAFETY_PROTOCOL_INCOMPATIBLE);

  hil_safety_init(&state);
  hil_safety_on_hello(&state);
  assert(state.state == HIL_STATE_DISARMED);
  assert(!hil_safety_request_mode(&state, HIL_MODE_ARM, 0U, 100U, 50U));
  assert(state.reason == HIL_SAFETY_COMMAND_STALE);

  fresh_state(&state);
  assert(hil_safety_is_fresh(&state, 150U, 100U, 50U));
  assert(hil_safety_request_mode(&state, HIL_MODE_ARM, 150U, 100U, 50U));
  assert(state.state == HIL_STATE_ACTIVE);
  assert(state.reason == HIL_SAFETY_NONE);

  hil_safety_evaluate(&state, 151U, 100U, 50U);
  assert(state.state == HIL_STATE_SAFE);
  assert(state.reason == HIL_SAFETY_FEEDBACK_STALE);
  hil_safety_evaluate(&state, 400U, 100U, 50U);
  assert(state.state == HIL_STATE_SAFE);

  hil_safety_note_command(&state, 400U);
  hil_safety_note_feedback(&state, 400U);
  assert(hil_safety_request_mode(&state, HIL_MODE_ARM, 400U, 100U, 50U));
  assert(state.state == HIL_STATE_ACTIVE);
  assert(hil_safety_request_mode(&state, HIL_MODE_DISARM, 401U, 100U, 50U));
  assert(state.state == HIL_STATE_DISARMED);
  assert(state.reason == HIL_SAFETY_MANUAL_DISARM);

  fresh_state(&state);
  state.last_command_ms = UINT32_MAX - 20U;
  state.last_feedback_ms = UINT32_MAX - 20U;
  assert(hil_safety_is_fresh(&state, 10U, 40U, 50U));
  hil_safety_internal_error(&state);
  assert(state.state == HIL_STATE_FAULT);
  assert(state.reason == HIL_SAFETY_INTERNAL_ERROR);
  return 0;
}
