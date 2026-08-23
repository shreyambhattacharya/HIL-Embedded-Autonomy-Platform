#include "hil_protocol.h"

#include <float.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24
#error "protocol requires IEEE-754 binary32 float"
#endif

static void write_u16(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)(value & 0xffU);
  out[1] = (uint8_t)((value >> 8U) & 0xffU);
}

static uint16_t read_u16(const uint8_t *in)
{
  return (uint16_t)in[0] | (uint16_t)((uint16_t)in[1] << 8U);
}

static void write_u32(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value & 0xffU);
  out[1] = (uint8_t)((value >> 8U) & 0xffU);
  out[2] = (uint8_t)((value >> 16U) & 0xffU);
  out[3] = (uint8_t)((value >> 24U) & 0xffU);
}

static uint32_t read_u32(const uint8_t *in)
{
  return (uint32_t)in[0] |
    ((uint32_t)in[1] << 8U) |
    ((uint32_t)in[2] << 16U) |
    ((uint32_t)in[3] << 24U);
}

static void write_float(uint8_t *out, float value)
{
  uint32_t bits = 0U;
  memcpy(&bits, &value, sizeof(bits));
  write_u32(out, bits);
}

static float read_float(const uint8_t *in)
{
  const uint32_t bits = read_u32(in);
  float value = 0.0F;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

uint16_t hil_protocol_crc16_ccitt_false(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xffffU;
  for (size_t i = 0U; i < length; ++i) {
    const uint16_t input_term = (uint16_t)((uint16_t)data[i] << 8U);
    crc = (uint16_t)(crc ^ input_term);
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      uint32_t shifted = (uint32_t)crc << 1U;
      if ((crc & 0x8000U) != 0U) {
        shifted ^= 0x1021U;
      }
      crc = (uint16_t)shifted;
    }
  }
  return crc;
}

size_t hil_protocol_cobs_encode(
  const uint8_t *input, size_t input_length, uint8_t *output, size_t output_capacity)
{
  if (input == NULL || output == NULL || input_length + 2U > output_capacity) {
    return 0U;
  }
  size_t read_index = 0U;
  size_t write_index = 1U;
  size_t code_index = 0U;
  uint8_t code = 1U;
  while (read_index < input_length) {
    if (input[read_index] == 0U) {
      output[code_index] = code;
      code = 1U;
      code_index = write_index++;
      ++read_index;
    } else {
      output[write_index++] = input[read_index++];
      ++code;
      if (code == 0xffU) {
        output[code_index] = code;
        code = 1U;
        code_index = write_index++;
      }
    }
  }
  output[code_index] = code;
  return write_index;
}

size_t hil_protocol_cobs_decode(
  const uint8_t *input, size_t input_length, uint8_t *output, size_t output_capacity)
{
  if (input == NULL || output == NULL || input_length == 0U) {
    return 0U;
  }
  size_t read_index = 0U;
  size_t write_index = 0U;
  while (read_index < input_length) {
    const uint8_t code = input[read_index++];
    if (code == 0U || (size_t)code > input_length - read_index + 1U) {
      return 0U;
    }
    const size_t copy_length = (size_t)code - 1U;
    if (copy_length > input_length - read_index || copy_length > output_capacity - write_index) {
      return 0U;
    }
    if (copy_length > 0U) {
      memcpy(&output[write_index], &input[read_index], copy_length);
      read_index += copy_length;
      write_index += copy_length;
    }
    if (code != 0xffU && read_index < input_length) {
      if (write_index >= output_capacity) {
        return 0U;
      }
      output[write_index++] = 0U;
    }
  }
  return write_index;
}

bool hil_protocol_encode_frame(
  const hil_protocol_frame_t *frame,
  uint8_t *output,
  size_t output_capacity,
  size_t *output_length)
{
  if (frame == NULL || output == NULL || output_length == NULL ||
    frame->payload_length > HIL_PROTOCOL_MAX_PAYLOAD || output_capacity < 2U) {
    return false;
  }
  uint8_t decoded[HIL_PROTOCOL_MAX_DECODED_FRAME];
  decoded[0] = frame->protocol_version;
  decoded[1] = frame->message_type;
  decoded[2] = frame->flags;
  decoded[3] = frame->payload_length;
  write_u32(&decoded[4], frame->sequence);
  write_u32(&decoded[8], frame->sender_tick_ms);
  if (frame->payload_length > 0U) {
    memcpy(&decoded[HIL_PROTOCOL_HEADER_SIZE], frame->payload, frame->payload_length);
  }
  const size_t crc_offset = HIL_PROTOCOL_HEADER_SIZE + frame->payload_length;
  write_u16(&decoded[crc_offset], hil_protocol_crc16_ccitt_false(decoded, crc_offset));
  const size_t encoded_length = hil_protocol_cobs_encode(
    decoded, crc_offset + HIL_PROTOCOL_CRC_SIZE, output, output_capacity - 1U);
  if (encoded_length == 0U || encoded_length + 1U > HIL_PROTOCOL_MAX_ENCODED_FRAME ||
    encoded_length + 1U > output_capacity) {
    return false;
  }
  output[encoded_length] = 0U;
  *output_length = encoded_length + 1U;
  return true;
}

void hil_protocol_decoder_init(hil_protocol_decoder_t *decoder)
{
  if (decoder != NULL) {
    memset(decoder, 0, sizeof(*decoder));
  }
}

static hil_protocol_result_t decode_frame(
  hil_protocol_decoder_t *decoder, hil_protocol_frame_t *frame)
{
  uint8_t decoded[HIL_PROTOCOL_MAX_DECODED_FRAME];
  const size_t decoded_length = hil_protocol_cobs_decode(
    decoder->encoded, decoder->encoded_length, decoded, sizeof(decoded));
  if (decoded_length == 0U) {
    ++decoder->counters.decode_failures;
    return HIL_PROTOCOL_DECODE_ERROR;
  }
  if (decoded_length < HIL_PROTOCOL_HEADER_SIZE + HIL_PROTOCOL_CRC_SIZE) {
    ++decoder->counters.length_failures;
    return HIL_PROTOCOL_LENGTH_ERROR;
  }
  const uint8_t payload_length = decoded[3];
  const size_t expected_length = HIL_PROTOCOL_HEADER_SIZE + (size_t)payload_length + HIL_PROTOCOL_CRC_SIZE;
  if (payload_length > HIL_PROTOCOL_MAX_PAYLOAD || decoded_length != expected_length) {
    ++decoder->counters.length_failures;
    return HIL_PROTOCOL_LENGTH_ERROR;
  }
  if (decoded[0] != HIL_PROTOCOL_VERSION) {
    ++decoder->counters.version_failures;
    return HIL_PROTOCOL_VERSION_ERROR;
  }
  const uint16_t expected_crc = hil_protocol_crc16_ccitt_false(decoded, expected_length - HIL_PROTOCOL_CRC_SIZE);
  if (read_u16(&decoded[expected_length - HIL_PROTOCOL_CRC_SIZE]) != expected_crc) {
    ++decoder->counters.crc_failures;
    return HIL_PROTOCOL_CRC_ERROR;
  }
  const uint32_t sequence = read_u32(&decoded[4]);
  // HELLO starts a new endpoint session and may legitimately restart its sequence.
  if (decoder->have_sequence && decoded[1] != HIL_MSG_HELLO) {
    const uint32_t delta = sequence - decoder->last_sequence;
    if (delta == 0U) {
      ++decoder->counters.duplicate_frames;
      return HIL_PROTOCOL_DUPLICATE;
    }
    if (delta >= 0x80000000U) {
      ++decoder->counters.stale_frames;
      return HIL_PROTOCOL_STALE;
    }
    if (delta > 1U) {
      decoder->counters.sequence_gaps += delta - 1U;
    }
  }
  frame->protocol_version = decoded[0];
  frame->message_type = decoded[1];
  frame->flags = decoded[2];
  frame->payload_length = payload_length;
  frame->sequence = sequence;
  frame->sender_tick_ms = read_u32(&decoded[8]);
  if (payload_length > 0U) {
    memcpy(frame->payload, &decoded[HIL_PROTOCOL_HEADER_SIZE], payload_length);
  }
  decoder->have_sequence = true;
  decoder->last_sequence = sequence;
  ++decoder->counters.valid_frames;
  return HIL_PROTOCOL_FRAME_READY;
}

hil_protocol_result_t hil_protocol_decoder_feed(
  hil_protocol_decoder_t *decoder, uint8_t byte, hil_protocol_frame_t *frame)
{
  if (decoder == NULL || frame == NULL) {
    return HIL_PROTOCOL_DECODE_ERROR;
  }
  if (byte != 0U) {
    if (decoder->encoded_length >= sizeof(decoder->encoded)) {
      decoder->encoded_length = 0U;
      ++decoder->counters.length_failures;
      return HIL_PROTOCOL_LENGTH_ERROR;
    }
    decoder->encoded[decoder->encoded_length++] = byte;
    return HIL_PROTOCOL_NO_FRAME;
  }
  if (decoder->encoded_length == 0U) {
    return HIL_PROTOCOL_NO_FRAME;
  }
  const hil_protocol_result_t result = decode_frame(decoder, frame);
  decoder->encoded_length = 0U;
  return result;
}

static bool set_pair(hil_protocol_frame_t *frame, uint8_t type, float left, float right)
{
  if (frame == NULL) {
    return false;
  }
  memset(frame, 0, sizeof(*frame));
  frame->protocol_version = HIL_PROTOCOL_VERSION;
  frame->message_type = type;
  frame->payload_length = 8U;
  write_float(&frame->payload[0], left);
  write_float(&frame->payload[4], right);
  return true;
}

bool hil_protocol_pack_control(hil_protocol_frame_t *frame, float linear_m_s, float yaw_rad_s)
{
  return set_pair(frame, HIL_MSG_CONTROL_COMMAND, linear_m_s, yaw_rad_s);
}

bool hil_protocol_unpack_control(
  const hil_protocol_frame_t *frame, float *linear_m_s, float *yaw_rad_s)
{
  if (frame == NULL || linear_m_s == NULL || yaw_rad_s == NULL ||
    frame->message_type != HIL_MSG_CONTROL_COMMAND || frame->payload_length != 8U) {
    return false;
  }
  *linear_m_s = read_float(&frame->payload[0]);
  *yaw_rad_s = read_float(&frame->payload[4]);
  return true;
}

bool hil_protocol_pack_wheel_pair(
  hil_protocol_frame_t *frame, uint8_t message_type, float left, float right)
{
  return (message_type == HIL_MSG_WHEEL_FEEDBACK || message_type == HIL_MSG_WHEEL_EFFORT) &&
    set_pair(frame, message_type, left, right);
}

bool hil_protocol_unpack_wheel_pair(
  const hil_protocol_frame_t *frame, float *left, float *right)
{
  if (frame == NULL || left == NULL || right == NULL ||
    (frame->message_type != HIL_MSG_WHEEL_FEEDBACK && frame->message_type != HIL_MSG_WHEEL_EFFORT) ||
    frame->payload_length != 8U) {
    return false;
  }
  *left = read_float(&frame->payload[0]);
  *right = read_float(&frame->payload[4]);
  return true;
}

bool hil_protocol_pack_mode(hil_protocol_frame_t *frame, uint8_t mode, uint32_t transaction)
{
  if (frame == NULL || (mode != HIL_MODE_ARM && mode != HIL_MODE_DISARM)) {
    return false;
  }
  memset(frame, 0, sizeof(*frame));
  frame->protocol_version = HIL_PROTOCOL_VERSION;
  frame->message_type = HIL_MSG_MODE_COMMAND;
  frame->payload_length = 5U;
  frame->payload[0] = mode;
  write_u32(&frame->payload[1], transaction);
  return true;
}

bool hil_protocol_unpack_mode(
  const hil_protocol_frame_t *frame, uint8_t *mode, uint32_t *transaction)
{
  if (frame == NULL || mode == NULL || transaction == NULL ||
    frame->message_type != HIL_MSG_MODE_COMMAND || frame->payload_length != 5U) {
    return false;
  }
  *mode = frame->payload[0];
  *transaction = read_u32(&frame->payload[1]);
  return *mode == HIL_MODE_ARM || *mode == HIL_MODE_DISARM;
}

bool hil_protocol_pack_ack(
  hil_protocol_frame_t *frame, uint8_t command_type, uint32_t transaction, uint8_t result)
{
  if (frame == NULL) {
    return false;
  }
  memset(frame, 0, sizeof(*frame));
  frame->protocol_version = HIL_PROTOCOL_VERSION;
  frame->message_type = HIL_MSG_ACK;
  frame->payload_length = 6U;
  frame->payload[0] = command_type;
  write_u32(&frame->payload[1], transaction);
  frame->payload[5] = result;
  return true;
}

bool hil_protocol_unpack_ack(
  const hil_protocol_frame_t *frame, uint8_t *command_type, uint32_t *transaction, uint8_t *result)
{
  if (frame == NULL || command_type == NULL || transaction == NULL || result == NULL ||
    frame->message_type != HIL_MSG_ACK || frame->payload_length != 6U) {
    return false;
  }
  *command_type = frame->payload[0];
  *transaction = read_u32(&frame->payload[1]);
  *result = frame->payload[5];
  return true;
}

bool hil_protocol_pack_ping(hil_protocol_frame_t *frame, uint32_t token)
{
  if (frame == NULL) {
    return false;
  }
  memset(frame, 0, sizeof(*frame));
  frame->protocol_version = HIL_PROTOCOL_VERSION;
  frame->message_type = HIL_MSG_PING;
  frame->payload_length = 4U;
  write_u32(frame->payload, token);
  return true;
}

bool hil_protocol_unpack_ping(const hil_protocol_frame_t *frame, uint32_t *token)
{
  if (frame == NULL || token == NULL ||
    (frame->message_type != HIL_MSG_PING && frame->message_type != HIL_MSG_PONG) ||
    frame->payload_length != 4U) {
    return false;
  }
  *token = read_u32(frame->payload);
  return true;
}
bool hil_protocol_pack_timing_status(
  hil_protocol_frame_t *frame, const hil_timing_status_t *status)
{
  if (frame == NULL || status == NULL) {
    return false;
  }
  memset(frame, 0, sizeof(*frame));
  frame->protocol_version = HIL_PROTOCOL_VERSION;
  frame->message_type = HIL_MSG_TIMING_STATUS;
  frame->payload_length = HIL_TIMING_STATUS_PAYLOAD_SIZE;
  frame->payload[0] = status->state;
  frame->payload[1] = status->safety_reason;
  frame->payload[2] = status->reset_cause;
  frame->payload[3] = status->reserved;
  write_u32(&frame->payload[4], status->boot_id);
  write_u32(&frame->payload[8], status->uptime_ms);
  write_u32(&frame->payload[12], status->sample_count);
  write_u32(&frame->payload[16], status->execution_min_us);
  write_u32(&frame->payload[20], status->execution_mean_us);
  write_u32(&frame->payload[24], status->execution_max_us);
  write_u32(&frame->payload[28], status->period_min_us);
  write_u32(&frame->payload[32], status->period_mean_us);
  write_u32(&frame->payload[36], status->period_max_us);
  write_u32(&frame->payload[40], status->deadline_misses);
  write_u32(&frame->payload[44], status->rx_stream_drops);
  write_u32(&frame->payload[48], status->tx_queue_drops);
  write_u32(&frame->payload[52], status->uart_overruns);
  write_u16(&frame->payload[56], status->rx_stack_high_water_words);
  write_u16(&frame->payload[58], status->control_stack_high_water_words);
  write_u16(&frame->payload[60], status->tx_stack_high_water_words);
  return true;
}

bool hil_protocol_unpack_timing_status(
  const hil_protocol_frame_t *frame, hil_timing_status_t *status)
{
  if (frame == NULL || status == NULL || frame->message_type != HIL_MSG_TIMING_STATUS ||
    frame->payload_length != HIL_TIMING_STATUS_PAYLOAD_SIZE) {
    return false;
  }
  status->state = frame->payload[0];
  status->safety_reason = frame->payload[1];
  status->reset_cause = frame->payload[2];
  status->reserved = frame->payload[3];
  status->boot_id = read_u32(&frame->payload[4]);
  status->uptime_ms = read_u32(&frame->payload[8]);
  status->sample_count = read_u32(&frame->payload[12]);
  status->execution_min_us = read_u32(&frame->payload[16]);
  status->execution_mean_us = read_u32(&frame->payload[20]);
  status->execution_max_us = read_u32(&frame->payload[24]);
  status->period_min_us = read_u32(&frame->payload[28]);
  status->period_mean_us = read_u32(&frame->payload[32]);
  status->period_max_us = read_u32(&frame->payload[36]);
  status->deadline_misses = read_u32(&frame->payload[40]);
  status->rx_stream_drops = read_u32(&frame->payload[44]);
  status->tx_queue_drops = read_u32(&frame->payload[48]);
  status->uart_overruns = read_u32(&frame->payload[52]);
  status->rx_stack_high_water_words = read_u16(&frame->payload[56]);
  status->control_stack_high_water_words = read_u16(&frame->payload[58]);
  status->tx_stack_high_water_words = read_u16(&frame->payload[60]);
  return true;
}
