#include "hil_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_crc(void)
{
  static const uint8_t text[] = "123456789";
  assert(hil_protocol_crc16_ccitt_false(text, 9U) == 0x29b1U);
}

static void test_cobs(void)
{
  const uint8_t input[] = {0x11U, 0x22U, 0x00U, 0x33U};
  const uint8_t expected[] = {0x03U, 0x11U, 0x22U, 0x02U, 0x33U};
  uint8_t encoded[16];
  uint8_t decoded[16];
  const size_t encoded_length = hil_protocol_cobs_encode(input, sizeof(input), encoded, sizeof(encoded));
  assert(encoded_length == sizeof(expected));
  assert(memcmp(encoded, expected, sizeof(expected)) == 0);
  assert(hil_protocol_cobs_decode(encoded, encoded_length, decoded, sizeof(decoded)) == sizeof(input));
  assert(memcmp(decoded, input, sizeof(input)) == 0);
}

static void test_round_trip(void)
{
  hil_protocol_frame_t source;
  memset(&source, 0, sizeof(source));
  source.protocol_version = HIL_PROTOCOL_VERSION;
  source.message_type = HIL_MSG_STATUS;
  source.flags = 0x5aU;
  source.sequence = 0x12345678U;
  source.sender_tick_ms = 0xabcdef01U;
  source.payload_length = HIL_PROTOCOL_MAX_PAYLOAD;
  for (size_t i = 0U; i < source.payload_length; ++i) {
    source.payload[i] = (uint8_t)(i * 3U);
  }
  uint8_t encoded[HIL_PROTOCOL_MAX_ENCODED_FRAME];
  size_t encoded_length = 0U;
  assert(hil_protocol_encode_frame(&source, encoded, sizeof(encoded), &encoded_length));
  assert(encoded[encoded_length - 1U] == 0U);
  hil_protocol_decoder_t decoder;
  hil_protocol_decoder_init(&decoder);
  hil_protocol_frame_t decoded;
  hil_protocol_result_t result = HIL_PROTOCOL_NO_FRAME;
  for (size_t i = 0U; i < encoded_length; ++i) {
    result = hil_protocol_decoder_feed(&decoder, encoded[i], &decoded);
  }
  assert(result == HIL_PROTOCOL_FRAME_READY);
  assert(memcmp(&source, &decoded, sizeof(source)) == 0);
}

static void test_message_helpers(void)
{
  hil_protocol_frame_t frame;
  float a = 0.0F;
  float b = 0.0F;
  assert(hil_protocol_pack_control(&frame, 0.25F, -0.5F));
  assert(hil_protocol_unpack_control(&frame, &a, &b));
  assert(a == 0.25F && b == -0.5F);
  assert(hil_protocol_pack_mode(&frame, HIL_MODE_ARM, 77U));
  uint8_t mode = 0U;
  uint32_t transaction = 0U;
  assert(hil_protocol_unpack_mode(&frame, &mode, &transaction));
  assert(mode == HIL_MODE_ARM && transaction == 77U);
  assert(hil_protocol_pack_ack(&frame, HIL_MSG_MODE_COMMAND, 77U, HIL_ACK_OK));
  uint8_t command = 0U;
  uint8_t result = 0xffU;
  assert(hil_protocol_unpack_ack(&frame, &command, &transaction, &result));
  assert(command == HIL_MSG_MODE_COMMAND && transaction == 77U && result == HIL_ACK_OK);
}

static void test_malformed_and_sequences(void)
{
  hil_protocol_frame_t source;
  assert(hil_protocol_pack_ping(&source, 9U));
  uint8_t encoded[HIL_PROTOCOL_MAX_ENCODED_FRAME];
  size_t length = 0U;
  hil_protocol_decoder_t decoder;
  hil_protocol_decoder_init(&decoder);
  hil_protocol_frame_t output;
  source.sequence = 10U;
  assert(hil_protocol_encode_frame(&source, encoded, sizeof(encoded), &length));
  for (size_t i = 0U; i < length; ++i) {
    (void)hil_protocol_decoder_feed(&decoder, encoded[i], &output);
  }
  source.sequence = 10U;
  assert(hil_protocol_encode_frame(&source, encoded, sizeof(encoded), &length));
  hil_protocol_result_t result = HIL_PROTOCOL_NO_FRAME;
  for (size_t i = 0U; i < length; ++i) {
    result = hil_protocol_decoder_feed(&decoder, encoded[i], &output);
  }
  assert(result == HIL_PROTOCOL_DUPLICATE);
  source.sequence = 12U;
  assert(hil_protocol_encode_frame(&source, encoded, sizeof(encoded), &length));
  for (size_t i = 0U; i < length; ++i) {
    (void)hil_protocol_decoder_feed(&decoder, encoded[i], &output);
  }
  assert(decoder.counters.sequence_gaps == 1U);
  source.sequence = 11U;
  assert(hil_protocol_encode_frame(&source, encoded, sizeof(encoded), &length));
  for (size_t i = 0U; i < length; ++i) {
    result = hil_protocol_decoder_feed(&decoder, encoded[i], &output);
  }
  assert(result == HIL_PROTOCOL_STALE);
  assert(decoder.counters.stale_frames == 1U);

  source.message_type = HIL_MSG_HELLO;
  source.payload_length = 0U;
  source.sequence = 0U;
  assert(hil_protocol_encode_frame(&source, encoded, sizeof(encoded), &length));
  for (size_t i = 0U; i < length; ++i) {
    result = hil_protocol_decoder_feed(&decoder, encoded[i], &output);
  }
  assert(result == HIL_PROTOCOL_FRAME_READY);
  assert(output.message_type == HIL_MSG_HELLO && output.sequence == 0U);
  encoded[length - 2U] ^= 0x01U;
  for (size_t i = 0U; i < length; ++i) {
    result = hil_protocol_decoder_feed(&decoder, encoded[i], &output);
  }
  assert(result == HIL_PROTOCOL_CRC_ERROR || result == HIL_PROTOCOL_DECODE_ERROR);
}

static void test_random_bytes(void)
{
  hil_protocol_decoder_t decoder;
  hil_protocol_decoder_init(&decoder);
  hil_protocol_frame_t frame;
  uint32_t value = 0x12345678U;
  for (size_t i = 0U; i < 10000U; ++i) {
    value = value * 1664525U + 1013904223U;
    (void)hil_protocol_decoder_feed(&decoder, (uint8_t)(value >> 24U), &frame);
  }
}

int main(void)
{
  test_crc();
  test_cobs();
  test_round_trip();
  test_message_helpers();
  test_malformed_and_sequences();
  test_random_bytes();
  puts("hil_protocol tests passed");
  return 0;
}
