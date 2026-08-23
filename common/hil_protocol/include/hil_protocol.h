#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HIL_PROTOCOL_VERSION 1U
#define HIL_PROTOCOL_MAX_PAYLOAD 64U
#define HIL_PROTOCOL_HEADER_SIZE 12U
#define HIL_PROTOCOL_CRC_SIZE 2U
#define HIL_PROTOCOL_MAX_DECODED_FRAME \
  (HIL_PROTOCOL_HEADER_SIZE + HIL_PROTOCOL_MAX_PAYLOAD + HIL_PROTOCOL_CRC_SIZE)
#define HIL_PROTOCOL_MAX_ENCODED_FRAME 96U

#ifdef __cplusplus
#define HIL_PROTOCOL_STATIC_ASSERT static_assert
#else
#define HIL_PROTOCOL_STATIC_ASSERT _Static_assert
#endif

HIL_PROTOCOL_STATIC_ASSERT(HIL_PROTOCOL_MAX_DECODED_FRAME == 78U, "protocol size changed");
HIL_PROTOCOL_STATIC_ASSERT(HIL_PROTOCOL_MAX_ENCODED_FRAME >= 80U, "encoded frame bound too small");
HIL_PROTOCOL_STATIC_ASSERT(sizeof(float) == 4U, "protocol requires 32-bit float");

typedef enum {
  HIL_MSG_HELLO = 1,
  HIL_MSG_HEARTBEAT = 2,
  HIL_MSG_CONTROL_COMMAND = 3,
  HIL_MSG_WHEEL_FEEDBACK = 4,
  HIL_MSG_WHEEL_EFFORT = 5,
  HIL_MSG_STATUS = 6,
  HIL_MSG_MODE_COMMAND = 7,
  HIL_MSG_ACK = 8,
  HIL_MSG_PING = 9,
  HIL_MSG_PONG = 10
} hil_message_type_t;

typedef enum {
  HIL_ROLE_LINUX_BRIDGE = 1,
  HIL_ROLE_STM32 = 2
} hil_endpoint_role_t;

typedef enum {
  HIL_MODE_ARM = 1,
  HIL_MODE_DISARM = 2
} hil_mode_t;

typedef enum {
  HIL_ACK_OK = 0,
  HIL_ACK_REJECTED = 1,
  HIL_ACK_INVALID = 2
} hil_ack_result_t;

typedef enum {
  HIL_STATE_WAIT_LINK = 0,
  HIL_STATE_DISARMED = 1,
  HIL_STATE_ACTIVE = 2,
  HIL_STATE_SAFE = 3,
  HIL_STATE_FAULT = 4
} hil_controller_state_t;

typedef struct {
  uint8_t protocol_version;
  uint8_t message_type;
  uint8_t flags;
  uint8_t payload_length;
  uint32_t sequence;
  uint32_t sender_tick_ms;
  uint8_t payload[HIL_PROTOCOL_MAX_PAYLOAD];
} hil_protocol_frame_t;

typedef struct {
  uint32_t valid_frames;
  uint32_t crc_failures;
  uint32_t decode_failures;
  uint32_t length_failures;
  uint32_t version_failures;
  uint32_t duplicate_frames;
  uint32_t stale_frames;
  uint32_t sequence_gaps;
} hil_protocol_counters_t;

typedef struct {
  uint8_t encoded[HIL_PROTOCOL_MAX_ENCODED_FRAME];
  size_t encoded_length;
  bool have_sequence;
  uint32_t last_sequence;
  hil_protocol_counters_t counters;
} hil_protocol_decoder_t;

typedef enum {
  HIL_PROTOCOL_NO_FRAME = 0,
  HIL_PROTOCOL_FRAME_READY = 1,
  HIL_PROTOCOL_DUPLICATE = 2,
  HIL_PROTOCOL_STALE = 3,
  HIL_PROTOCOL_DECODE_ERROR = 4,
  HIL_PROTOCOL_LENGTH_ERROR = 5,
  HIL_PROTOCOL_CRC_ERROR = 6,
  HIL_PROTOCOL_VERSION_ERROR = 7
} hil_protocol_result_t;

uint16_t hil_protocol_crc16_ccitt_false(const uint8_t *data, size_t length);
size_t hil_protocol_cobs_encode(
  const uint8_t *input, size_t input_length, uint8_t *output, size_t output_capacity);
size_t hil_protocol_cobs_decode(
  const uint8_t *input, size_t input_length, uint8_t *output, size_t output_capacity);

bool hil_protocol_encode_frame(
  const hil_protocol_frame_t *frame,
  uint8_t *output,
  size_t output_capacity,
  size_t *output_length);
void hil_protocol_decoder_init(hil_protocol_decoder_t *decoder);
hil_protocol_result_t hil_protocol_decoder_feed(
  hil_protocol_decoder_t *decoder,
  uint8_t byte,
  hil_protocol_frame_t *frame);

bool hil_protocol_pack_control(
  hil_protocol_frame_t *frame, float linear_m_s, float yaw_rad_s);
bool hil_protocol_unpack_control(
  const hil_protocol_frame_t *frame, float *linear_m_s, float *yaw_rad_s);
bool hil_protocol_pack_wheel_pair(
  hil_protocol_frame_t *frame, uint8_t message_type, float left, float right);
bool hil_protocol_unpack_wheel_pair(
  const hil_protocol_frame_t *frame, float *left, float *right);
bool hil_protocol_pack_mode(
  hil_protocol_frame_t *frame, uint8_t mode, uint32_t transaction);
bool hil_protocol_unpack_mode(
  const hil_protocol_frame_t *frame, uint8_t *mode, uint32_t *transaction);
bool hil_protocol_pack_ack(
  hil_protocol_frame_t *frame, uint8_t command_type, uint32_t transaction, uint8_t result);
bool hil_protocol_unpack_ack(
  const hil_protocol_frame_t *frame, uint8_t *command_type, uint32_t *transaction, uint8_t *result);
bool hil_protocol_pack_ping(hil_protocol_frame_t *frame, uint32_t token);
bool hil_protocol_unpack_ping(const hil_protocol_frame_t *frame, uint32_t *token);

#ifdef __cplusplus
}
#endif

#endif
