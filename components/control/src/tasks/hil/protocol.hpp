#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_MAGIC               0xA5A55A5AUL
#define PROTO_HEADER_SIZE         8
#define PROTO_MAX_PAYLOAD_SIZE    256

typedef enum {
    MSG_TYPE_SIM_INPUT  = 1,
    MSG_TYPE_FC_COMMAND = 2,
    MSG_TYPE_SIM_RESET  = 3,
} proto_msg_type_t;

typedef struct {
    uint16_t type;
    uint16_t len;
    uint8_t  payload[PROTO_MAX_PAYLOAD_SIZE];
} proto_msg_t;

bool protocol_encode_frame(const proto_msg_t *msg, uint8_t *out_buf, size_t out_buf_size, size_t *out_len);
bool protocol_decode_frame(const uint8_t *frame, size_t frame_len, proto_msg_t *out_msg);

#ifdef __cplusplus
}
#endif