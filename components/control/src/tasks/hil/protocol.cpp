#include "protocol.hpp"
#include <string.h>
#include <arpa/inet.h>

bool protocol_encode_frame(const proto_msg_t *msg, uint8_t *out_buf, size_t out_buf_size, size_t *out_len)
{
    if (!msg || !out_buf || !out_len) {
        return false;
    }

    if (msg->len > PROTO_MAX_PAYLOAD_SIZE) {
        return false;
    }

    const size_t total = PROTO_HEADER_SIZE + msg->len;
    if (out_buf_size < total) {
        return false;
    }

    uint32_t magic_be = htonl(PROTO_MAGIC);
    uint16_t len_be   = htons(msg->len);
    uint16_t type_be  = htons(msg->type);

    memcpy(out_buf + 0, &magic_be, sizeof(magic_be));
    memcpy(out_buf + 4, &len_be,   sizeof(len_be));
    memcpy(out_buf + 6, &type_be,  sizeof(type_be));
    memcpy(out_buf + 8, msg->payload, msg->len);

    *out_len = total;
    return true;
}

bool protocol_decode_frame(const uint8_t *frame, size_t frame_len, proto_msg_t *out_msg)
{
    if (!frame || !out_msg) {
        return false;
    }

    if (frame_len < PROTO_HEADER_SIZE) {
        return false;
    }

    uint32_t magic_be;
    uint16_t len_be;
    uint16_t type_be;

    memcpy(&magic_be, frame + 0, sizeof(magic_be));
    memcpy(&len_be,   frame + 4, sizeof(len_be));
    memcpy(&type_be,  frame + 6, sizeof(type_be));

    const uint32_t magic = ntohl(magic_be);
    const uint16_t len   = ntohs(len_be);
    const uint16_t type  = ntohs(type_be);

    if (magic != PROTO_MAGIC) {
        return false;
    }

    if (len > PROTO_MAX_PAYLOAD_SIZE) {
        return false;
    }

    if (frame_len != (size_t)(PROTO_HEADER_SIZE + len)) {
        return false;
    }

    out_msg->type = type;
    out_msg->len  = len;
    memcpy(out_msg->payload, frame + PROTO_HEADER_SIZE, len);

    return true;
}