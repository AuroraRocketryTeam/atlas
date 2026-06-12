#pragma once
#include "LogPayload.hpp"
#include <variant>

namespace PayloadSerializers {

    inline bool toJson(const LogPayload& payload, uint8_t* buffer, size_t bufferCapacity, size_t& currentOffset) {
        size_t remainingSpace = bufferCapacity - currentOffset;

        // Need at least 2 bytes (one character + null terminator) to even try
        if (remainingSpace <= 1) return false;

        // Attempt the write directly into the buffer
        size_t requiredBytes = std::visit([&](auto&& arg) -> size_t {
            return arg.serializeJson(reinterpret_cast<char*>(buffer + currentOffset), remainingSpace);
        }, payload);

        // RUNTIME CHECK: Did it fit?
        // snprintf returns the number of chars it *wanted* to write.
        // It requires (requiredBytes + 1) to fit the string AND the null terminator.
        if (requiredBytes >= remainingSpace) {
            // TRUNCATION OCCURRED: The buffer didn't have enough space.
            // We return false. The buffer was mutated, but because we don't 
            // advance currentOffset, this partial data effectively doesn't exist.
            return false;
        }

        // SUCCESS: It fit perfectly.
        // Advance the offset by requiredBytes. We intentionally exclude the 
        // null terminator (+1) from the offset because JSONL is newline separated.
        // The next log will overwrite the null terminator left by this log.
        currentOffset += requiredBytes;
        
        return true;
    }
}