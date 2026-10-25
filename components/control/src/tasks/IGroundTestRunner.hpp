#pragma once

#include <cstddef>
#include <cstdint>

enum class GroundTestVerdict {
    PASSED,
    FAILED,
    RETRY,
    EXIT,
    REBOOT,
};

struct GroundTestDescriptor {
    int id;
    const char* group;
    const char* name;
    const char* description;
    bool destructive;
    const char* confirmation;
};

struct GroundTestStatus {
    bool running = false;
    bool waiting_for_verdict = false;
    bool last_passed = false;
    int active_id = 0;
    uint32_t started_ms = 0;
    uint32_t finished_ms = 0;
    char state[24] = "idle";
    char active_name[64] = "";
    char prompt[192] = "";
    char message[192] = "idle";
};

class IGroundTestRunner {
public:
    virtual ~IGroundTestRunner() = default;

    virtual size_t getGroundTestCount() const = 0;
    virtual const GroundTestDescriptor* getGroundTest(size_t index) const = 0;
    virtual bool startGroundTest(int id, char* error, size_t error_size) = 0;
    virtual bool submitGroundTestVerdict(GroundTestVerdict verdict, char* error, size_t error_size) = 0;
    virtual void getGroundTestStatus(GroundTestStatus* out) const = 0;
};
