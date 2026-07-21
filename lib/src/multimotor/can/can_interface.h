#pragma once
#include <stdint.h>

enum class CanFrame : bool {
    Standard = false, //use standard 11-bit CAN IDs
    Extended = true, //use extended 29-bit CAN IDs
};

enum class CanSS : bool {
    Singleshot = true, //single shot mode. no retries, good for high-frequency motor commands
    Retry = false, //retries sending if no ACK received
};

enum class CanReq : bool {
    Command = false, //not expecting a reply
    RequestReply = true, //expecting a reply
};

struct CanMessage {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
};

class CanInterface {
public:
    virtual void send(uint32_t id, uint8_t* data, uint8_t len, CanFrame extended, CanSS ss = CanSS::Singleshot, CanReq rtr = CanReq::Command) = 0;
    virtual bool readOne(CanMessage&, uint32_t timeout_ms) = 0;
};

