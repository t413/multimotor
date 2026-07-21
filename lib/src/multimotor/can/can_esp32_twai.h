#pragma once
#if defined(ARDUINO) && defined(ESP32)
#include "can_interface.h"
#include <Arduino.h>

class CanEsp32Twai : public CanInterface {
public:
    virtual void setup(uint8_t rx, uint8_t tx, int baudrate=1000000, Stream* debug = nullptr);
    virtual void send(uint32_t id, uint8_t* data, uint8_t len, CanFrame extended, CanSS ss = CanSS::Singleshot, CanReq rtr = CanReq::Command) override;
    virtual bool readOne(CanMessage&, uint32_t timeout_ms) override;
    virtual String getAlerts();
};

#endif
