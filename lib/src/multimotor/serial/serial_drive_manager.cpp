#include "serial_drive_manager.h"
#include <cstring>  //memcpy, memmove
#include <algorithm> //std::min
#include "../motordrive.h"
#include "../debugprint.h"
#include "lx_servo.h"

void SerialDriveManager::beginSinglePin(SerialInterface* port, int txRxPin) {
    interface_ = port;
    uartSinglePin_ = txRxPin;
    if (!interface_)
        return;
#ifdef ARDUINO_ARCH_ESP32
    auto serial = (HardwareSerial*)interface_;
    serial->begin(115200, SERIAL_8N1, txRxPin, txRxPin);
    // pinMode(txRxPin, OUTPUT | PULLUP);
#endif
    delay(3);
    while (interface_->available())
        interface_->read();
}

void SerialDriveManager::beginDualPins(SerialInterface* port, int txPin, int rxPin) {
    interface_ = port;
    if (!interface_)
        return;
    uartSinglePin_ = -1;
    #ifdef ARDUINO
    auto serial = (HardwareSerial*)interface_;
    serial->begin(115200, SERIAL_8N1, txPin, rxPin);
    #endif
    // pinMode(txPin, OUTPUT | PULLUP);
    // pinMode(rxPin, INPUT | PULLDOWN);
}

void SerialDriveManager::addDrive(MotorDrive* drive) {
    if (driveCount_ < MAX_DRIVES) {
        drives_[driveCount_++] = drive;
    }
}

MotorDrive* SerialDriveManager::getDrive(uint8_t id) {
    for (uint8_t i = 0; i < driveCount_; ++i) {
        if (drives_[i] && drives_[i]->getId() == id) {
            return drives_[i];
        }
    }
    return nullptr;
}

uint8_t SerialDriveManager::getCount() const {
    return driveCount_;
}

bool SerialDriveManager::writeAndConsumeEcho(uint8_t const* data, uint8_t len, uint32_t timeout_us) {
    write(data, len); // existing write(), handles pin direction switch
    uint32_t start = micros();
    uint8_t consumed = 0;
    while (consumed < len) {
        if (interface_->available()) {
            interface_->read(); // discard echoed byte, it's just our own tx
            consumed++;
        } else if ((uint32_t)(micros() - start) > timeout_us) {
            DebugPrinter::log("SerialDriveManager: echo timeout %d/%d\n", consumed, len);
            return false; // wiring/bus fault — don't wait for a reply that'll never come
        }
    }
    return true;
}

bool SerialDriveManager::waitForReply(int16_t id, uint32_t timeout_us, uint8_t const** outData, uint8_t* outLen) {
    inBufferIndex_ = 0; // echo already consumed, start clean
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < timeout_us) {
        while (interface_->available() && inBufferIndex_ < INBUF_LEN)
            inBuffer_[inBufferIndex_++] = interface_->read();

        ParseResult result = LXServo::parsePacket(inBuffer_, inBufferIndex_);
        if (result.id < 0) continue;

        if (result.id == id) {
            *outData = inBuffer_ + result.start;
            *outLen  = result.len;
            return true;
        }
        // wrong id — drop it and keep waiting for the real reply
        memmove(inBuffer_, inBuffer_ + result.start + result.len,
                inBufferIndex_ - (result.start + result.len));
        inBufferIndex_ -= (result.start + result.len);
    }
    DebugPrinter::log("SerialDriveManager: no reply from id %d within %luus\n", id, timeout_us);
    return false;
}

bool SerialDriveManager::handleIncoming(uint32_t id, uint8_t const* indata, uint8_t inlen, uint32_t now) {
    #ifdef ARDUINO
    Serial.printf("SerialDriveManager: unexpected %d bytes: ", inlen);
    for (uint8_t i = 0; i < inlen; ++i)
        Serial.printf("%02X ", indata[i]);
    Serial.printf("\n");
    #endif
    return false;
}

uint8_t SerialDriveManager::iterate(uint32_t now, uint32_t timeout_ms) {
    uint8_t buf[32];
    int len = 0;
    while (interface_->available() && len < (int)sizeof(buf))
        buf[len++] = interface_->read();
    if (len > 0)
        handleIncoming(0, buf, len, now);
    return 0; // never "handled" a real command here
}

bool SerialDriveManager::readOnce(uint32_t now, uint32_t timeout_us) { return false; } //not used, send() blocks/consumes

void SerialDriveManager::write(uint8_t const* data, uint8_t len) {
    if (!interface_ || len == 0) return;
#if defined ARDUINO_ARCH_ESP32
    // if (uartSinglePin_ != GPIO_NUM_NC)
        // pinMode(uartSinglePin_, OUTPUT | PULLUP);
    delayMicroseconds(10);
#endif
    interface_->write(data, len);
}
