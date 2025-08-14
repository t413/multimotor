#pragma once
#include <stdint.h>
#ifdef ARDUINO
#include <Stream.h>
typedef Stream SerialInterface;

#else
#include <cstddef>

class SerialInterface {
public:
    virtual ~SerialInterface() {}

    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;

    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;

    virtual void setTimeout(unsigned long timeout) = 0;
    virtual unsigned long getTimeout(void) = 0;
};

#endif
