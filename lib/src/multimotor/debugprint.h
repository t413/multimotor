#pragma once
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__)
#include <string>
#define DEBUG_USE_STDOUT
#else
#warning "Unsupported platform for debug print"
#endif

class DebugPrinter {
public:
    virtual ~DebugPrinter() = default;
    virtual bool availableForWrite() = 0;
    virtual void printf(const char* format, ...) = 0;
    virtual void println(const char* str) = 0;
    virtual void printhex(const uint8_t* buf, size_t len, bool newline=true);
    #ifdef ARDUINO
    virtual void println(String) = 0; // For Arduino compatibility
    #elif defined(DEBUG_USE_STDOUT)
    virtual void println(std::string) = 0;
    #endif

    // Static interface for singleton access
    static void setPrinter(DebugPrinter* printer);
    static DebugPrinter* getPrinter();
    static void log(const char* format, ...);

    static void setPlatformSpecific(void*);

private:
    static DebugPrinter* instance_;
};

