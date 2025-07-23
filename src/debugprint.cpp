#include "debugprint.h"
#include <cstdarg>

DebugPrinter* DebugPrinter::instance_ = nullptr;

void DebugPrinter::setPrinter(DebugPrinter* printer) { instance_ = printer; }
DebugPrinter* DebugPrinter::getPrinter() { return instance_; }



// --------------------------------------- //
// -- platform specific implementations -- //
// --------------------------------------- //

#ifdef ARDUINO
#include <Arduino.h>

class DebugSerial : public DebugPrinter {
private:
    Stream* stream_;

public:
    DebugSerial(Stream* stream) : stream_(stream) { }
    bool availableForWrite() override {
        return stream_->availableForWrite();
    }
    void printf(const char* format, ...) override {
        if (availableForWrite()) {
            va_list args;
            va_start(args, format);
            stream_->printf(format, args);
            va_end(args);
        }
    }

    void println(const char* str) override { if (stream_) stream_->println(str); }
    void println(String str) override { if (stream_) stream_->println(str); }

};

void DebugPrinter::setPlatformSpecific(void* stream) {
    DebugPrinter::setPrinter(new DebugSerial((Stream*)stream));
}

#elif defined(DEBUG_USE_STDOUT)
#include <cstdio>

class DebugStdout : public DebugPrinter {
public:
    bool availableForWrite() override {
        return true;
    }
    void printf(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
    void println(const char* str) override {
        puts(str);
    }
    void println(std::string str) override {
        println(str.c_str());
    }
};

void DebugPrinter::setPlatformSpecific(void*) {
    DebugPrinter::setPrinter(new DebugStdout());
}

#else
#warning "No DebugPrinter implementation for this platform. Debugging will not work."

void DebugPrinter::setPlatformSpecific(void*) {
    //nothing!
}

#endif
