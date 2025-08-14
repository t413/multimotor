#include <multimotor/can/can_interface.h>
#include <multimotor/serial/serial_interface.h>
#include <vector>
#include <cstring>
#include <string>

void printHex(const char* info, const uint8_t* data, size_t len);

// Mock CanInterface for testing
class MockCanInterface : public CanInterface {
public:
    struct SentFrame {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
    };
    std::vector<SentFrame> sentFrames;

    virtual void send(uint32_t id, uint8_t* data, uint8_t len, bool extended, bool ss = true, bool rtr = false) override {
        printf("MockCanInterface: Sending frame with ID: %08X, Length: %d ", id, len);
        printHex("data: ", data, len);
        SentFrame frame;
        frame.id = id;
        frame.len = len;
        memcpy(frame.data, data, len);
        sentFrames.push_back(frame);
    }
};

class MockSerial : public SerialInterface {
public:
    std::vector<std::string> sentFrames;
    std::string buffer;
    unsigned long timeout = 0;

    virtual int available() override { return static_cast<int>(buffer.size()); }

    virtual int read() override {
        if (buffer.empty()) return -1;
        char c = buffer.front();
        buffer.erase(0, 1);
        return static_cast<unsigned char>(c);
    }

    virtual int peek() override {
        if (buffer.empty()) return -1;
        return static_cast<unsigned char>(buffer.front());
    }

    virtual size_t write(uint8_t c) override { // Write one byte
        if (sentFrames.empty()) sentFrames.emplace_back();
        sentFrames.back().push_back(static_cast<char>(c));
        return 1;
    }

    virtual size_t write(const uint8_t *buf, size_t size) override { // Write multiple bytes
        printf("MockSerial: Writing %zu bytes: ", size);
        printHex("data: ", buf, size);
        printf("\n");
        fflush(stdout);
        sentFrames.emplace_back(reinterpret_cast<const char*>(buf), size);
        return size;
    }

    virtual void flush() override { }
    virtual void setTimeout(unsigned long t) override { timeout = t; }
    virtual unsigned long getTimeout(void) override { return timeout; }
};
