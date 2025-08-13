#include <multimotor/can/can_interface.h>
#include <vector>
#include <cstring>

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
        printf("MockCanInterface: Sending frame with ID: %08X, Length: %d, Data: ", id, len);
        for (uint8_t i = 0; i < len; ++i) {
            printf("%02X ", data[i]);
        }
        printf("\n");
        fflush(stdout);
        SentFrame frame;
        frame.id = id;
        frame.len = len;
        memcpy(frame.data, data, len);
        sentFrames.push_back(frame);
    }
};
