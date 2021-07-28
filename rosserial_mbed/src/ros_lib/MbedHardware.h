#pragma once

#include "mbed.h"

using namespace std::chrono;

// In case mbed_lib.json is not available
#ifndef ROSSERIAL_TX
#   define ROSSERIAL_TX USBTX
#endif

#ifndef ROSSERIAL_RX
#   define ROSSERIAL_RX USBRX
#endif

#ifndef ROSSERIAL_BAUDRATE
#   define ROSSERIAL_BAUDRATE 115200
#endif

#ifndef ROSSERIAL_INPUT_MSG_BUFFER_SIZE
#   define ROSSERIAL_INPUT_MSG_BUFFER_SIZE 512
#endif

#ifndef ROSSERIAL_OUTPUT_MSG_BUFFER_SIZE
#   define ROSSERIAL_OUTPUT_MSG_BUFFER_SIZE 512
#endif

#ifndef ROSSERIAL_MAX_SUBSCRIBERS
#   define ROSSERIAL_MAX_SUBSCRIBERS 25
#endif

#ifndef ROSSERIAL_MAX_PUBLISHERS
#   define ROSSERIAL_MAX_PUBLISHERS 25
#endif

class MbedHardware
{
public:
    MbedHardware(PinName tx, PinName rx, int baud)
        : _iostream(tx, rx, baud)
    {
        MBED_VERSION_CHECK(6,12,0);
    }

    MbedHardware()
        : MbedHardware(ROSSERIAL_TX, ROSSERIAL_RX, ROSSERIAL_BAUDRATE)
    {}

    void setBaud(int baud)
    {
        _iostream.baud(baud);
    }

    int read()
    {
        return -1;
        // if (iostream.readable())
        // {
        //     return iostream.getc();
        // }
        // else
        // {
        //     return -1;
        // }
    };

    void write(MBED_UNUSED uint8_t *data, MBED_UNUSED int length)
    {
        MBED_UNUSED
        // for (int i = 0; i < length; i++)
        //     iostream.putc(data[i]);
    }

    uint32_t time() { return 0}

protected:
    BufferedSerial _iostream;
};
