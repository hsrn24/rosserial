#pragma once

#include "mbed.h"

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

#ifndef ROSSERIAL_USE_RTOS_CLOCK
#   define ROSSERIAL_USE_RTOS_CLOCK 0
#endif

#ifndef ROSSERIAL_CHUNK_SIZE
#   define ROSSERIAL_CHUNK_SIZE 16
#endif

class MbedHardware
{
public:
    MbedHardware(PinName tx, PinName rx, int baud)
        : _iostream(tx, rx, baud)
    {
        MBED_VERSION_CHECK(6,13,0);
    }

    MbedHardware()
        : MbedHardware(ROSSERIAL_TX, ROSSERIAL_RX, ROSSERIAL_BAUDRATE)
    {}

    void init(){

#if ROSSERIAL_USE_RTOS_CLOCK == 0
        _t.start();
#endif
        _iostream.set_blocking(false);
    }

    void setBaud(int baud)
    {
        _iostream.set_baud(baud);
    }

    ssize_t read(uint8_t* data, size_t length)
    {
        return _iostream.read(data, length); 
    };

    ssize_t read(){
        return 0;
    }

    ssize_t write(uint8_t *data, size_t length)
    {
        _iostream.set_blocking(true);
        ssize_t res = _iostream.write(data, length);
        _iostream.set_blocking(false);
        return res;
    }

#if ROSSERIAL_USE_RTOS_CLOCK == 0
    Timer * getInternalTimer()
    {
        return &_t;
    }
#endif

    uint32_t time()
    {
#if ROSSERIAL_USE_RTOS_CLOCK
        return std::chrono::duration_cast<std::chrono::duration<uint32_t, std::milli>>(Kernel::Clock::now().time_since_epoch()).count();
#else
        return std::chrono::duration_cast<std::chrono::duration<uint32_t, std::milli>>(_t.elapsed_time()).count();
#endif
    }

protected:
    BufferedSerial _iostream;
#if ROSSERIAL_USE_RTOS_CLOCK == 0
    Timer _t;
#endif
};
