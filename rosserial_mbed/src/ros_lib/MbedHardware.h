/** @file MbedHardware.h
 * Serial hardware implementation for mbed.
 */
#ifndef ROS_MBED_HARDWARE_H_
#define ROS_MBED_HARDWARE_H_

#include "platform/PlatformMutex.h"
#include "platform/CircularBuffer.h"
#include "platform/NonCopyable.h"
#include "drivers/SerialBase.h"
#include "drivers/Timer.h"

// In case mbed_lib.json is not available
#ifndef ROSSERIAL_TX
#define ROSSERIAL_TX USBTX
#endif

#ifndef ROSSERIAL_RX
#define ROSSERIAL_RX USBRX
#endif

#ifndef ROSSERIAL_BAUDRATE
#define ROSSERIAL_BAUDRATE 115200
#endif

#ifndef ROSSERIAL_INPUT_MSG_BUFFER_SIZE
#define ROSSERIAL_INPUT_MSG_BUFFER_SIZE 512
#endif

#ifndef ROSSERIAL_OUTPUT_MSG_BUFFER_SIZE
#define ROSSERIAL_OUTPUT_MSG_BUFFER_SIZE 512
#endif

#ifndef ROSSERIAL_CIRCULAR_BUFFER_SIZE
#define ROSSERIAL_CIRCULAR_BUFFER_SIZE 1024
#endif

#ifndef ROSSERIAL_MAX_SUBSCRIBERS
#define ROSSERIAL_MAX_SUBSCRIBERS 25
#endif

#ifndef ROSSERIAL_MAX_PUBLISHERS
#define ROSSERIAL_MAX_PUBLISHERS 25
#endif

#ifndef ROSSERIAL_USE_RTOS_CLOCK
#define ROSSERIAL_USE_RTOS_CLOCK 0
#endif

class MbedHardware : private mbed::SerialBase, private mbed::NonCopyable<MbedHardware> {
public:
    MbedHardware(PinName tx, PinName rx, int baud);

    MbedHardware();

    void init();

    void setBaud(int baud);

    int read();

    ssize_t write(uint8_t *data, size_t length);

    uint32_t time();

private:
    bool _tx_irq_enabled = false;
    bool _rx_irq_enabled = false;
    bool _tx_enabled     = true;
    bool _rx_enabled     = true;
    bool _blocking       = true;
    mbed::CircularBuffer<char, ROSSERIAL_CIRCULAR_BUFFER_SIZE> _rxbuf;
    mbed::CircularBuffer<char, ROSSERIAL_CIRCULAR_BUFFER_SIZE> _txbuf;
    PlatformMutex _mutex;

#if ROSSERIAL_USE_RTOS_CLOCK == 0
    mbed::Timer _t;
#endif

private:
    /** Acquire mutex
     */
    void api_lock(void);

    /** Release mutex
     */
    void api_unlock(void);

    /** Enable processing of byte reception IRQs and register a callback to
     * process them.
     */
    void enable_rx_irq();

    /** Disable processing of byte reception IRQs and de-register callback to
     * process them.
     */
    void disable_rx_irq();

    /** Enable processing of byte transmission IRQs and register a callback to
     * process them.
     */
    void enable_tx_irq();

    /** Disable processing of byte transmission IRQs and de-register callback to
     * process them.
     */
    void disable_tx_irq();

    /** ISRs for serial
     *  Routines to handle interrupts on serial pins.
     *  Copies data into Circular Buffer.
     */
    void tx_irq(void);
    void rx_irq(void);
};

#endif /* ROS_MBED_HARDWARE_H_ */
