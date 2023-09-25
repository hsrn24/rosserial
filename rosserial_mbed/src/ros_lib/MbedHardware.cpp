/** @file MbedHardware.h
 * Serial hardware implementation for mbed.
 */

#include "MbedHardware.h"
#include "platform/mbed_thread.h"
#include "platform/mbed_version.h"
#include "rtos/Kernel.h"

MbedHardware::MbedHardware(PinName tx, PinName rx, int baud) : mbed::SerialBase(tx, rx, baud)
{
    MBED_VERSION_CHECK(6, 13, 0);
    enable_rx_irq();
}

MbedHardware::MbedHardware() : MbedHardware(ROSSERIAL_TX, ROSSERIAL_RX, ROSSERIAL_BAUDRATE) {}

void MbedHardware::init()
{
#if ROSSERIAL_USE_RTOS_CLOCK == 0
    _t.start();
#endif
}

void MbedHardware::setBaud(int baud) { SerialBase::baud(baud); }

int MbedHardware::read()
{
    int ret   = -1;
    char data = 0;

    api_lock();

    if (!_rxbuf.empty() && _rxbuf.pop(data)) {
        ret = data;
    }

    core_util_critical_section_enter();
    if (_rx_enabled && !_rx_irq_enabled) {
        // only read from hardware in one place
        MbedHardware::rx_irq();
        if (!_rxbuf.full()) {
            enable_rx_irq();
        }
    }
    core_util_critical_section_exit();

    api_unlock();

    return ret;
}

ssize_t MbedHardware::write(uint8_t *data, size_t length)
{
    size_t data_written = 0;
    const char *buf_ptr = reinterpret_cast<const char *>(data);

    if (length == 0) {
        return 0;
    }

    api_lock();

    // Unlike read, we should write the whole thing if blocking. POSIX only
    // allows partial as a side-effect of signal handling; it normally tries to
    // write everything if blocking. Without signals we can always write all.
    while (data_written < length) {
        if (_txbuf.full()) {
            if (!_blocking) {
                break;
            }
            do {
                api_unlock();
                // Should we have a proper wait?
                thread_sleep_for(1);
                api_lock();
            } while (_txbuf.full());
        }

        while (data_written < length && !_txbuf.full()) {
            _txbuf.push(*buf_ptr++);
            data_written++;
        }

        core_util_critical_section_enter();
        if (_tx_enabled && !_tx_irq_enabled) {
            // only write to hardware in one place
            MbedHardware::tx_irq();
            if (!_txbuf.empty()) {
                enable_tx_irq();
            }
        }
        core_util_critical_section_exit();
    }

    api_unlock();

    return data_written != 0 ? (ssize_t) data_written : (ssize_t) -EAGAIN;
}

uint32_t MbedHardware::time()
{
#if ROSSERIAL_USE_RTOS_CLOCK
    return std::chrono::duration_cast<std::chrono::duration<uint32_t, std::milli>>(
               rtos::Kernel::Clock::now().time_since_epoch())
        .count();
#else
    return std::chrono::duration_cast<std::chrono::duration<uint32_t, std::milli>>(_t.elapsed_time()).count();
#endif
}

#if defined(ROSSERIAL_NO_LOCK)
void MbedHardware::api_lock(void) { }
void MbedHardware::api_unlock(void) { }
#else
void MbedHardware::api_lock(void) { _mutex.lock(); }
void MbedHardware::api_unlock(void) { _mutex.unlock(); }
#endif

/* These are all called from critical section
 * Attatch IRQ routines to the serial device.
 */
void MbedHardware::enable_rx_irq()
{
    SerialBase::attach(callback(this, &MbedHardware::rx_irq), RxIrq);
    _rx_irq_enabled = true;
}

void MbedHardware::disable_rx_irq()
{
    SerialBase::attach(NULL, RxIrq);
    _rx_irq_enabled = false;
}

void MbedHardware::enable_tx_irq()
{
    SerialBase::attach(callback(this, &MbedHardware::tx_irq), TxIrq);
    _tx_irq_enabled = true;
}

void MbedHardware::disable_tx_irq()
{
    SerialBase::attach(NULL, TxIrq);
    _tx_irq_enabled = false;
}

void MbedHardware::rx_irq(void)
{
    // Fill in the receive buffer if the peripheral is readable
    // and receive buffer is not full.
    while (!_rxbuf.full() && SerialBase::readable()) {
        char data = SerialBase::_base_getc();
        _rxbuf.push(data);
    }

    if (_rx_irq_enabled && _rxbuf.full()) {
        disable_rx_irq();
    }
}

// Also called from write to start transfer
void MbedHardware::tx_irq(void)
{
    char data;

    // Write to the peripheral if there is something to write
    // and if the peripheral is available to write.
    while (SerialBase::writeable() && _txbuf.pop(data)) {
        SerialBase::_base_putc(data);
    }

    if (_tx_irq_enabled && _txbuf.empty()) {
        disable_tx_irq();
    }
}
