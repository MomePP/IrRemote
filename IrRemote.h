#pragma once

#include "Arduino.h"

#include "Remote-Rx.h"
#include "Remote-Tx.h"

#include "IrProtocol.h"

class IrRemote : public IrRemoteTx, public IrRemoteRx
{
public:
    IrRemote(void);
    ~IrRemote(void);

    void begin(ir_remote_protocol irProtocol, int txPin, int rxPin);

    bool getIrCode(uint32_t timeout);
    bool sendIrCode(ir_code_t send_code, uint32_t timeout_ms);

    // _ir_protocol is written here (e.g. from the UI/menu task) and read in
    // getIrCode() from the IR-poll task on a different core. Mark it volatile and
    // barrier the store so the reader sees the switch on the next poll
    // (cross-core shared state, per project rules).
    void setProtocol(ir_remote_protocol irProtocol)
    {
        _ir_protocol = irProtocol;
        __asm__ volatile("memw" ::: "memory");
    }
    ir_remote_protocol getProtocol(void) { return _ir_protocol; }

    // Last protocol actually decoded from a received frame. Written and read only
    // inside the IR-poll task (getIrCode -> firmware poll), so no cross-core barrier
    // is needed (unlike _ir_protocol, which the menu task also writes).
    ir_remote_protocol getDetectedProtocol(void) { return _detected_protocol; }

    uint8_t getIrAddress(void) { return lowByte(_received_code.address); }
    uint8_t getIrCommand(void) { return lowByte(_received_code.command); }

    bool testReadIrCode(uint32_t timeout_ms = 1000);
    bool testSendIrCode(void);

private:
    IrRemoteTx remoteTx;
    IrRemoteRx remoteRx;

    gpio_num_t _txPin = GPIO_NUM_NC;
    gpio_num_t _rxPin = GPIO_NUM_NC;

    volatile ir_remote_protocol _ir_protocol;
    ir_code_t _received_code;
    ir_remote_protocol _detected_protocol = IR_NEC; // sane cold-start before any frame
};
