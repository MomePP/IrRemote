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

    uint8_t getIrAddress(void) { return lowByte(_received_code.address); }
    uint8_t getIrCommand(void) { return lowByte(_received_code.command); }

    bool testReadIrCode(uint32_t timeout_ms = 1000);
    bool testSendIrCode(void);

private:
    IrRemoteTx remoteTx;
    IrRemoteRx remoteRx;

    gpio_num_t _txPin = GPIO_NUM_NC;
    gpio_num_t _rxPin = GPIO_NUM_NC;

    ir_remote_protocol _ir_protocol;
    ir_code_t _received_code;
};
