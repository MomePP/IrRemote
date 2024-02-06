#pragma once

#include "Remote-Rx.h"
#include "Remote-Tx.h"

class IrRemote : public IrRemoteTx, public IrRemoteRx
{
public:
    IrRemote(void);
    ~IrRemote(void);

    void begin(int txPin, int rxPin);

    bool testReadIrCode(uint32_t timeout_ms = 1000);
    bool testSendIrCode(void);

private:
    IrRemoteTx remoteTx;
    IrRemoteRx remoteRx;

    gpio_num_t _txPin = GPIO_NUM_NC;
    gpio_num_t _rxPin = GPIO_NUM_NC;
};
