#pragma once

#include "Remote-Tx.h"
#include "Remote-Rx.h"

#define EXAMPLE_IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us

class IrRemote : public IrRemoteTx, public IrRemoteRx
{
public:
    IrRemote();
    ~IrRemote();

    void begin(void);

private:
    IrRemoteTx remoteTx;
    IrRemoteRx remoteRx;
};
