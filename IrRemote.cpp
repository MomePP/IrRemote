#include "IrRemote.h"

#include "esp_log.h"
static const char *TAG = "IrRemote";

#define IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us


IrRemote::IrRemote()
    : remoteTx(), remoteRx() {}

IrRemote::~IrRemote() {}

void IrRemote::begin(ir_remote_protocol irProtocol, int txPin, int rxPin)
{
    _ir_protocol = irProtocol;
    _txPin = (gpio_num_t)txPin;
    _rxPin = (gpio_num_t)rxPin;

    if (_txPin >= 0)
        remoteTx.begin(_txPin, IR_RESOLUTION_HZ);
    if (_rxPin >= 0)
        remoteRx.begin(_rxPin, IR_RESOLUTION_HZ);
}

bool IrRemote::getIrCode(uint32_t timeout_ms)
{
    bool retrived_status = false;
    switch (_ir_protocol)
    {
    case IR_NEC:
        retrived_status = remoteRx.receiveNEC(&_received_code, pdMS_TO_TICKS(timeout_ms));
        break;
    case IR_SONY:
        // TODO: sony protocol might update in future
        break;
    };
    return retrived_status;
}

bool IrRemote::sendIrCode(ir_code_t send_code, uint32_t timeout_ms)
{
    bool sent_status = false;
    switch (_ir_protocol)
    {
    case IR_NEC:
        sent_status = remoteTx.sendNEC(send_code.address, send_code.command);
        break;
    case IR_SONY:
        // TODO: sony protocol might update in future
        break;
    };
    return sent_status;
}

bool IrRemote::testReadIrCode(uint32_t timeout_ms)
{
    return remoteRx.receiveNEC(&_received_code, pdMS_TO_TICKS(timeout_ms));
}

bool IrRemote::testSendIrCode(void)
{
    // NEC protocol example data
    const ir_code_t sent_code = {
        .address = 0x0440,
        .command = 0x3003,
    };
    return remoteTx.sendNEC(sent_code.address, sent_code.command);
}
