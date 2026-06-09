#include "IrRemote.h"

#include "esp_log.h"
static const char *TAG = "IrRemote";

#define IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us


IrRemote::IrRemote()
    : remoteTx(), remoteRx() {}

IrRemote::~IrRemote() {}

void IrRemote::begin(ir_remote_protocol irProtocol, int txPin, int rxPin)
{
    setProtocol(irProtocol);
    _txPin = (gpio_num_t)txPin;
    _rxPin = (gpio_num_t)rxPin;

    if (_txPin >= 0)
        remoteTx.begin(_txPin, IR_RESOLUTION_HZ);
    if (_rxPin >= 0)
        remoteRx.begin(_rxPin, IR_RESOLUTION_HZ);
}

bool IrRemote::getIrCode(uint32_t timeout_ms)
{
    bool retrieved_status = false;
    switch (_ir_protocol)
    {
    case IR_AUTO:
        retrieved_status = remoteRx.receiveAuto(&_received_code, &_detected_protocol, pdMS_TO_TICKS(timeout_ms));
        break;
    case IR_NEC:
        retrieved_status = remoteRx.receiveNEC(&_received_code, pdMS_TO_TICKS(timeout_ms));
        if (retrieved_status)
            _detected_protocol = IR_NEC;
        break;
    case IR_SONY:
        retrieved_status = remoteRx.receiveSony(&_received_code, pdMS_TO_TICKS(timeout_ms));
        if (retrieved_status)
            _detected_protocol = IR_SONY;
        break;
    case IR_DATA:
        break; // data mode uses receiveData(), not the ir_code_t path
    };
    return retrieved_status;
}

bool IrRemote::sendIrCode(ir_code_t send_code, uint32_t timeout_ms)
{
    bool sent_status = false;
    switch (_ir_protocol)
    {
    case IR_AUTO:
        // TX is bench-only; transmit with the last detected protocol (IR_NEC until
        // the first frame is received).
        sent_status = (_detected_protocol == IR_SONY)
                          ? remoteTx.sendSony(send_code.address, send_code.command)
                          : remoteTx.sendNEC(send_code.address, send_code.command);
        break;
    case IR_NEC:
        sent_status = remoteTx.sendNEC(send_code.address, send_code.command);
        break;
    case IR_SONY:
        sent_status = remoteTx.sendSony(send_code.address, send_code.command);
        break;
    case IR_DATA:
        break; // data mode uses sendData(), not the ir_code_t path
    };
    return sent_status;
}

bool IrRemote::testReadIrCode(uint32_t timeout_ms)
{
    return getIrCode(timeout_ms);
}

bool IrRemote::testSendIrCode(void)
{
    switch (_ir_protocol)
    {
    case IR_AUTO:
    {
        // bench helper: mirror sendIrCode's Auto behaviour (detected protocol)
        if (_detected_protocol == IR_SONY)
        {
            const ir_code_t sony_code = {.address = 0x01, .command = 0x15};
            return remoteTx.sendSony(sony_code.address, sony_code.command);
        }
        const ir_code_t nec_code = {.address = 0x0440, .command = 0x3003};
        return remoteTx.sendNEC(nec_code.address, nec_code.command);
    }
    case IR_NEC:
    {
        const ir_code_t nec_code = {.address = 0x0440, .command = 0x3003};
        return remoteTx.sendNEC(nec_code.address, nec_code.command);
    }
    case IR_SONY:
    {
        // Sony SIRC example: address 1 (TV), command 21 (power)
        const ir_code_t sony_code = {.address = 0x01, .command = 0x15};
        return remoteTx.sendSony(sony_code.address, sony_code.command);
    }
    case IR_DATA:
        break; // no remote-code test frame for data mode
    }
    return false;
}
