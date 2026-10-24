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

// Remote-code TX is always NEC, whatever the receive protocol. RX adds SONY_IR_CODE_OFFSET to a
// Sony press and TX would have to subtract it, and the Auto receive mode follows whatever frame
// was seen last, which made the same `ir_send_code N` go out as NEC or Sony depending on history.
bool IrRemote::sendIrCode(ir_code_t send_code)
{
    return remoteTx.sendNEC(send_code.address, send_code.command);
}
