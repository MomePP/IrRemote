#pragma once

#include "driver/rmt_tx.h"

#include "Ir-NEC.h"

#define IR_TX_CARRIER_FREQ_HZ_MAX 38000
#define IR_TX_CARRIER_FREQ_HZ_MIN 30000
#define IR_TX_CARRIER_DUTY        0.33
#define IR_TX_QUEUE_DEPTH         4 // transactions allowed to be pending in the driver
// rmt_transmit() keeps only a pointer to the payload and the encoder reads it later (from the
// ISR once the frame is queued behind another), so every queued frame needs its own slot.
// One more than the queue depth: a slot is rewritten before rmt_transmit() blocks for a
// free transaction, at which point the oldest frame's slot may still be in flight.
#define IR_TX_PAYLOAD_SLOTS       (IR_TX_QUEUE_DEPTH + 1)

class IrRemoteTx
{
public:
    IrRemoteTx();
    ~IrRemoteTx();

    void begin(gpio_num_t tx_gpio_num, uint32_t resolution_hz);
    void updateCarrierFrequency(uint32_t frequency);

    bool sendNEC(uint16_t address, uint16_t command);

private:
    rmt_channel_handle_t _channel;
    rmt_carrier_config_t _carrier_cfg;
    rmt_transmit_config_t _transmit_config;
    rmt_encoder_handle_t _encoder;
    ir_code_t _payloads[IR_TX_PAYLOAD_SLOTS];
    uint8_t _next_payload;
};
