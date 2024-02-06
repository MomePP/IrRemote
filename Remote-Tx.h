#pragma once

#include "Ir-NEC.h"
#include "driver/rmt_tx.h"

#define IR_TX_CARRIER_FREQ_HZ_MAX 38000
#define IR_TX_CARRIER_FREQ_HZ_MIN 30000
#define IR_TX_CARRIER_DUTY        0.33

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
};
