#pragma once

#include "driver/rmt_tx.h"

#include "Ir-NEC.h"
#include "Ir-Sony.h"

#define IR_TX_CARRIER_FREQ_HZ_NEC  38000
#define IR_TX_CARRIER_FREQ_HZ_SONY 40000
#define IR_TX_CARRIER_FREQ_HZ_MAX  40000 // raised from 38000 so Sony's 40kHz is not clamped
#define IR_TX_CARRIER_FREQ_HZ_MIN  30000
#define IR_TX_CARRIER_DUTY         0.33

class IrRemoteTx
{
public:
    IrRemoteTx();
    ~IrRemoteTx();

    void begin(gpio_num_t tx_gpio_num, uint32_t resolution_hz);
    void updateCarrierFrequency(uint32_t frequency);

    bool sendNEC(uint16_t address, uint16_t command);
    bool sendSony(uint16_t address, uint16_t command);

private:
    rmt_channel_handle_t _channel;
    rmt_carrier_config_t _carrier_cfg;
    rmt_transmit_config_t _transmit_config;
    rmt_encoder_handle_t _encoder;       // NEC custom encoder
    rmt_encoder_handle_t _copy_encoder;  // used to push pre-built Sony frames
    uint32_t _resolution;                // RMT tick resolution, for Sony frame building
    uint32_t _current_carrier_hz;        // cache to avoid redundant rmt_apply_carrier
};
