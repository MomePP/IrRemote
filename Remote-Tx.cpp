#include "Remote-Tx.h"

#include "esp_log.h"
static const char *TAG = "IrRemoteTx";

IrRemoteTx::IrRemoteTx()
{
    _channel = NULL;
    _encoder = NULL;
}

IrRemoteTx::~IrRemoteTx()
{
}

void IrRemoteTx::begin(gpio_num_t tx_gpio_num, uint32_t resolution_hz)
{
    ESP_LOGI(TAG, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .gpio_num = tx_gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &_channel));

    ESP_LOGI(TAG, "set default modulate carrier to TX channel");
    _carrier_cfg = {
        .frequency_hz = IR_TX_CARRIER_FREQ_HZ_MAX,
        .duty_cycle = IR_TX_CARRIER_DUTY,
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(_channel, &_carrier_cfg));

    // this example won't send NEC frames in a loop
    _transmit_config = {
        .loop_count = 0, // no loop
    };

    ESP_LOGI(TAG, "install IR NEC encoder");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = resolution_hz
    };
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &_encoder));

    ESP_LOGI(TAG, "enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(_channel));
}

void IrRemoteTx::updateCarrierFrequency(uint32_t frequency)
{
    _carrier_cfg.frequency_hz = (frequency < IR_TX_CARRIER_FREQ_HZ_MIN)
                                    ? IR_TX_CARRIER_FREQ_HZ_MIN
                                : (frequency > IR_TX_CARRIER_FREQ_HZ_MAX)
                                    ? IR_TX_CARRIER_FREQ_HZ_MAX
                                    : frequency;
    ESP_ERROR_CHECK(rmt_apply_carrier(_channel, &_carrier_cfg));
}

bool IrRemoteTx::sendNEC(uint16_t address, uint16_t command)
{
    ESP_LOGD(TAG, "send NEC frame");
    ir_code_t nec_code = {
        .address = address,
        .command = command,
    };
    return rmt_transmit(_channel, _encoder, &nec_code, sizeof(nec_code), &_transmit_config) == ESP_OK;
}
