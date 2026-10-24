#include "Remote-Tx.h"

#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char *TAG = "IrRemoteTx";

IrRemoteTx::IrRemoteTx()
{
    _channel       = NULL;
    _encoder       = NULL;
    _copy_encoder  = NULL;
    _next_payload  = 0;
}

IrRemoteTx::~IrRemoteTx()
{
}

void IrRemoteTx::begin(gpio_num_t tx_gpio_num, uint32_t resolution_hz)
{
    _tx_lock = xSemaphoreCreateMutex();
    assert(_tx_lock);

    ESP_LOGI(TAG, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .gpio_num = tx_gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = IR_TX_QUEUE_DEPTH, // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &_channel));

    ESP_LOGI(TAG, "set default modulate carrier to TX channel");
    _resolution          = resolution_hz;
    _current_carrier_hz  = IR_TX_CARRIER_FREQ_HZ_NEC;
    _carrier_cfg = {
        .frequency_hz = _current_carrier_hz,
        .duty_cycle   = IR_TX_CARRIER_DUTY,
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

    ESP_LOGI(TAG, "install RMT copy encoder for Sony frames");
    rmt_copy_encoder_config_t copy_encoder_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_cfg, &_copy_encoder));

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
    // Don't abort() the board on a transient carrier-apply failure — log and bail;
    // _current_carrier_hz stays unchanged so the next send retries the apply.
    esp_err_t err = rmt_apply_carrier(_channel, &_carrier_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "rmt_apply_carrier failed: %s", esp_err_to_name(err));
        return;
    }
    _current_carrier_hz = _carrier_cfg.frequency_hz;
}

bool IrRemoteTx::sendNEC(uint16_t address, uint16_t command)
{
    TxLock lock(_tx_lock);
    uint32_t target = _carrierFor(IR_TX_CARRIER_FREQ_HZ_NEC);
    if (_current_carrier_hz != target)
        updateCarrierFrequency(target);
    ESP_LOGD(TAG, "send NEC frame");
    ir_code_t *nec_code = &_payloads[_next_payload];
    nec_code->address = address;
    nec_code->command = command;
    _next_payload = (_next_payload + 1) % IR_TX_PAYLOAD_SLOTS;
    return rmt_transmit(_channel, _encoder, nec_code, sizeof(*nec_code), &_transmit_config) == ESP_OK;
}

// NOTE: blocks the calling task ~135ms — SIRC requires SONY_REPEAT_COUNT (3)
// frames, each waited to completion then spaced by SONY_FRAME_PERIOD_MS (45ms).
// Do not call from a latency-sensitive task (e.g. the Logo VM loop).
bool IrRemoteTx::sendSony(uint16_t address, uint16_t command)
{
    TxLock lock(_tx_lock);
    ESP_LOGD(TAG, "send Sony frame");
    uint32_t target = _carrierFor(IR_TX_CARRIER_FREQ_HZ_SONY);
    if (_current_carrier_hz != target)
        updateCarrierFrequency(target);

    ir_code_t sony_code = {
        .address = address,
        .command = command,
    };
    rmt_symbol_word_t frame[IR_SONY_RX_SYMBOL_COUNT];
    sony_build_frame(frame, sony_code, _resolution);

    bool ok = true;
    for (int i = 0; i < SONY_REPEAT_COUNT; i++)
    {
        ok &= rmt_transmit(_channel, _copy_encoder, frame, sizeof(frame), &_transmit_config) == ESP_OK;
        // ensure frame done before spacing; on error log + abort the loop, never abort() the board
        esp_err_t err = rmt_tx_wait_all_done(_channel, -1);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "rmt_tx_wait_all_done failed: %s", esp_err_to_name(err));
            ok = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(SONY_FRAME_PERIOD_MS));
    }
    return ok;
}

// One read of the override: clearTransmitRange() can zero it between two reads.
uint32_t IrRemoteTx::_carrierFor(uint32_t protocol_hz)
{
    const uint32_t range_hz = _range_carrier_hz;
    return range_hz ? range_hz : protocol_hz;
}

uint32_t IrRemoteTx::rangeToCarrierHz(uint16_t cm)
{
    // Sorted ascending by cm; clamp to the table endpoints. Out-of-range or wrapped cm
    // (e.g. a negative Logo value cast to uint16_t -> ~65535) clamps to max range.
    if (cm <= IR_RANGE_CALIBRATION[0].cm)
        return IR_RANGE_CALIBRATION[0].hz;
    if (cm >= IR_RANGE_CALIBRATION[IR_RANGE_CAL_COUNT - 1].cm)
        return IR_RANGE_CALIBRATION[IR_RANGE_CAL_COUNT - 1].hz;

    // Find the bracketing pair and linearly interpolate Hz across [lo.cm, hi.cm].
    for (size_t i = 1; i < IR_RANGE_CAL_COUNT; i++)
    {
        if (cm <= IR_RANGE_CALIBRATION[i].cm)
        {
            const ir_range_cal_point_t lo = IR_RANGE_CALIBRATION[i - 1];
            const ir_range_cal_point_t hi = IR_RANGE_CALIBRATION[i];
            int32_t span_cm = (int32_t)hi.cm - (int32_t)lo.cm;
            int32_t span_hz = (int32_t)hi.hz - (int32_t)lo.hz;
            return (uint32_t)((int32_t)lo.hz + span_hz * ((int32_t)cm - (int32_t)lo.cm) / span_cm);
        }
    }
    return IR_RANGE_CALIBRATION[IR_RANGE_CAL_COUNT - 1].hz; // unreachable
}

void IrRemoteTx::transmitRange(uint16_t cm)
{
    TxLock lock(_tx_lock);
    // One read: clearTransmitRange() takes no lock and can zero _range_carrier_hz between a store and a reload.
    const uint32_t range_hz = rangeToCarrierHz(cm);
    _range_carrier_hz = range_hz;
    updateCarrierFrequency(range_hz); // existing clamp + apply + cache
    ESP_LOGI(TAG, "transmitRange: %u cm -> carrier %u Hz", cm, range_hz);
}

void IrRemoteTx::clearTransmitRange(void)
{
    _range_carrier_hz = 0; // lazy: the RMT carrier reverts on the NEXT send, not now
    ESP_LOGI(TAG, "clearTransmitRange: revert to per-protocol carrier");
}

// NOTE: blocks the caller for the random back-off plus the whole frame (~frame duration). Data uses
// the 38 kHz carrier; honors the range-carrier override exactly like sendNEC.
bool IrRemoteTx::sendData(const uint8_t *bytes, uint8_t len)
{
    if (len > IR_DATA_MAX_PAYLOAD)
        return false;

    // Back off before taking the lock so a sender queued behind this one is not held up by the wait.
    vTaskDelay(pdMS_TO_TICKS(esp_random() % (IR_DATA_TX_BACKOFF_MAX_MS + 1)));

    TxLock lock(_tx_lock);
    uint32_t target = _carrierFor(IR_TX_CARRIER_FREQ_HZ_DATA);
    if (_current_carrier_hz != target)
        updateCarrierFrequency(target);

    size_t symbols = data_build_frame(_data_frame, bytes, len, _resolution);
    if (symbols == 0)
        return false;

    ESP_LOGD(TAG, "send data frame: len=%u, symbols=%u", (unsigned)len, (unsigned)symbols);
    if (rmt_transmit(_channel, _copy_encoder, _data_frame, symbols * sizeof(rmt_symbol_word_t), &_transmit_config) != ESP_OK)
        return false;
    esp_err_t err = rmt_tx_wait_all_done(_channel, -1);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "data rmt_tx_wait_all_done failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}
