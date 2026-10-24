#include "Remote-Rx.h"

#include "esp_log.h"
static const char *TAG = "IrRemoteRx";

IrRemoteRx::IrRemoteRx()
{
    _receive_queue = NULL;
    _channel = NULL;
    _fill_index = 0;
    for (volatile bool &held : _held)
        held = false;
}

IrRemoteRx::~IrRemoteRx() {}

void IrRemoteRx::begin(gpio_num_t rx_gpio_num, uint32_t resolution_hz)
{
    ESP_LOGI(TAG, "create RMT RX channel");
    rmt_rx_channel_config_t rx_channel_cfg = {
        .gpio_num = rx_gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
    };
    _channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &_channel));

    ESP_LOGI(TAG, "register RX done callback");
    _receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(_receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = _rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(_channel, &cbs, this));

    ESP_LOGI(TAG, "enable RMT RX channel");
    ESP_ERROR_CHECK(rmt_enable(_channel));

    // the following timing requirement is based on NEC protocol
    _receive_config = {
        .signal_range_min_ns = IR_NEC_SIGNAL_RANGE_MIN,
        .signal_range_max_ns = IR_NEC_SIGNAL_RANGE_MAX,
    };
    // save the received RMT symbols
    ESP_LOGI(TAG, "start receive RMT symbols");
    ESP_ERROR_CHECK(rmt_receive(_channel, _symbols_buffer[_fill_index], sizeof(_symbols_buffer[0]), &_receive_config));
}

bool IrRemoteRx::receiveNEC(ir_code_t *received_code, TickType_t timeout)
{
    rmt_rx_done_event_data_t rx_data;

    // wait for RX done signal
    bool received_status = xQueueReceive(_receive_queue, &rx_data, timeout) == pdPASS;
    if (received_status)
    {
        // parse the receive symbols and print the result
        *received_code = nec_decode_frame(rx_data.received_symbols, rx_data.num_symbols);
        // the channel was already re-armed from the ISR; give the frame buffer back to it
        _releaseBuffer(rx_data.received_symbols);
    }
    return received_status;
}

bool IrRemoteRx::receiveSony(ir_code_t *received_code, TickType_t timeout)
{
    rmt_rx_done_event_data_t rx_data;
    bool received_status = xQueueReceive(_receive_queue, &rx_data, timeout) == pdPASS;
    if (received_status)
    {
        // a NEC frame or noise is not a Sony press: report no code instead of {0,0}
        received_status = sony_decode_frame(rx_data.received_symbols, rx_data.num_symbols, received_code);
        // the channel was already re-armed from the ISR (NEC range covers Sony); give the frame buffer back
        _releaseBuffer(rx_data.received_symbols);
    }
    return received_status;
}

bool IrRemoteRx::receiveAuto(ir_code_t *received_code, ir_remote_protocol *detected, TickType_t timeout)
{
    rmt_rx_done_event_data_t rx_data;
    bool received_status = xQueueReceive(_receive_queue, &rx_data, timeout) == pdPASS;
    if (received_status)
    {
        // Symbol-count dispatch for the remote-control protocols. Counts never collide
        // AMONG NEC/Sony: Sony 12-bit = 13, NEC frame = 34, NEC repeat = 2. (IR_DATA
        // frames are NOT dispatched here — they use receiveData(); note a 2-byte data
        // frame is also 34 symbols, so the modes are intentionally exclusive.)
        size_t symbol_num = rx_data.num_symbols;
        if (symbol_num == IR_SONY_RX_SYMBOL_COUNT)
        {
            // a malformed 13-symbol burst must neither report a code nor flip *detected to Sony
            received_status = sony_decode_frame(rx_data.received_symbols, symbol_num, received_code);
            if (received_status)
                *detected = IR_SONY;
        }
        else if (symbol_num == IR_NEC_RX_FRAME_SYMBOL_COUNT ||
                 symbol_num == IR_NEC_RX_REPEAT_SYMBOL_COUNT)
        {
            *received_code = nec_decode_frame(rx_data.received_symbols, symbol_num);
            *detected = IR_NEC;
        }
        else
        {
            received_status = false; // unknown frame: no code, *detected unchanged
        }
        // the channel was already re-armed from the ISR (NEC range covers Sony); give the frame buffer back
        _releaseBuffer(rx_data.received_symbols);
    }
    return received_status;
}

bool IrRemoteRx::receiveData(uint8_t *out, uint8_t *out_len, TickType_t timeout)
{
    rmt_rx_done_event_data_t rx_data;
    bool received_status = xQueueReceive(_receive_queue, &rx_data, timeout) == pdPASS;
    if (received_status)
    {
        int len = data_decode_frame(rx_data.received_symbols, rx_data.num_symbols, out);
        _releaseBuffer(rx_data.received_symbols);
        if (len < 0)
            return false; // garbled / CRC mismatch — a frame arrived but is unusable
        *out_len = (uint8_t)len;
        return true;
    }
    return false;
}

void IrRemoteRx::_releaseBuffer(const rmt_symbol_word_t *symbols)
{
    __asm__ volatile("memw" ::: "memory"); // finish reading the symbols before the ISR may reuse them
    _held[symbols == _symbols_buffer[1]] = false;
}

// Runs in the RMT ISR. The driver has already switched the channel back to the enabled
// state, so rmt_receive() is legal here: re-arming immediately closes the window in which
// the old consumer-side re-arm (up to one poll period later) could miss the next repeat code.
void IrRemoteRx::_onReceiveDone(const rmt_rx_done_event_data_t *edata, BaseType_t *high_task_wakeup)
{
    const uint8_t other = _fill_index ^ 1;
    // the other buffer is still owned by the consumer: drop this frame and refill the same
    // buffer (the old code did not receive at all while the consumer had not caught up)
    if (!_held[other])
    {
        _held[_fill_index] = true;
        __asm__ volatile("memw" ::: "memory");
        if (xQueueSendFromISR(_receive_queue, edata, high_task_wakeup) == pdPASS)
            _fill_index = other;
        else
            _held[_fill_index] = false;
    }
    // same arguments as the begin() arming call, which cannot fail after it succeeded once
    rmt_receive(_channel, _symbols_buffer[_fill_index], sizeof(_symbols_buffer[0]), &_receive_config);
}

bool IrRemoteRx::_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    static_cast<IrRemoteRx *>(user_data)->_onReceiveDone(edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}
