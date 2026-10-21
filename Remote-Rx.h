#pragma once

#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "Ir-NEC.h"
#include "Ir-Sony.h"
#include "Ir-Data.h"

// Capacity of one RX frame buffer, in RMT symbols. It must hold the longest IR_DATA frame
// (IR_DATA_MAX_FRAME_SYMBOLS, 146 symbols); 192 gives headroom. The NEC (34 symbols), Sony (13)
// and repeat frames are all shorter. The RX channel mem_block (64) is unchanged: the driver
// ping-pongs into this buffer and truncates a longer burst to its size (no overrun).
#define IR_RX_SYMBOL_BUF_SIZE 192
// The ISR re-arms the channel into the buffer the consumer does not own, so two are needed.
#define IR_RX_BUFFER_COUNT 2

class IrRemoteRx
{
public:
    IrRemoteRx();
    ~IrRemoteRx();

    void begin(gpio_num_t rx_gpio_num, uint32_t resolution_hz);

    bool receiveNEC(ir_code_t *received_code, TickType_t timeout);
    bool receiveSony(ir_code_t *received_code, TickType_t timeout);

    // Auto-detect: dequeue one frame from the shared window and dispatch by RMT
    // symbol count. Writes *detected (IR_NEC / IR_SONY) for the frame's actual
    // protocol; leaves *detected unchanged on an unrecognised symbol count.
    bool receiveAuto(ir_code_t *received_code, ir_remote_protocol *detected, TickType_t timeout);

    // Dequeue one frame from the shared window and decode it as an IR_DATA frame.
    // On success writes up to IR_DATA_MAX_PAYLOAD bytes to `out`, sets *out_len, returns
    // true. Returns false on timeout, a garbled frame, or a CRC mismatch.
    bool receiveData(uint8_t *out, uint8_t *out_len, TickType_t timeout);

private:
    static bool _rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);
    void _onReceiveDone(const rmt_rx_done_event_data_t *edata, BaseType_t *high_task_wakeup);
    void _releaseBuffer(const rmt_symbol_word_t *symbols);

    QueueHandle_t _receive_queue;
    rmt_channel_handle_t _channel;
    rmt_symbol_word_t _symbols_buffer[IR_RX_BUFFER_COUNT][IR_RX_SYMBOL_BUF_SIZE];
    rmt_receive_config_t _receive_config;
    uint8_t _fill_index;                     // buffer the channel is filling; ISR only
    volatile bool _held[IR_RX_BUFFER_COUNT]; // handed to the consumer, not yet released
};
