#pragma once

#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "Ir-NEC.h"
#include "Ir-Sony.h"
#include "Ir-Data.h"

// Holds the largest expected RX frame. An IR_DATA max frame is
// IR_DATA_MAX_FRAME_SYMBOLS (146) symbols, so 128 is too small; 192 gives headroom.
// The RX channel mem_block (64) is unchanged — the driver ping-pongs into this buffer.
#define IR_RX_SYMBOL_BUF_SIZE 192

static bool _rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);

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
    // (Re-)arm the RX channel. Never abort()s on a transient RMT error — a
    // peripheral hiccup must not reset the whole board; it logs and carries on.
    void _armReceive(void);

    QueueHandle_t _receive_queue;
    rmt_channel_handle_t _channel;
    rmt_symbol_word_t _symbols_buffer[IR_RX_SYMBOL_BUF_SIZE];
    rmt_receive_config_t _receive_config;
};
