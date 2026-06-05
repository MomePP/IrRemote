#pragma once

#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "Ir-NEC.h"
#include "Ir-Sony.h"

#define IR_RX_SYMBOL_BUF_SIZE 128

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

private:
    // (Re-)arm the RX channel. Never abort()s on a transient RMT error — a
    // peripheral hiccup must not reset the whole board; it logs and carries on.
    void _armReceive(void);

    QueueHandle_t _receive_queue;
    rmt_channel_handle_t _channel;
    rmt_symbol_word_t _symbols_buffer[IR_RX_SYMBOL_BUF_SIZE];
    rmt_receive_config_t _receive_config;
};
