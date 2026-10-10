#pragma once

#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "Ir-NEC.h"

// Capacity of one RX frame buffer, in RMT symbols. The longest valid NEC frame is
// 34 symbols; the slack only has to keep a longer (noise) burst from
// being reported as a 34-symbol frame. The driver truncates a longer burst to the buffer
// size (no overrun), and every count other than 34/2 decodes to {0,0} anyway.
#define IR_RX_SYMBOL_BUF_SIZE 40
// The ISR re-arms the channel into the buffer the consumer does not own, so two are needed.
#define IR_RX_BUFFER_COUNT 2

class IrRemoteRx
{
public:
    IrRemoteRx();
    ~IrRemoteRx();

    void begin(gpio_num_t rx_gpio_num, uint32_t resolution_hz);

    bool receiveNEC(ir_code_t *received_code, TickType_t timeout);

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
