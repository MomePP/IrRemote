#pragma once

#include "Ir-NEC.h"
#include "driver/rmt_rx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define IR_RX_SYMBOL_BUF_SIZE 128

static bool _rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);

class IrRemoteRx
{
public:
    IrRemoteRx();
    ~IrRemoteRx();

    void begin(gpio_num_t rx_gpio_num, uint32_t resolution_hz);

    bool receiveNEC(void (*data_callback)(rmt_symbol_word_t*, size_t), TickType_t timeout);

private:
    QueueHandle_t _receive_queue;
    rmt_channel_handle_t _channel;
    rmt_symbol_word_t _symbols_buffer[IR_RX_SYMBOL_BUF_SIZE];
    rmt_receive_config_t _receive_config;
};
