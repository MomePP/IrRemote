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

    // receiveNEC returns true for any frame it dequeued (a malformed frame yields {0,0}).
    // receiveSony returns true only for a valid Sony frame; a frame that is not one is consumed
    // silently, like a timeout.
    bool receiveNEC(ir_code_t *received_code, TickType_t timeout);
    bool receiveSony(ir_code_t *received_code, TickType_t timeout);

    // Auto-detect: dequeue one frame from the shared window and dispatch by RMT
    // symbol count. Writes *detected (IR_NEC / IR_SONY) for the frame's actual
    // protocol. A 13-symbol burst that is not a valid Sony frame, or an unrecognised symbol count,
    // returns false and leaves *detected unchanged.
    bool receiveAuto(ir_code_t *received_code, ir_remote_protocol *detected, TickType_t timeout);

    // Dequeue one frame from the shared window and decode it as an IR_DATA frame.
    // On success writes up to IR_DATA_MAX_PAYLOAD bytes to `out`, sets *out_len, returns
    // true. Returns false on timeout, a garbled frame, or a CRC mismatch.
    bool receiveData(uint8_t *out, uint8_t *out_len, TickType_t timeout);

    // Like receiveData, but a full 34-symbol NEC remote frame that fails to decode as data is
    // decoded as NEC and returned as IR_DATA_RX_NEC_CODE in *nec_code, so a remote control still
    // works while the board is in data mode. NEC repeat frames and a {0,0} decode (indistinguishable
    // from a garbled frame) are reported as IR_DATA_RX_NONE.
    ir_data_rx_result receiveDataOrNec(uint8_t *out, uint8_t *out_len, ir_code_t *nec_code, TickType_t timeout);

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
