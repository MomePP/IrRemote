#include "Ir-Data.h"
#include "esp_log.h"

static const char *TAG = "data_codec";

static inline uint16_t data_ticks(uint64_t us, uint32_t resolution)
{
    return (uint16_t)(us * resolution / 1000000ULL);
}

uint8_t data_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ IR_DATA_CRC8_POLY) : (uint8_t)(crc << 1);
    }
    return crc;
}

// CRC-8 over the wire-format prefix [LEN, payload...] — the bytes the frame protects.
static uint8_t data_frame_crc(uint8_t len, const uint8_t *payload)
{
    uint8_t buf[1 + IR_DATA_MAX_PAYLOAD];
    buf[0] = len;
    for (uint8_t i = 0; i < len; i++)
        buf[1 + i] = payload[i];
    return data_crc8(buf, (size_t)(1 + len));
}

// Emit one byte as 8 pulse-distance bit-symbols (LSB-first) into out[idx..]; returns new idx.
static size_t data_emit_byte(rmt_symbol_word_t *out, size_t idx, uint8_t value, uint32_t resolution)
{
    for (int i = 0; i < 8; i++)
    {
        bool one = (value >> i) & 0x1;
        out[idx++] = (rmt_symbol_word_t){
            .duration0 = data_ticks(IR_DATA_BIT_MARK, resolution),
            .level0    = 1,
            .duration1 = data_ticks(one ? IR_DATA_ONE_SPACE : IR_DATA_ZERO_SPACE, resolution),
            .level1    = 0,
        };
    }
    return idx;
}

size_t data_build_frame(rmt_symbol_word_t *out, const uint8_t *payload, uint8_t len, uint32_t resolution)
{
    if (len > IR_DATA_MAX_PAYLOAD)
        return 0;

    uint8_t crc = data_frame_crc(len, payload);

    size_t idx = 0;
    // leader
    out[idx++] = (rmt_symbol_word_t){
        .duration0 = data_ticks(IR_DATA_LEADER_MARK, resolution),
        .level0    = 1,
        .duration1 = data_ticks(IR_DATA_LEADER_SPACE, resolution),
        .level1    = 0,
    };
    idx = data_emit_byte(out, idx, len, resolution);          // LEN
    for (uint8_t i = 0; i < len; i++)                          // PAYLOAD
        idx = data_emit_byte(out, idx, payload[i], resolution);
    idx = data_emit_byte(out, idx, crc, resolution);          // CRC8
    // stop mark; the trailing idle gap (duration1 max) terminates the frame
    out[idx++] = (rmt_symbol_word_t){
        .duration0 = data_ticks(IR_DATA_STOP_MARK, resolution),
        .level0    = 1,
        .duration1 = IR_DATA_STOP_IDLE_TICKS,
        .level1    = 0,
    };
    return idx;
}

static inline bool data_in_range(uint32_t dur, uint32_t spec)
{
    uint32_t lo = (spec > IR_DATA_DECODE_MARGIN) ? (spec - IR_DATA_DECODE_MARGIN) : 0;
    return (dur < spec + IR_DATA_DECODE_MARGIN) && (dur > lo);
}

// Decode 8 pulse-distance bits (LSB-first) starting at symbols[idx]; *ok=false on a bad bit.
static uint8_t data_read_byte(const rmt_symbol_word_t *symbols, size_t idx, bool *ok)
{
    uint8_t v = 0;
    for (int i = 0; i < 8; i++)
    {
        uint32_t space = symbols[idx + i].duration1;
        if (data_in_range(space, IR_DATA_ONE_SPACE))
            v |= (1u << i);
        else if (!data_in_range(space, IR_DATA_ZERO_SPACE))
        {
            *ok = false;
            return 0;
        }
    }
    *ok = true;
    return v;
}

int data_decode_frame(const rmt_symbol_word_t *symbols, size_t symbol_num, uint8_t *out)
{
    ESP_LOGD(TAG, "data frame start--- num=%u", (unsigned)symbol_num);

    if (symbol_num < (size_t)IR_DATA_FRAME_SYMBOLS(0)) // leader + LEN + CRC + stop
        return -1;
    if (!data_in_range(symbols[0].duration0, IR_DATA_LEADER_MARK))
        return -1;

    bool ok;
    uint8_t len = data_read_byte(symbols, 1, &ok);
    if (!ok || len > IR_DATA_MAX_PAYLOAD)
        return -1;
    if (symbol_num != (size_t)IR_DATA_FRAME_SYMBOLS(len))
        return -1;

    for (uint8_t i = 0; i < len; i++)
    {
        out[i] = data_read_byte(symbols, (size_t)(1 + (1 + i) * 8), &ok);
        if (!ok)
            return -1;
    }
    uint8_t rx_crc = data_read_byte(symbols, (size_t)(1 + (1 + len) * 8), &ok);
    if (!ok)
        return -1;

    if (data_frame_crc(len, out) != rx_crc)
    {
        ESP_LOGD(TAG, "data frame CRC mismatch");
        return -1;
    }
    ESP_LOGD(TAG, "data frame ok, len=%u", (unsigned)len);
    return (int)len;
}
