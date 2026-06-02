#include "Ir-Sony.h"
#include "esp_log.h"

static const char *TAG = "sony_codec";

// Holds the most recently decoded Sony scan code. File-scope (not in the header)
// so each translation unit does not get its own private copy.
static ir_code_t s_sony_code;

static inline uint16_t sony_ticks(uint64_t us, uint32_t resolution)
{
    return (uint16_t)(us * resolution / 1000000ULL);
}

void sony_build_frame(rmt_symbol_word_t *out, ir_code_t code, uint32_t resolution)
{
    // header: 2400us mark + 600us space
    out[0] = (rmt_symbol_word_t){
        .duration0 = sony_ticks(SONY_HEADER_MARK, resolution),
        .level0    = 1,
        .duration1 = sony_ticks(SONY_BIT_SPACE, resolution),
        .level1    = 0,
    };

    // 12 data bits, LSB-first: command (7) in bits [0..6], address (5) in bits [7..11]
    uint16_t bits = (uint16_t)((code.command & SONY_COMMAND_MASK) |
                               ((code.address & SONY_ADDRESS_MASK) << SONY_COMMAND_BITS));
    for (int i = 0; i < SONY_FRAME_BITS; i++)
    {
        bool one = (bits >> i) & 0x1;
        out[1 + i] = (rmt_symbol_word_t){
            .duration0 = sony_ticks(one ? SONY_BIT_ONE_MARK : SONY_BIT_ZERO_MARK, resolution),
            .level0    = 1,
            .duration1 = sony_ticks(SONY_BIT_SPACE, resolution),
            .level1    = 0,
        };
    }
}

static inline bool sony_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + IR_SONY_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - IR_SONY_DECODE_MARGIN));
}

ir_code_t sony_decode_frame(rmt_symbol_word_t *rmt_sony_symbols, size_t symbol_num)
{
    ESP_LOGD(TAG, "Sony frame start--- num=%u", (unsigned)symbol_num);
    for (size_t i = 0; i < symbol_num; i++)
    {
        ESP_LOGD(TAG, "{%d:%d},{%d:%d}", rmt_sony_symbols[i].level0, rmt_sony_symbols[i].duration0,
                 rmt_sony_symbols[i].level1, rmt_sony_symbols[i].duration1);
    }

    if (symbol_num != IR_SONY_RX_SYMBOL_COUNT)
    {
        s_sony_code = (ir_code_t){0, 0};
        ESP_LOGD(TAG, "Unknown Sony frame");
        return s_sony_code;
    }

    // validate header mark
    if (!sony_check_in_range(rmt_sony_symbols[0].duration0, SONY_HEADER_MARK))
    {
        s_sony_code = (ir_code_t){0, 0};
        return s_sony_code;
    }

    uint16_t bits = 0;
    for (int i = 0; i < SONY_FRAME_BITS; i++)
    {
        uint32_t mark = rmt_sony_symbols[1 + i].duration0;
        if (sony_check_in_range(mark, SONY_BIT_ONE_MARK))
            bits |= (1u << i);
        else if (!sony_check_in_range(mark, SONY_BIT_ZERO_MARK))
        {
            s_sony_code = (ir_code_t){0, 0}; // neither a one nor a zero
            return s_sony_code;
        }
    }

    s_sony_code.command = bits & SONY_COMMAND_MASK;
    s_sony_code.address = (bits >> SONY_COMMAND_BITS) & SONY_ADDRESS_MASK;
    ESP_LOGD(TAG, "Address=%02X, Command=%02X", s_sony_code.address, s_sony_code.command);
    return s_sony_code;
}
