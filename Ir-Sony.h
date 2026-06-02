#pragma once

#include "IrProtocol.h"
#include "driver/rmt_encoder.h"

/**
 * @brief Sony SIRC 12-bit timing spec (base unit T = 600us)
 *        Pulse-width coding, 40kHz carrier, LSB-first: 7 command bits + 5 address bits.
 */
#define SONY_SIRC_UNIT_DURATION 600ULL  // 1T
#define SONY_HEADER_MARK        2400ULL // 4T header mark
#define SONY_BIT_SPACE          600ULL  // 1T space trailing every symbol
#define SONY_BIT_ONE_MARK       1200ULL // 2T mark = logic 1
#define SONY_BIT_ZERO_MARK      600ULL  // 1T mark = logic 0

#define SONY_FRAME_BITS    12 // 7 command + 5 address
#define SONY_COMMAND_BITS  7
#define SONY_ADDRESS_BITS  5
#define SONY_COMMAND_MASK  0x7F // (1 << SONY_COMMAND_BITS) - 1
#define SONY_ADDRESS_MASK  0x1F // (1 << SONY_ADDRESS_BITS) - 1

#define SONY_FRAME_PERIOD_MS 45 // inter-repeat spacing
#define SONY_REPEAT_COUNT    3  // SIRC requires >= 3 frames per key press

#define IR_SONY_DECODE_MARGIN   200 // RMT duration tolerance (cf. IR_NEC_DECODE_MARGIN)
#define IR_SONY_RX_SYMBOL_COUNT 13  // 1 header + 12 bit symbols

#ifdef __cplusplus
extern "C"
{
#endif

    static ir_code_t s_sony_code;

    /**
     * @brief Build a 13-symbol Sony SIRC frame into an RMT symbol buffer.
     *
     * @param[out] out        Buffer of at least IR_SONY_RX_SYMBOL_COUNT symbols
     * @param[in]  code       address (5-bit) + command (7-bit) to encode
     * @param[in]  resolution RMT tick resolution in Hz (e.g. 1000000 for 1us ticks)
     */
    void sony_build_frame(rmt_symbol_word_t *out, ir_code_t code, uint32_t resolution);

    /**
     * @brief Decode RMT symbols into a Sony SIRC scan code.
     *        Returns {0,0} on malformed/unknown frames.
     */
    ir_code_t sony_decode_frame(rmt_symbol_word_t *rmt_sony_symbols, size_t symbol_num);

#ifdef __cplusplus
}
#endif
