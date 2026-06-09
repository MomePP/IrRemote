#pragma once

#include "IrProtocol.h"
#include "driver/rmt_encoder.h"

/**
 * @brief IR data-link frame (pulse-distance, NEC-style bit timing, LSB-first).
 *        Frame layout: [LEADER] [LEN:1] [PAYLOAD:LEN] [CRC8:1] [STOP]
 *        CRC8 is taken over (LEN concatenated with PAYLOAD).
 *
 * ALL timing/size constants are PROVISIONAL (NEC-derived) — the IRM-H638J7 AGC class
 * governs the real max frame length / min mark / gap rules; characterize on the bench
 * (board-to-board loopback) and refine here. See the design spec + research note.
 */
#define IR_DATA_BIT_MARK      560ULL   // fixed mark per data bit
#define IR_DATA_ZERO_SPACE    560ULL   // short space => logic 0
#define IR_DATA_ONE_SPACE     1690ULL  // long space  => logic 1
#define IR_DATA_LEADER_MARK   4500ULL  // PROVISIONAL sync / AGC-settle (distinct from NEC 9000)
#define IR_DATA_LEADER_SPACE  4500ULL  // PROVISIONAL
#define IR_DATA_STOP_MARK     560ULL   // trailing mark; end-of-frame is the idle gap

#define IR_DATA_DECODE_MARGIN 250      // RMT duration tolerance (us)
#define IR_DATA_MAX_PAYLOAD   16       // PROVISIONAL, AGC-bounded; bench-tune
#define IR_DATA_CRC8_POLY     0x07     // CRC-8/CCITT

// Symbol count for a `len`-byte payload: leader(1) + (LEN + payload + CRC) * 8 + stop(1).
#define IR_DATA_FRAME_SYMBOLS(len) (1 + (2 + (len)) * 8 + 1)
#define IR_DATA_MAX_FRAME_SYMBOLS  IR_DATA_FRAME_SYMBOLS(IR_DATA_MAX_PAYLOAD) // 146

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief CRC-8/CCITT (poly IR_DATA_CRC8_POLY, init 0x00) over `len` bytes. */
    uint8_t data_crc8(const uint8_t *data, size_t len);

    /**
     * @brief Build a pulse-distance data frame into `out`.
     * @param[out] out        buffer of at least IR_DATA_MAX_FRAME_SYMBOLS symbols
     * @param[in]  payload    LEN bytes to send
     * @param[in]  len        payload length (0..IR_DATA_MAX_PAYLOAD)
     * @param[in]  resolution RMT tick resolution in Hz
     * @return number of symbols written, or 0 if len > IR_DATA_MAX_PAYLOAD.
     */
    size_t data_build_frame(rmt_symbol_word_t *out, const uint8_t *payload, uint8_t len, uint32_t resolution);

    /**
     * @brief Decode RMT symbols into payload bytes (writes up to IR_DATA_MAX_PAYLOAD to `out`).
     * @return payload length on success (CRC ok), or -1 on bad leader / length / bit / CRC.
     */
    int data_decode_frame(const rmt_symbol_word_t *symbols, size_t symbol_num, uint8_t *out);

#ifdef __cplusplus
}
#endif
