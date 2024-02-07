#pragma once

#include "IrProtocol.h"
#include "driver/rmt_encoder.h"

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0 9000ULL
#define NEC_LEADING_CODE_DURATION_1 4500ULL
#define NEC_ENDING_ZERO_DURATION_0  560ULL
#define NEC_PAYLOAD_ZERO_DURATION_0 560ULL
#define NEC_PAYLOAD_ZERO_DURATION_1 560ULL
#define NEC_PAYLOAD_ONE_DURATION_0  560ULL
#define NEC_PAYLOAD_ONE_DURATION_1  1690ULL
#define NEC_REPEAT_CODE_DURATION_0  9000ULL
#define NEC_REPEAT_CODE_DURATION_1  2250ULL

#define IR_NEC_DECODE_MARGIN        200      // Tolerance for parsing RMT symbols into bit stream
#define IR_NEC_SIGNAL_RANGE_MIN     1250     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
#define IR_NEC_SIGNAL_RANGE_MAX     12000000 // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early

#ifdef __cplusplus
extern "C"
{
#endif

    static ir_code_t s_nec_code;

    /**
     * @brief Type of IR NEC encoder configuration
     */
    typedef struct
    {
        uint32_t resolution; /*!< Encoder resolution, in Hz */
    } ir_nec_encoder_config_t;

    /**
     * @brief Create RMT encoder for encoding IR NEC frame into RMT symbols
     *
     * @param[in] config Encoder configuration
     * @param[out] ret_encoder Returned encoder handle
     * @return
     *      - ESP_ERR_INVALID_ARG for any invalid arguments
     *      - ESP_ERR_NO_MEM out of memory when creating IR NEC encoder
     *      - ESP_OK if creating encoder successfully
     */
    esp_err_t rmt_new_ir_nec_encoder(const ir_nec_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

    /**
     * @brief Decode RMT symbols into NEC scan code and print the result
     */
    ir_code_t nec_decode_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num);

#ifdef __cplusplus
}
#endif
