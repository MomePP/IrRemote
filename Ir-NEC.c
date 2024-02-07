#include "Ir-NEC.h"
#include "esp_check.h"

static const char *TAG = "nec_codec";

typedef struct
{
    rmt_encoder_t base;                   // the base "class", declares the standard encoder interface
    rmt_encoder_t *copy_encoder;          // use the copy_encoder to encode the leading and ending pulse
    rmt_encoder_t *bytes_encoder;         // use the bytes_encoder to encode the address and command data
    rmt_symbol_word_t nec_leading_symbol; // NEC leading code with RMT representation
    rmt_symbol_word_t nec_ending_symbol;  // NEC ending code with RMT representation
    int state;
} rmt_ir_nec_encoder_t;

static size_t rmt_encode_ir_nec(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_ir_nec_encoder_t *nec_encoder = __containerof(encoder, rmt_ir_nec_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    ir_code_t *scan_code = (ir_code_t *)primary_data;
    rmt_encoder_handle_t copy_encoder = nec_encoder->copy_encoder;
    rmt_encoder_handle_t bytes_encoder = nec_encoder->bytes_encoder;
    switch (nec_encoder->state)
    {
    case 0: // send leading code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &nec_encoder->nec_leading_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            nec_encoder->state = 1; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 1: // send address
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &scan_code->address, sizeof(uint16_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            nec_encoder->state = 2; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 2: // send command
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &scan_code->command, sizeof(uint16_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            nec_encoder->state = 3; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 3: // send ending code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &nec_encoder->nec_ending_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            nec_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_ir_nec_encoder(rmt_encoder_t *encoder)
{
    rmt_ir_nec_encoder_t *nec_encoder = __containerof(encoder, rmt_ir_nec_encoder_t, base);
    rmt_del_encoder(nec_encoder->copy_encoder);
    rmt_del_encoder(nec_encoder->bytes_encoder);
    free(nec_encoder);
    return ESP_OK;
}

static esp_err_t rmt_ir_nec_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_ir_nec_encoder_t *nec_encoder = __containerof(encoder, rmt_ir_nec_encoder_t, base);
    rmt_encoder_reset(nec_encoder->copy_encoder);
    rmt_encoder_reset(nec_encoder->bytes_encoder);
    nec_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_ir_nec_encoder(const ir_nec_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_ir_nec_encoder_t *nec_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    nec_encoder = calloc(1, sizeof(rmt_ir_nec_encoder_t));
    ESP_GOTO_ON_FALSE(nec_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for ir nec encoder");
    nec_encoder->base.encode = rmt_encode_ir_nec;
    nec_encoder->base.del = rmt_del_ir_nec_encoder;
    nec_encoder->base.reset = rmt_ir_nec_encoder_reset;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &nec_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    // construct the leading code and ending code with RMT symbol format
    nec_encoder->nec_leading_symbol = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = NEC_LEADING_CODE_DURATION_0 * config->resolution / 1000000,
        .level1 = 0,
        .duration1 = NEC_LEADING_CODE_DURATION_1 * config->resolution / 1000000,
    };
    nec_encoder->nec_ending_symbol = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = NEC_ENDING_ZERO_DURATION_0 * config->resolution / 1000000,
        .level1 = 0,
        .duration1 = 0x7FFF,
    };

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = NEC_PAYLOAD_ZERO_DURATION_0 * config->resolution / 1000000, // T0H=560us
            .level1 = 0,
            .duration1 = NEC_PAYLOAD_ZERO_DURATION_1 * config->resolution / 1000000, // T0L=560us
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = NEC_PAYLOAD_ONE_DURATION_0 * config->resolution / 1000000, // T1H=560us
            .level1 = 0,
            .duration1 = NEC_PAYLOAD_ONE_DURATION_1 * config->resolution / 1000000, // T1L=1690us
        },
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &nec_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");

    *ret_encoder = &nec_encoder->base;
    return ESP_OK;
err:
    if (nec_encoder)
    {
        if (nec_encoder->bytes_encoder)
        {
            rmt_del_encoder(nec_encoder->bytes_encoder);
        }
        if (nec_encoder->copy_encoder)
        {
            rmt_del_encoder(nec_encoder->copy_encoder);
        }
        free(nec_encoder);
    }
    return ret;
}

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

static ir_code_t nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols)
{
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code)
    {
        return (ir_code_t){0, 0}; // return empty result
    }
    cur++;
    for (int i = 0; i < 16; i++)
    {
        if (nec_parse_logic1(cur))
        {
            address |= 1 << i;
        }
        else if (nec_parse_logic0(cur))
        {
            address &= ~(1 << i);
        }
        else
        {
            return (ir_code_t){0, 0}; // return empty result
        }
        cur++;
    }
    for (int i = 0; i < 16; i++)
    {
        if (nec_parse_logic1(cur))
        {
            command |= 1 << i;
        }
        else if (nec_parse_logic0(cur))
        {
            command &= ~(1 << i);
        }
        else
        {
            return (ir_code_t){0, 0}; // return empty result
        }
        cur++;
    }
    return (ir_code_t){address, command};
}

static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

ir_code_t nec_decode_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
    ESP_LOGD(TAG, "NEC frame start---");
    for (size_t i = 0; i < symbol_num; i++)
    {
        ESP_LOGD(TAG, "{%d:%d},{%d:%d}", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
                 rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    ESP_LOGD(TAG, "---NEC frame end");

    // decode RMT symbols
    switch (symbol_num)
    {
    case 34: // NEC normal frame
    {
        s_nec_code = nec_parse_frame(rmt_nec_symbols);
        ESP_LOGD(TAG, "Address=%04X, Command=%04X\r\n\r\n", s_nec_code.address, s_nec_code.command);
        break;
    }
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols))
        {
            ESP_LOGD(TAG, "Address=%04X, Command=%04X, repeat\r\n\r\n", s_nec_code.address, s_nec_code.command);
        }
        break;
    default:
        s_nec_code = (ir_code_t){0, 0};
        ESP_LOGD(TAG, "Unknown NEC frame\r\n\r\n");
        break;
    }
    return s_nec_code;
}
