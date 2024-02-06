#include "IrRemote.h"

#include "esp_log.h"
static const char *TAG = "IrRemote";

#define IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us

/**
 * @brief Saving NEC decode results
 */
static ir_nec_scan_code_t s_nec_code;

// NEC protocol example data
const ir_nec_scan_code_t scan_code = {
    .address = 0x0440,
    .command = 0x3003,
};

/**
 * @brief Decode RMT symbols into NEC scan code and print the result
 */
static void example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
    printf("NEC frame start---\r\n");
    for (size_t i = 0; i < symbol_num; i++)
    {
        printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
               rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    printf("---NEC frame end: ");
    // decode RMT symbols
    switch (symbol_num)
    {
    case 34: // NEC normal frame
    {

        s_nec_code = nec_parse_frame(rmt_nec_symbols);
        printf("Address=%04X, Command=%04X\r\n\r\n", s_nec_code.address, s_nec_code.command);
        break;
    }
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols))
        {
            printf("Address=%04X, Command=%04X, repeat\r\n\r\n", s_nec_code.address, s_nec_code.command);
        }
        break;
    default:
        printf("Unknown NEC frame\r\n\r\n");
        break;
    }
}

IrRemote::IrRemote()
    : remoteTx(), remoteRx() {}

IrRemote::~IrRemote() {}

void IrRemote::begin(int txPin, int rxPin)
{
    _txPin = (gpio_num_t)txPin;
    _rxPin = (gpio_num_t)rxPin;

    if (_txPin >= 0)
        remoteTx.begin(_txPin, IR_RESOLUTION_HZ);
    if (_rxPin >= 0)
        remoteRx.begin(_rxPin, IR_RESOLUTION_HZ);
}

bool IrRemote::testReadIrCode(uint32_t timeout_ms)
{
    return remoteRx.receiveNEC(example_parse_nec_frame, pdMS_TO_TICKS(timeout_ms));
}

bool IrRemote::testSendIrCode(void)
{
    return remoteTx.sendNEC(scan_code.address, scan_code.command);
}
