#pragma once

#include <stdint.h>

enum ir_remote_protocol
{
    IR_AUTO = 0, // auto-detect by RMT symbol count (default)
    IR_NEC,      // 1
    IR_SONY,     // 2
    IR_DATA,     // 3 — byte-data link mode (sendData/receiveData; distinct from remote codes)
};

typedef struct
{
    uint16_t address;
    uint16_t command;
} ir_code_t;

// Outcome of one IR_DATA-mode receive (IrRemoteRx::receiveDataOrNec).
enum ir_data_rx_result
{
    IR_DATA_RX_NONE = 0, // timeout, or a frame that is neither a data frame nor a NEC code
    IR_DATA_RX_FRAME,    // a data frame with a good CRC
    IR_DATA_RX_NEC_CODE, // a full NEC remote frame: the data link cannot carry it, so the code is handed over
};
