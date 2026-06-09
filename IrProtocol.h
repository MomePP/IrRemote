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
