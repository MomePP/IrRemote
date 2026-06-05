#pragma once

#include <stdint.h>

enum ir_remote_protocol
{
    IR_AUTO = 0, // auto-detect by RMT symbol count (default)
    IR_NEC,      // 1
    IR_SONY,     // 2
};

typedef struct
{
    uint16_t address;
    uint16_t command;
} ir_code_t;
