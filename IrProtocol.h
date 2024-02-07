#pragma once

#include <stdint.h>

enum ir_remote_protocol
{
    IR_NEC = 0,
    IR_SONY,
};

typedef struct
{
    uint16_t address;
    uint16_t command;
} ir_code_t;
