#pragma once
#include <stdint.h>
#include <stddef.h>

void stk500_init(void);
void stk500_on_byte(uint8_t byte);
void stk500_reset(void);
