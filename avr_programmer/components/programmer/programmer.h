#pragma once
#include <stdint.h>
#include <stddef.h>

void programmer_init(void);
int programmer_enter(void);
int programmer_leave(void);

uint8_t programmer_read_signature(uint8_t index);
int programmer_load_address(uint16_t addr);
int programmer_prog_page(uint16_t addr, const uint8_t *data, size_t len);
int programmer_read_page(uint16_t addr, uint8_t *data, size_t len);
