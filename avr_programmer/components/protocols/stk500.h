// stk500.h
#pragma once
#include <stdint.h>
#include "ringbuffer.h"

#define FRAME_MAX_SIZE 280

void stk500_feed(void);
void stk500_init(ring_buffer_t *external_ring);
void stk500_reset(void);
void stk500_on_byte(uint8_t byte);
