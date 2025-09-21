#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;
    size_t head;
    size_t tail;
    size_t size;
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t *rb, uint8_t *buf, size_t size);
bool ring_buffer_push(ring_buffer_t *rb, uint8_t data);
bool ring_buffer_pop(ring_buffer_t *rb, uint8_t *data);
bool ring_buffer_empty(ring_buffer_t *rb);
bool ring_buffer_full(ring_buffer_t *rb);
size_t ring_buffer_count(ring_buffer_t *rb);
