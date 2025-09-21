#include "ringbuffer.h"

void ring_buffer_init(ring_buffer_t *rb, uint8_t *buf, size_t size) {
    rb->buffer = buf;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

bool ring_buffer_push(ring_buffer_t *rb, uint8_t data) {
    size_t next = (rb->head + 1) % rb->size;
    if(next == rb->tail) {
        // Buffer full
        return false;
    }
    rb->buffer[rb->head] = data;
    rb->head = next;
    return true;
}

bool ring_buffer_pop(ring_buffer_t *rb, uint8_t *data) {
    if(rb->head == rb->tail) {
        // Buffer empty
        return false;
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    return true;
}

bool ring_buffer_empty(ring_buffer_t *rb) {
    return rb->head == rb->tail;
}

bool ring_buffer_full(ring_buffer_t *rb) {
    return ((rb->head + 1) % rb->size) == rb->tail;
}

size_t ring_buffer_count(ring_buffer_t *rb) {
    if(rb->head >= rb->tail) {
        return rb->head - rb->tail;
    }
    return rb->size - rb->tail + rb->head;
}
