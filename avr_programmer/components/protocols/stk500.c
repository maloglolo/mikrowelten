#include "stk500.h"
#include "programmer.h"
#include "ringbuffer.h"

#include "driver/uart.h"
#include <string.h>

#define RX_RING_SIZE 512
#define FRAME_MAX_SIZE 260

#define STK_START 0x1B
#define STK_END   0x20

#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_INSYNC  0x14

#define STK_GET_SYNC       0x30
#define STK_GET_SIGN_ON    0x31
#define STK_ENTER_PROGMODE 0x50
#define STK_LEAVE_PROGMODE 0x51
#define STK_LOAD_ADDRESS   0x55
#define STK_PROG_PAGE      0x64
#define STK_READ_PAGE      0x74

static ring_buffer_t rx_ring;
static uint8_t rx_buffer[RX_RING_SIZE];
static uint16_t current_address = 0;

// ----------------------------
// UART helpers
// ----------------------------
static void send_status(uint8_t ok) {
    uint8_t msg[2] = { STK_INSYNC, ok ? STK_OK : STK_FAILED };
    uart_write_bytes(UART_NUM_0, (const char*)msg, sizeof(msg));
}

static void send_bytes(const uint8_t *data, size_t len) {
    uart_write_bytes(UART_NUM_0, (const char*)data, len);
}

// ----------------------------
// Frame handling
// ----------------------------
static void handle_frame(const uint8_t *frame, size_t len) {
    if(len < 4) {
        send_status(0); // too short to be valid
        return;
    }

    uint8_t cmd = frame[0];
    uint16_t payload_len = ((uint16_t)frame[1] << 8) | frame[2];
    size_t expected_len = 1 /*cmd*/ + 2 /*lenH+lenL*/ + payload_len + 1 /*checksum*/;
    if(len != expected_len) {
        send_status(0); // length mismatch
        return;
    }

    // Checksum: XOR of CMD + LEN_H + LEN_L + PAYLOAD
    uint8_t expected_csum = frame[len-1];
    uint8_t actual_csum = 0;
    for(size_t i = 0; i < len-1; i++) actual_csum ^= frame[i];
    if(expected_csum != actual_csum) {
        send_status(0); // bad checksum
        return;
    }

    const uint8_t *payload = payload_len ? &frame[3] : NULL;

    switch(cmd) {
        case STK_GET_SYNC:
            send_status(1);
            break;

        case STK_GET_SIGN_ON: {
            uint8_t resp[] = { STK_INSYNC, 'A', 'V', 'R', STK_OK };
            send_bytes(resp, sizeof(resp));
            break;
        }

        case STK_ENTER_PROGMODE:
            programmer_enter();
            send_status(1);
            break;

        case STK_LEAVE_PROGMODE:
            programmer_leave();
            send_status(1);
            break;

        case STK_LOAD_ADDRESS:
            if(payload_len == 2) {
                current_address = (payload[0] << 8) | payload[1];
                send_status(1);
            } else {
                send_status(0);
            }
            break;

        case STK_PROG_PAGE:
            if(payload_len > 0 && payload) {
                programmer_prog_page(current_address, payload, payload_len);
                current_address += payload_len / 2; // increment by words
                send_status(1);
            } else {
                send_status(0);
            }
            break;

        case STK_READ_PAGE:
            if(payload_len > 0 && payload) {
                programmer_read_page(current_address, payload, payload_len);
                current_address += payload_len / 2;
                uint8_t resp[2 + payload_len + 1];
                resp[0] = STK_INSYNC;
                memcpy(&resp[1], payload, payload_len);
                resp[1 + payload_len] = STK_OK;
                send_bytes(resp, sizeof(resp));
            } else {
                send_status(0);
            }
            break;

        default:
            send_status(0); // unknown command
            break;
    }
}

// ----------------------------
// Ring buffer processing
// ----------------------------
static void process_ring(void) {
    static uint8_t frame[FRAME_MAX_SIZE];
    static size_t frame_len = 0;

    while(!ring_buffer_empty(&rx_ring)) {
        uint8_t b;
        ring_buffer_pop(&rx_ring, &b);

        if(frame_len == 0 && b != STK_START) continue;

        frame[frame_len++] = b;

        if(b == STK_END) {
            // Exclude STK_START and STK_END
            if(frame_len >= 2) handle_frame(&frame[1], frame_len - 2);
            frame_len = 0;
        } else if(frame_len >= FRAME_MAX_SIZE) {
            frame_len = 0; // reset on overflow
        }
    }
}

// ----------------------------
// Public API
// ----------------------------
void stk500_init(void) {
    ring_buffer_init(&rx_ring, rx_buffer, RX_RING_SIZE);
    current_address = 0;
}

void stk500_reset(void) {
    current_address = 0;
    ring_buffer_init(&rx_ring, rx_buffer, RX_RING_SIZE);
}

void stk500_on_byte(uint8_t byte) {
    if(!ring_buffer_push(&rx_ring, byte)) {
        uint8_t discard;
        ring_buffer_pop(&rx_ring, &discard);
        ring_buffer_push(&rx_ring, byte);
    }
    process_ring();
}
