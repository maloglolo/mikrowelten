/*
 * Rotary Encoder Module with ISR Logic (Quadrature Decoding)
 * -----------------------------------------------
 * A rotary encoder outputs two square waves (CLK=A, DT=B) shifted by 90°.
 * By tracking their *phase relationship* we can detect rotation direction.
 *
 * State Encoding (A,B → 2-bit value):
 *   00 = 0b00, 01 = 0b01, 11 = 0b11, 10 = 0b10
 *
 * Valid Gray-code Sequences:
 *   Clockwise  (CW)     : 00 → 01 → 11 → 10 → 00
 *   Counter-Clockwise (CCW): 00 → 10 → 11 → 01 → 00
 *
 * Detecting Direction:
 *   - Store last_state (2 bits)
 *   - Read current_state (2 bits)
 *   - Encode as 4-bit index: idx = (last_state << 2) | current_state
 *
 * Transition Table (idx → delta, binary encoding annotated):
 *   +1 = CW, -1 = CCW, 0 = invalid/no move
 *           current
 *      last         00(0) 01(1) 10(2) 11(3)
 *      -------------------------------------
 *           00(0)   0      +1     -1      0
 *           01(1)  -1       0      0     +1
 *           10(2)  +1       0      0     -1
 *           11(3)   0      -1     +1      0
 *
 * Usage:
 *   delta = transition_table[(last_state << 2) | current_state];
 *   counter += delta * STEP_SIZE;  // e.g., STEP_SIZE = 4
 *   last_state = current_state;
 *
 * Resources:
 *   - Gray code & quadrature encoders:
 *       https://en.wikipedia.org/wiki/Quadrature_encoder
 *   - Examples and discussions:
 *       https://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
 *        - http://web.archive.org/web/20250614170824/https://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
 *       https://www.best-microcontroller-projects.com/rotary-encoder.html
 *   - General encoder theory:
 *       https://www.pjrc.com/teensy/td_libs_Encoder.html
 */

#include "rotary.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

// Rotary pins from Kconfig
#define ROTARY_CLK CONFIG_ROTARY_PIN_CLK
#define ROTARY_DT  CONFIG_ROTARY_PIN_DT

static volatile int contrast = 128;

/* void IRAM_ATTR rotary_isr(void *arg) {
    static uint8_t last_state = 0;
    uint8_t current_state =
        (gpio_get_level(ROTARY_CLK) << 1) | gpio_get_level(ROTARY_DT);

    if ((last_state == 0b00 && current_state == 0b01) ||
        (last_state == 0b01 && current_state == 0b11) ||
        (last_state == 0b11 && current_state == 0b10) ||
        (last_state == 0b10 && current_state == 0b00)) {
        contrast += 4;
    } else if ((last_state == 0b00 && current_state == 0b10) ||
               (last_state == 0b10 && current_state == 0b11) ||
               (last_state == 0b11 && current_state == 0b01) ||
               (last_state == 0b01 && current_state == 0b00)) {
        contrast -= 4;
    }

    if (contrast < 0) contrast = 0;
    if (contrast > 255) contrast = 255;

    last_state = current_state;
}
*/

void IRAM_ATTR rotary_isr(void *arg) {
    static uint8_t last_state = 0;
    static const int8_t transition_table[16] = {
        0,  1, -1,  0,
       -1,  0,  0,  1,
        1,  0,  0, -1,
        0, -1,  1,  0
    };

    uint8_t current_state =
        (gpio_get_level(ROTARY_CLK) << 1) | gpio_get_level(ROTARY_DT);

    int8_t delta = transition_table[(last_state << 2) | current_state];
    contrast += delta * 4;

    if (contrast < 0) contrast = 0;
    if (contrast > 255) contrast = 255;

    last_state = current_state;
}

void rotary_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ROTARY_CLK) | (1ULL << ROTARY_DT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_set_intr_type(ROTARY_CLK, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ROTARY_DT, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(ROTARY_CLK, rotary_isr, NULL);
    gpio_isr_handler_add(ROTARY_DT, rotary_isr, NULL);
}

int rotary_get_contrast(void) {
    return contrast;
}
