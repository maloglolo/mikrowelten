#include "rotary.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

// Rotary pins from Kconfig
#define ROTARY_CLK CONFIG_ROTARY_PIN_CLK
#define ROTARY_DT  CONFIG_ROTARY_PIN_DT

static volatile int contrast = 128;

void IRAM_ATTR rotary_isr(void *arg) {
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
