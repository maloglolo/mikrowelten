// u8g2_esp32_hal_spi_only.c
// ------------------------
// SPI-only HAL for U8g2 on ESP32.
// Provides a bridge between the hardware-independent U8g2 library
// and ESP32 SPI/GPIO interfaces.
// Heavily inspired by https://github.com/nkolban/esp32-snippets/tree/master/hardware/displays/U8G2
//
// Resources:
//  - U8g2 library: https://github.com/olikraus/u8g2
//  - ESP32 SPI API: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html
//  - Hardware Abstraction Layer concept: https://en.wikipedia.org/wiki/Hardware_abstraction_layer
//
// TODO: Refactor ESP32 U8g2 HAL
// - HAL struct: add I2C pins, optional flags, group pins logically.
// - Init: move SPI/I2C setup out of callbacks, validate pins, use bitmask for GPIOs.
// - Callbacks: minimal logic, use HAL struct, handle optional pins, separate GPIO/delay.
// - Extensibility: support multiple devices, new protocols, optional custom byte/delay funcs.
// - Docs: add inline explanations, usage example, link ESP32 SPI/I2C & U8g2.
//
// Notes: HAL decouples library from hardware; struct-based config; callbacks implement board-specific behavior; separation of concerns; scalable GPIO init.
//

#include "u8g2_esp32_hal.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "u8g2_hal";
static u8g2_esp32_hal_t u8g2_esp32_hal;
static spi_device_handle_t handle_spi = NULL;

// -----------------------------------------------------------------------------
// Initialize HAL (SPI only)
// -----------------------------------------------------------------------------
void u8g2_esp32_hal_init(u8g2_esp32_hal_t *hal_param) {
    u8g2_esp32_hal = *hal_param; // copy config

    ESP_LOGI(TAG, "HAL init: CLK=%d, MOSI=%d, CS=%d, DC=%d, RST=%d",
             u8g2_esp32_hal.clk,
             u8g2_esp32_hal.mosi,
             u8g2_esp32_hal.cs,
             u8g2_esp32_hal.dc,
             u8g2_esp32_hal.reset);

    // --- SPI bus ---
    spi_bus_config_t buscfg = {
        .sclk_io_num = hal_param->clk,
        .mosi_io_num = hal_param->mosi,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128 * 64 / 8 + 8
    };
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // --- SPI device ---
    if (hal_param->cs == U8G2_ESP32_HAL_UNDEFINED) {
        ESP_LOGE(TAG, "CS pin must be defined for SPI!");
        abort();
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1, // let u8g2 toggle CS manually via GPIO
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY
    };
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &handle_spi));

    ESP_LOGI(TAG, "SPI device added on VSPI, manual CS via GPIO %d",
             hal_param->cs);

    // --- GPIO init (DC, RESET, CS) ---
    uint64_t bitmask = 0;
    if (hal_param->dc != U8G2_ESP32_HAL_UNDEFINED) bitmask |= (1ull << hal_param->dc);
    if (hal_param->reset != U8G2_ESP32_HAL_UNDEFINED) bitmask |= (1ull << hal_param->reset);
    if (hal_param->cs != U8G2_ESP32_HAL_UNDEFINED) bitmask |= (1ull << hal_param->cs);

    if (bitmask) {
        gpio_config_t cfg = {
            .pin_bit_mask = bitmask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&cfg);
    }
}

// -----------------------------------------------------------------------------
// SPI byte callback for U8G2
// -----------------------------------------------------------------------------
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_SET_DC:
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED)
                gpio_set_level(u8g2_esp32_hal.dc, arg_int);
            break;

        case U8X8_MSG_BYTE_SEND: {
            spi_transaction_t t = {
                .length = arg_int * 8,
                .tx_buffer = arg_ptr
            };
            esp_err_t err = spi_device_transmit(handle_spi, &t);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "spi_device_transmit failed: %d", err);
            }
            break;
        }
    }
    return 0;
}

// -----------------------------------------------------------------------------
// GPIO + delay callback
// -----------------------------------------------------------------------------
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8,
                                     uint8_t msg,
                                     uint8_t arg_int,
                                     void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;
        case U8X8_MSG_GPIO_RESET:
            if(u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED)
                gpio_set_level(u8g2_esp32_hal.reset, arg_int);
            break;
        case U8X8_MSG_GPIO_CS:
            if(u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED)
                gpio_set_level(u8g2_esp32_hal.cs, arg_int);
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            if(u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED)
                gpio_set_level(u8g2_esp32_hal.cs, 0);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            if(u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED)
                gpio_set_level(u8g2_esp32_hal.cs, 1);
            break;
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
    }
    return 0;
}
