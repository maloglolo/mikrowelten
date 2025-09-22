#include "stk500.h"
#include "ringbuffer.h"
#include "programmer.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_AVR       UART_NUM_1    // AVRdude
#define UART_LOG       UART_NUM_0    // ESP32 debug logs
#define RX_BUF_SIZE    1024
#define TX_BUF_SIZE    64

// ----------------------
// Ring buffer for AVR
// ----------------------
static ring_buffer_t rx_ring;
static uint8_t rx_buf[RX_BUF_SIZE];

// ----------------------
// Task to process AVR frames
// ----------------------
void stk500_task(void *arg) {
    while (1) {
        stk500_feed();                // Process any received bytes
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield CPU
    }
}

// ----------------------
// Main
// ----------------------
void app_main(void) {
    // ----------------------
    // UART for AVR communication
    // ----------------------
    esp_log_level_set("*", ESP_LOG_NONE);
    const uart_config_t uart_cfg_avr = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_AVR, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_AVR, &uart_cfg_avr);

    // ----------------------
    // UART for logging (USB to host)
    // ----------------------
    const uart_config_t uart_cfg_log = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_LOG, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_LOG, &uart_cfg_log);


    ring_buffer_init(&rx_ring, rx_buf, RX_BUF_SIZE);
    stk500_init(&rx_ring);
    programmer_init();

    // ----------------------
    // Start task to process STK500 frames
    // ----------------------
    xTaskCreate(stk500_task, "stk500_task", 4096, NULL, 10, NULL);

    // ----------------------
    // Read bytes from AVR UART
    // ----------------------
    uint8_t byte;
    while (1) {
        int len = uart_read_bytes(UART_AVR, &byte, 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            stk500_on_byte(byte);  // Push to STK500 ring buffer
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Avoid tight loop
    }
}
