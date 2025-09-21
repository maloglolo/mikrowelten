#include "programmer.h"
#include "stk500.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#define UART_PORT    UART_NUM_0
#define RX_BUF_SIZE  1024
#define TAG         "ESP32_STK500"

void app_main(void) {
    // UART configuration
    const uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Install UART driver
    uart_driver_install(UART_PORT, RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_cfg);

    // Initialize programmer and STK500
    programmer_init();
    stk500_init();
    stk500_reset();

    //ESP_LOGI(TAG, "STK500 programmer ready, waiting for UART bytes...");

    // Poll UART for incoming bytes
    uint8_t byte;
    while (1) {
        int len = uart_read_bytes(UART_PORT, &byte, 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            stk500_on_byte(byte); // Feed byte into STK500 parser
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // small delay to yield CPU
    }
}
