#include "role_select.h"
#if IS_MASTER

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#define TAG "RS485_MASTER"
#define BUF_SIZE 256
#define START_BYTE 0xAA
#define CMD_PING 0x01
#define CMD_PONG 0x02
#define POLL_INTERVAL_MS 500
#define PACKET_READ_TICKS (200 / portTICK_PERIOD_MS)
#define READ_TOUT 3

#define UART_PORT ((uart_port_t)CONFIG_RS485_UART_PORT)
#define UART_TX_PIN CONFIG_RS485_UART_TX
#define UART_RX_PIN CONFIG_RS485_UART_RX
#define UART_DE_PIN CONFIG_RS485_UART_DE
#define UART_RE_PIN CONFIG_RS485_UART_RE 
#define BAUD_RATE CONFIG_RS485_BAUD_RATE

static uint8_t checksum(const uint8_t *b, size_t len) {
  uint8_t s = 0;
  for (size_t i = 0; i < len; ++i)
    s += b[i];
  return s;
}

static void build_frame(uint8_t *out, uint8_t addr, uint8_t cmd,
                        const uint8_t *data, uint8_t len, int *out_len) {
  out[0] = START_BYTE;
  out[1] = addr;
  out[2] = cmd;
  out[3] = len;
  if (len && data)
    memcpy(&out[4], data, len);
  out[4 + len] = checksum(&out[1], 3 + len);
  *out_len = 5 + len;
}

void app_main(void) {
    ESP_LOGI(TAG, "Master starting");

    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup DE/RE pins as GPIO
    gpio_reset_pin(UART_DE_PIN);
    gpio_set_direction(UART_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(UART_DE_PIN, 0); // Idle: receive

    gpio_reset_pin(UART_RE_PIN);
    gpio_set_direction(UART_RE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(UART_RE_PIN, 0); // Idle: receive

    uint8_t txbuf[BUF_SIZE];
    uint8_t rxbuf[BUF_SIZE];

    const int N_SLAVES = 3;

    while (1) {
        for (int addr = 1; addr <= N_SLAVES; ++addr) {
            int txlen = 0;
            build_frame(txbuf, (uint8_t)addr, CMD_PING, NULL, 0, &txlen);

            // Enable transmit
            gpio_set_level(UART_DE_PIN, 1);
            gpio_set_level(UART_RE_PIN, 1);

            ESP_LOGI(TAG, "PING -> addr=%d", addr);
            uart_write_bytes(UART_PORT, (const char *)txbuf, txlen);
            ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT, 50 / portTICK_PERIOD_MS));

            // Back to receive
            gpio_set_level(UART_DE_PIN, 0);
            gpio_set_level(UART_RE_PIN, 0);

            int len = uart_read_bytes(UART_PORT, rxbuf, sizeof(rxbuf), PACKET_READ_TICKS);
            if (len >= 5 && rxbuf[0] == START_BYTE) {
                uint8_t r_addr = rxbuf[1];
                uint8_t r_cmd  = rxbuf[2];
                uint8_t r_len  = rxbuf[3];
                uint8_t r_cs   = rxbuf[4 + r_len];

            if (checksum(&rxbuf[1], 3 + r_len) == r_cs && r_cmd == CMD_PONG) {
                ESP_LOGI(TAG, "PONG from %d", r_addr);

                if (r_len >= 1) { // 1 byte moisture
                    uint8_t moist = rxbuf[5];
                    ESP_LOGI(TAG, "Soil moisture from %d: %d %%", r_addr, moist);
                }
            }

            }
            vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        }
    }
}


#endif // IS_MASTER
