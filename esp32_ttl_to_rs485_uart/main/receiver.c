#include "role_select.h"
#if !IS_MASTER

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"

#define TAG "RS485_SLAVE"
#define BUF_SIZE 256
#define START_BYTE 0xAA
#define CMD_PING 0x01
#define CMD_PONG 0x02

#define UART_PORT       ((uart_port_t)CONFIG_RS485_UART_PORT)
#define UART_TX_PIN     CONFIG_RS485_UART_TX
#define UART_RX_PIN     CONFIG_RS485_UART_RX
#define UART_DE_PIN     CONFIG_RS485_UART_DE
#define UART_RE_PIN     CONFIG_RS485_UART_RE
#define BAUD_RATE       CONFIG_RS485_BAUD_RATE
#define MY_ADDR         CONFIG_SLAVE_ADDRESS

static uint8_t checksum(const uint8_t *b, size_t len) {
    uint8_t s = 0;
    for (size_t i = 0; i < len; ++i) s += b[i];
    return s;
}

static void build_frame(uint8_t *out, uint8_t addr, uint8_t cmd,
                        const uint8_t *data, uint8_t len, int *out_len) {
    out[0] = START_BYTE;
    out[1] = addr;
    out[2] = cmd;
    out[3] = len;
    if (len && data) memcpy(&out[4], data, len);
    out[4 + len] = checksum(&out[1], 3 + len);
    *out_len = 5 + len;
}

// --------------------- Updated Temperature Reading ---------------------
float read_temperature(adc_oneshot_unit_handle_t adc_handle) {
    int raw = 0;
    adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &raw);
    float v = (3.3f * raw) / 4095.0f;
    ESP_LOGI(TAG, "ADC raw=%d, V=%.3f", raw, v);

    if (v <= 0.01f) return -273.0f; // prevent log(0)

    // Voltage divider
    float R_fixed = 100000.0f;
    float R_ntc = (v * R_fixed) / (3.3f - v);

    // Beta formula
    float B = 3950.0f;
    float R0 = 47000.0f;
    float T0 = 298.15f;

    float inv_T = (1.0f / T0) + (1.0f / B) * logf(R_ntc / R0);
    float T = 1.0f / inv_T;

    return T - 273.15f; // Celsius
}

void app_main(void)
{
    ESP_LOGI(TAG, "Slave starting, addr=%d", MY_ADDR);

    // ---------------- UART Setup ----------------
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE*2, BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // ---------------- DE/RE GPIO Setup ----------------
    gpio_reset_pin(UART_DE_PIN);
    gpio_set_direction(UART_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(UART_DE_PIN, 0);

    gpio_reset_pin(UART_RE_PIN);
    gpio_set_direction(UART_RE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(UART_RE_PIN, 0);

    // ---------------- ADC Setup ----------------
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12, // replaces deprecated DB_11
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg));

    // ---------------- Buffers ----------------
    uint8_t rxbuf[BUF_SIZE];
    uint8_t txbuf[BUF_SIZE];

    // ---------------- Main Loop ----------------
    while(1) {
        int len = uart_read_bytes(UART_PORT, rxbuf, sizeof(rxbuf), 100 / portTICK_PERIOD_MS);
        if (len < 5) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        if (len > 0) ESP_LOGI(TAG, "Received %d bytes: %02X %02X %02X ...", len, rxbuf[0], rxbuf[1], rxbuf[2]);

        vTaskDelay(pdMS_TO_TICKS(50));

        int start_idx = -1;
        for (int i = 0; i < len; ++i) if (rxbuf[i] == START_BYTE) { start_idx = i; break; }
        if (start_idx < 0) continue;

        uint8_t *frame = &rxbuf[start_idx];
        uint8_t addr = frame[1];
        uint8_t cmd  = frame[2];
        uint8_t dlen = frame[3];
        if (4 + dlen >= len) continue;
        if (frame[4 + dlen] != checksum(&frame[1], 3 + dlen)) continue;

        if (addr == MY_ADDR && cmd == CMD_PING) {
            float tempC = read_temperature(adc_handle);
            int16_t temp_i16 = (int16_t)(tempC * 100);

            uint8_t payload[4] = {
                MY_ADDR,
                (uint8_t)(temp_i16 & 0xFF),
                (uint8_t)((temp_i16 >> 8) & 0xFF),
                0
            };

            int txlen = 0;
            build_frame(txbuf, MY_ADDR, CMD_PONG, payload, sizeof(payload), &txlen);

            // Transmit
            gpio_set_level(UART_DE_PIN, 1);
            gpio_set_level(UART_RE_PIN, 1);

            uart_write_bytes(UART_PORT, (const char*)txbuf, txlen);
            ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT, 50 / portTICK_PERIOD_MS));

            // Back to receive
            gpio_set_level(UART_DE_PIN, 0);
            gpio_set_level(UART_RE_PIN, 0);

            ESP_LOGI(TAG, "Replied PONG with temperature %.2f°C", tempC);
        }
    }
}

#endif // !IS_MASTER
