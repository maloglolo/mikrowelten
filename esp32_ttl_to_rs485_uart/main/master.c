#include "role_select.h"
#if IS_MASTER

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <stdio.h>
#include <string.h>

#include "wifi_creds.h"
#include "esp_system.h"  // for esp_reset_reason()

#define TAG "RS485_MASTER"
#define BUF_SIZE 256
#define START_BYTE 0xAA
#define CMD_PING 0x01
#define CMD_PONG 0x02
#define POLL_INTERVAL_MS 500
#define PACKET_READ_TICKS (200 / portTICK_PERIOD_MS)

#define UART_PORT ((uart_port_t)CONFIG_RS485_UART_PORT)
#define UART_TX_PIN CONFIG_RS485_UART_TX
#define UART_RX_PIN CONFIG_RS485_UART_RX
#define UART_DE_PIN CONFIG_RS485_UART_DE
#define UART_RE_PIN CONFIG_RS485_UART_RE 
#define BAUD_RATE CONFIG_RS485_BAUD_RATE

#include "soc/rtc_cntl_reg.h"




// --- UDP mirror setup ---
#define UDP_PORT 5005
#define UDP_TARGET_IP "192.168.0.241"  

static int udp_sock = -1;
static struct sockaddr_in dest_addr;

// --- checksum ---
static uint8_t checksum(const uint8_t *b, size_t len) {
    uint8_t s = 0;
    for (size_t i = 0; i < len; ++i) s += b[i];
    return s;
}

// --- build frame ---
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

// --- UDP send helper ---
static void udp_send(const uint8_t *data, int len) {
    if (udp_sock >= 0) {
        sendto(udp_sock, data, len, 0,
               (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }
}

// --- Wi-Fi init ---
static void wifi_init(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI(TAG, "Connected to SSID:%s, RSSI:%d", ap_info.ssid, ap_info.rssi);
    } else {
        ESP_LOGE(TAG, "Wi-Fi not connected yet!");
    }

    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID:%s", WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_connect());

    // crude wait (could use event handler instead)
    vTaskDelay(pdMS_TO_TICKS(3000));

    // setup UDP socket
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    dest_addr.sin_addr.s_addr = inet_addr(UDP_TARGET_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);

    ESP_LOGI(TAG, "UDP mirror ready -> %s:%d", UDP_TARGET_IP, UDP_PORT);
}

// --- main ---
void app_main(void) {
    REG_WRITE(RTC_CNTL_BROWN_OUT_REG, 0);

    // Check previous reset reason for brownout
    uint8_t master_brownout_flag = 0;
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "Previous reset was a BROWNOUT!");
        master_brownout_flag = 1;
    }

    ESP_LOGI(TAG, "Master starting");

    wifi_init();

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
            // Include brownout flag in first PING
            uint8_t ping_payload[1] = { master_brownout_flag };
            int txlen = 0;
            build_frame(txbuf, (uint8_t)addr, CMD_PING, ping_payload, sizeof(ping_payload), &txlen);

            // Enable transmit
            gpio_set_level(UART_DE_PIN, 1);
            gpio_set_level(UART_RE_PIN, 1);

            ESP_LOGI(TAG, "PING -> addr=%d", addr);
            uart_write_bytes(UART_PORT, (const char *)txbuf, txlen);
            ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT, 50 / portTICK_PERIOD_MS));

            // Mirror to Wi-Fi
            udp_send(txbuf, txlen);

            // Back to receive
            gpio_set_level(UART_DE_PIN, 0);
            gpio_set_level(UART_RE_PIN, 0);

            int len = uart_read_bytes(UART_PORT, rxbuf, sizeof(rxbuf), PACKET_READ_TICKS);
            if (len >= 5 && rxbuf[0] == START_BYTE) {
                uint8_t r_addr = rxbuf[1];
                uint8_t r_cmd  = rxbuf[2];
                uint8_t r_len  = rxbuf[3];
                uint8_t r_cs   = rxbuf[4 + r_len];

                // Mirror incoming packet
                udp_send(rxbuf, len);

                if (checksum(&rxbuf[1], 3 + r_len) == r_cs && r_cmd == CMD_PONG) {
                    ESP_LOGI(TAG, "PONG from %d", r_addr);

                    if (r_len >= 1) { // 1 byte moisture
                        uint8_t moist = rxbuf[5];
                        ESP_LOGI(TAG, "Soil moisture from %d: %d %%", r_addr, moist);
                    }
                }
            }

            // Clear brownout flag after first PING sent
            master_brownout_flag = 0;

            vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        }
    }
}

#endif // IS_MASTER
