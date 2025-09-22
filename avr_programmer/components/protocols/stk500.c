#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#define UART_NUM        UART_NUM_0
#define BUF_SIZE        256

#define STK_START       0x1B
#define STK_INSYNC      0x14
#define STK_OK          0x10
#define STK_CRC_EOP     0x20
#define STK_GET_SYNC    0x30
#define STK_GET_SIGN_ON 0x31

static const char *TAG = "STK500_TEST";

static void reply_ok(void) {
    uint8_t resp[2] = {STK_INSYNC, STK_OK};
    uart_write_bytes(UART_NUM, (const char*)resp, 2);
}

static void reply_sign_on(void) {
    const char msg[] = "AVR ISP";
    uint8_t resp[sizeof(msg) + 2];
    resp[0] = STK_INSYNC;
    memcpy(&resp[1], msg, sizeof(msg)-1);
    resp[sizeof(msg)] = STK_OK;
    uart_write_bytes(UART_NUM, (const char*)resp, sizeof(resp));
}

void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "STK500 test server running");

    uint8_t buf[1];
    while (1) {
        int len = uart_read_bytes(UART_NUM, buf, 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI(TAG, "RX: 0x%02X", buf[0]);

            if (buf[0] == STK_START) {
                int cmd = uart_read_bytes(UART_NUM, buf, 1, 10 / portTICK_PERIOD_MS);
                if (cmd > 0) {
                    switch(buf[0]) {
                        case STK_GET_SYNC: reply_ok(); break;
                        case STK_GET_SIGN_ON: reply_sign_on(); break;
                        default: reply_ok(); break;
                    }
                    // consume CRC_EOP
                    uart_read_bytes(UART_NUM, buf, 1, 10 / portTICK_PERIOD_MS);
                }
            }
        }
    }
}
