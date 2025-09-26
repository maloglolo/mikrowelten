#include "buttons.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define TAG "BUTTONS"

#define BUTTON_ADC_GPIO CONFIG_BUTTON_ADC_PIN
#define BUTTON_ADC_CHANNEL ADC_CHANNEL_0
#define BUTTON_DEBOUNCE_MS CONFIG_BUTTON_DEBOUNCE_MS

static int last_button = -1;
static TickType_t last_tick = 0;
static volatile int button_to_log = -2;

static int read_button_adc(adc_oneshot_unit_handle_t adc, adc_channel_t channel) {
    int raw = 0;
    adc_oneshot_read(adc, channel, &raw);
    return raw;
}

static void button_log_task(void *arg) {
    while (1) {
        if (button_to_log != -2) {
            ESP_LOGI(TAG, "Button ADC: %d", button_to_log);
            button_to_log = -2;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void buttons_init(adc_oneshot_unit_handle_t *adc_handle) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&init_cfg, adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };
    adc_oneshot_config_channel(*adc_handle, BUTTON_ADC_CHANNEL, &chan_cfg);

    xTaskCreate(button_log_task, "button_log", 2048, NULL, 5, NULL);
}

void buttons_process(adc_oneshot_unit_handle_t adc) {
    int raw = read_button_adc(adc, BUTTON_ADC_CHANNEL);
    TickType_t now = xTaskGetTickCount();

    if (raw != last_button &&
        (now - last_tick) * portTICK_PERIOD_MS > BUTTON_DEBOUNCE_MS) {
        last_tick = now;
        last_button = raw;
        button_to_log = raw;
    }
}
