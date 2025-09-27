#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "display.h"
#include "rotary.h"
#include "buttons.h"

void app_main(void) {
    display_init();
    rotary_init();

    adc_oneshot_unit_handle_t adc_handle;
    buttons_init(&adc_handle);

    while (1) {
        buttons_process(adc_handle);
        display_stats();
        display_update_contrast(rotary_get_contrast());
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
