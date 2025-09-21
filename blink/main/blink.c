#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"  // ✅ New ADC driver

#define LED_GPIO 18
#define THERMISTOR_ADC_CHANNEL ADC_CHANNEL_6  // GPIO34
#define THERMISTOR_GPIO 34

// Timer handle
TimerHandle_t led_timer;

// ADC handle and config (global)
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_chan_cfg_t adc_chan_config = {
    .bitwidth = ADC_BITWIDTH_12,
    .atten = ADC_ATTEN_DB_12,
};

// Constants for thermistor
const float Vref = 3.3;
const int ADC_Max = 4095;
const int R_fixed = 10000;
const float Beta = 3950;
const float T0 = 298.15; // 25°C in Kelvin
const float R0 = 10000;  // 10k thermistor at 25°C

// Timer callback function
void led_timer_callback(TimerHandle_t xTimer)
{
    static int state = 0;
    state = !state;
    gpio_set_level(LED_GPIO, state);
    printf("LED toggled to %d\n", state);
}

// Temperature reading function
float read_temperature_celsius()
{
    int adc_raw = 0;
    esp_err_t result = adc_oneshot_read(adc1_handle, THERMISTOR_ADC_CHANNEL, &adc_raw);

    if (result != ESP_OK) {
        printf("ADC read failed: %d\n", result);
        return -999;
    }
    printf("ADC Raw: %d\n", adc_raw);
    float voltage = (adc_raw * Vref) / ADC_Max;
    float resistance = R_fixed * (voltage / (Vref - voltage));
    
    if (voltage <= 0 || voltage >= Vref) {
        printf("Invalid voltage reading: %.3f V\n", voltage);
    return -999;  // error indicator
}
    float tempK = 1.0 / ((1.0 / T0) + (1.0 / Beta) * log(resistance / R0));
    float tempC = tempK - 273.15;

    return tempC;
}

void app_main(void)
{
    // LED pin config
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // Init ADC (new driver)
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&adc_init_config, &adc1_handle);
    adc_oneshot_config_channel(adc1_handle, THERMISTOR_ADC_CHANNEL, &adc_chan_config);

    // LED timer setup
    led_timer = xTimerCreate("LED Timer",
                             pdMS_TO_TICKS(500),
                             pdTRUE,
                             NULL,
                             led_timer_callback);
    xTimerStart(led_timer, 0);

    // Main loop
    while (1)
    {
        float tempC = read_temperature_celsius();
        printf("Temperature: %.2f °C\n", tempC);
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}
