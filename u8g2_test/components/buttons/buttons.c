/*
 * Buttons Module (ADC-based) with Bitmasking
 * -------------------------------------------
 * This module reads multiple buttons connected to a single ADC channel.
 * Each button produces a distinct voltage level. The raw ADC value is
 * converted into a **bitmask**, where each bit represents one button.
 *
 * Bitmasking Concept:
 * ---------------------
 * A **bitmask** is an integer where each bit represents a separate flag/state.
 * In this module:
 *   - uint8_t pressed_buttons stores the state of 5 buttons
 *   - Bits 0-4 correspond to BUTTON_1 .. BUTTON_5
 *
 * Example: pressed_buttons = 0b000101
 *   - Bit 0 = 1 → BUTTON_1 pressed
 *   - Bit 1 = 0 → BUTTON_2 not pressed
 *   - Bit 2 = 1 → BUTTON_3 pressed
 *   - Bits 3-4 = 0 → BUTTON_4 and BUTTON_5 not pressed
 *
 * Bit Operations:
 *   - Set a bit (mark button pressed):
 *       mask |= (1 << i);
 *   - Check a bit (is button pressed?):
 *       if (mask & (1 << i)) { ... }
 *   - Clear a bit (mark button released):
 *       mask &= ~(1 << i);
 *
 * Usage:
 *   - adc_to_bitmask(raw) converts ADC value → bitmask
 *   - pressed_buttons holds the current button states
 *   - buttons_get_pressed() returns the bitmask for other code to use
 *
 * Resources:
 *   - Bitmasking basics:
 * https://www.clivemaxfield.com/coolbeans/masking-and-the-c-c-bitwise-operators/
 *   - FreeRTOS task and ADC usage:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html
 */

#include "buttons.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define TAG "BUTTONS"

#define BUTTON_ADC_CHANNEL ADC_CHANNEL_0
#define BUTTON_DEBOUNCE_MS CONFIG_BUTTON_DEBOUNCE_MS

// Expected ADC values for each button
static const int button_values[5] = {0, 400, 1100, 1800, 2700};
static const int tolerance[5] = {50, 50, 100, 50,
                                 80}; // ± tolerance for each button

static volatile uint8_t pressed_buttons =
    0; // bitmask: bit0=BUTTON1 ... bit4=BUTTON5
static int last_raw = -1;
static TickType_t last_tick = 0;

// Read ADC raw value
static int read_button_adc(adc_oneshot_unit_handle_t adc) {
  int raw = 0;
  adc_oneshot_read(adc, BUTTON_ADC_CHANNEL, &raw);
  return raw;
}

// Convert raw ADC value to bitmask
static uint8_t adc_to_bitmask(int raw) {
  uint8_t mask = 0;
  for (int i = 0; i < 5; i++) {
    if (raw >= button_values[i] - tolerance[i] &&
        raw <= button_values[i] + tolerance[i]) {
      mask |= (1 << i);
    }
  }
  return mask;
}

// Map bitmask to string for logging
static const char *bit_to_name(int bit) {
  switch (bit) {
  case 0:
    return "BUTTON_1";
  case 1:
    return "BUTTON_2";
  case 2:
    return "BUTTON_3";
  case 3:
    return "BUTTON_4";
  case 4:
    return "BUTTON_5";
  default:
    return "UNKNOWN";
  }
}

static void button_log_task(void *arg) {
  while (1) {
    uint8_t mask = pressed_buttons;
    if (mask) {
      ESP_LOGI(TAG, "Pressed buttons bitmask: 0x%02X", mask);
      for (int i = 0; i < 5; i++) {
        if (mask & (1 << i)) {
          ESP_LOGI(TAG, "%s", bit_to_name(i));
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void buttons_init(adc_oneshot_unit_handle_t *adc_handle) {
  adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT_2,
                                          .ulp_mode = ADC_ULP_MODE_DISABLE};
  adc_oneshot_new_unit(&init_cfg, adc_handle);

  adc_oneshot_chan_cfg_t chan_cfg = {.bitwidth = ADC_BITWIDTH_DEFAULT,
                                     .atten = ADC_ATTEN_DB_12};
  adc_oneshot_config_channel(*adc_handle, BUTTON_ADC_CHANNEL, &chan_cfg);

  xTaskCreate(button_log_task, "button_log", 2048, NULL, 5, NULL);
}

// Call this in main loop
void buttons_process(adc_oneshot_unit_handle_t adc) {
  int raw = read_button_adc(adc);
  TickType_t now = xTaskGetTickCount();

  if (raw != last_raw &&
      (now - last_tick) * portTICK_PERIOD_MS > BUTTON_DEBOUNCE_MS) {
    last_tick = now;
    last_raw = raw;
    pressed_buttons = adc_to_bitmask(raw); // update bitmask
  }
}

// Return currently pressed buttons bitmask
uint8_t buttons_get_pressed(void) { return pressed_buttons; }
