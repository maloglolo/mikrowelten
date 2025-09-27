#include "display.h"
#include "driver/gpio.h"
#include "esp_clk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"
#include <esp_log.h>
#include <inttypes.h>

// SPI pins from Kconfig
#define PIN_CLK CONFIG_DISPLAY_PIN_CLK
#define PIN_MOSI CONFIG_DISPLAY_PIN_MOSI
#define PIN_CS CONFIG_DISPLAY_PIN_CS
#define PIN_DC CONFIG_DISPLAY_PIN_DC
#define PIN_RST CONFIG_DISPLAY_PIN_RST

static u8g2_t u8g2;

void display_init(void) {
  static u8g2_esp32_hal_t u8g2_hal = {.clk = PIN_CLK,
                                      .mosi = PIN_MOSI,
                                      .cs = PIN_CS,
                                      .dc = PIN_DC,
                                      .reset = PIN_RST};
  u8g2_esp32_hal_init(&u8g2_hal);

  gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT); // make sure reset is an output
  gpio_set_level(PIN_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(50));
  gpio_set_level(PIN_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(50));

  u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp32_spi_byte_cb,
                                     u8g2_esp32_gpio_and_delay_cb);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_ClearBuffer(&u8g2);

  u8g2_SendBuffer(&u8g2);
}

void display_stats(void) {
  char buf[32];

  u8g2_ClearBuffer(&u8g2);

  // Draw frame
  u8g2_DrawFrame(&u8g2, 0, 0, 128, 64);

  // Free heap
  snprintf(buf, sizeof(buf), "Heap: %" PRIu32, esp_get_free_heap_size());
  u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);

  u8g2_DrawStr(&u8g2, 5, 12, buf);

  // Uptime
  uint32_t ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  uint32_t sec = ms / 1000;
  uint32_t min = sec / 60;
  sec %= 60;
  snprintf(buf, sizeof(buf), "Uptime: %02" PRIu32 ":%02" PRIu32, min, sec);
  u8g2_DrawStr(&u8g2, 5, 24, buf);

  // CPU frequency
  snprintf(buf, sizeof(buf), "CPU: %d MHz", esp_clk_cpu_freq() / 1000000);
  u8g2_DrawStr(&u8g2, 5, 36, buf);

  u8g2_SendBuffer(&u8g2);
}

void display_update_contrast(int contrast) {
  if (contrast < 0)
    contrast = 0;
  if (contrast > 255)
    contrast = 255;
  u8g2_SetContrast(&u8g2, contrast);
}

u8g2_t *display_get_u8g2(void) { return &u8g2; }
