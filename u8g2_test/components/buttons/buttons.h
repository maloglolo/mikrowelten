#pragma once
#include "esp_adc/adc_oneshot.h"

void buttons_init(adc_oneshot_unit_handle_t *adc_handle);
void buttons_process(adc_oneshot_unit_handle_t adc);
