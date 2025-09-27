#pragma once
#include "u8g2.h"

void display_init(void);
void display_update_contrast(int contrast);
u8g2_t *display_get_u8g2(void);
void display_stats(void);