#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t actuator_light_init(int gpio_num);
esp_err_t actuator_light_set_level(uint8_t level_pct);
uint8_t actuator_light_get_level(void);

#ifdef __cplusplus
}
#endif
