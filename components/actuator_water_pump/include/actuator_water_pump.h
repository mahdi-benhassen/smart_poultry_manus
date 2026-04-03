#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t actuator_water_pump_init(int gpio_num);
esp_err_t actuator_water_pump_set(bool on);
bool actuator_water_pump_get(void);

#ifdef __cplusplus
}
#endif
