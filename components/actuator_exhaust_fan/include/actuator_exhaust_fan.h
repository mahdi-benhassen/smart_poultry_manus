#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t actuator_exhaust_fan_init(int gpio_num);
esp_err_t actuator_exhaust_fan_set(bool on);
bool actuator_exhaust_fan_get(void);

#ifdef __cplusplus
}
#endif
