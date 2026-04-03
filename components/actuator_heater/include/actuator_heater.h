#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t actuator_heater_init(int gpio_num);
esp_err_t actuator_heater_set(bool on);
bool actuator_heater_get(void);

#ifdef __cplusplus
}
#endif
