#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t actuator_cooler_init(int gpio_num);
esp_err_t actuator_cooler_set(bool on);
bool actuator_cooler_get(void);

#ifdef __cplusplus
}
#endif
