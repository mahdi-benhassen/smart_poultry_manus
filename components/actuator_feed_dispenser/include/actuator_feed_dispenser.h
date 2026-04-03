#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t actuator_feed_dispenser_init(int gpio_num);
esp_err_t actuator_feed_dispenser_dispense(uint32_t duration_ms);
bool actuator_feed_dispenser_is_active(void);

#ifdef __cplusplus
}
#endif
