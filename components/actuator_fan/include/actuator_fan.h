#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize PWM fan actuator using LEDC
 * @param gpio_num GPIO pin connected to fan MOSFET gate
 * @return ESP_OK on success
 */
esp_err_t actuator_fan_init(int gpio_num);

/**
 * @brief Set fan speed
 * @param speed_pct Speed percentage 0-100
 * @return ESP_OK on success
 */
esp_err_t actuator_fan_set_speed(uint8_t speed_pct);

/**
 * @brief Get current fan speed
 * @return Current speed percentage 0-100
 */
uint8_t actuator_fan_get_speed(void);

#ifdef __cplusplus
}
#endif
