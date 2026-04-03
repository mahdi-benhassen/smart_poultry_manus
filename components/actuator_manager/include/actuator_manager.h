#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t fan_speed_pct;
    bool    exhaust_fan_on;
    bool    heater_on;
    bool    cooler_on;
    uint8_t light_level_pct;
    bool    water_pump_on;
    bool    feed_dispenser_on;
    int     alarm_pattern;      // alarm_pattern_t
    uint8_t door_servo_angle;
} actuator_state_t;

/**
 * @brief Initialize all actuators
 */
esp_err_t actuator_manager_init(void);

/**
 * @brief Apply actuator states to hardware
 */
esp_err_t actuator_manager_apply(const actuator_state_t *state);

/**
 * @brief Get current actuator states (thread-safe copy)
 */
esp_err_t actuator_manager_get_state(actuator_state_t *out_state);

/**
 * @brief Set actuator state (thread-safe)
 */
esp_err_t actuator_manager_set_state(const actuator_state_t *state);

#ifdef __cplusplus
}
#endif
