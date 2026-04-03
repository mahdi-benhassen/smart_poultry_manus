#pragma once

#include "esp_err.h"
#include "sensor_manager.h"
#include "actuator_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Alert severity levels ── */
typedef enum {
    ALERT_LEVEL_NONE = 0,
    ALERT_LEVEL_INFO,
    ALERT_LEVEL_WARNING,
    ALERT_LEVEL_CRITICAL,
    ALERT_LEVEL_EMERGENCY,
} alert_level_t;

/* ── Air Quality Index categories ── */
typedef enum {
    AQI_GOOD = 0,
    AQI_MODERATE,
    AQI_UNHEALTHY_SENSITIVE,
    AQI_UNHEALTHY,
    AQI_VERY_UNHEALTHY,
    AQI_HAZARDOUS,
} aqi_category_t;

/* ── PID controller state ── */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float integral_max;
} pid_controller_t;

/* ── Alert callback ── */
typedef void (*alert_callback_t)(alert_level_t level, const char *message);

/**
 * @brief Initialize control logic subsystem
 */
esp_err_t control_logic_init(void);

/**
 * @brief Run one cycle of control logic
 * @param sensor_data Current sensor readings
 * @param actuator_state Output actuator commands
 */
esp_err_t control_logic_run(const sensor_data_t *sensor_data, actuator_state_t *actuator_state);

/**
 * @brief Get current AQI value (0-500)
 */
uint16_t control_logic_get_aqi(void);

/**
 * @brief Get current AQI category
 */
aqi_category_t control_logic_get_aqi_category(void);

/**
 * @brief Get current alert level
 */
alert_level_t control_logic_get_alert_level(void);

/**
 * @brief Register alert callback
 */
void control_logic_register_alert_callback(alert_callback_t cb);

/**
 * @brief PID helper functions
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);
float pid_compute(pid_controller_t *pid, float setpoint, float measured, float dt);
void pid_reset(pid_controller_t *pid);

#ifdef __cplusplus
}
#endif
