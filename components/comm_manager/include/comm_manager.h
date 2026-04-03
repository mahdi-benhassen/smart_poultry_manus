#pragma once

#include "esp_err.h"
#include "sensor_manager.h"
#include "actuator_manager.h"
#include "control_logic.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize communication manager (WiFi, MQTT, HTTP server)
 */
esp_err_t comm_manager_init(void);

/**
 * @brief Publish sensor data via MQTT as JSON
 */
esp_err_t comm_manager_publish_sensor_data(const sensor_data_t *data);

/**
 * @brief Publish actuator state via MQTT as JSON
 */
esp_err_t comm_manager_publish_actuator_state(const actuator_state_t *state);

/**
 * @brief Send alert notification via MQTT
 */
esp_err_t comm_manager_send_alert(alert_level_t level, const char *message);

/**
 * @brief Publish AQI data
 */
esp_err_t comm_manager_publish_aqi(uint16_t aqi, aqi_category_t category);

/**
 * @brief Check if MQTT is connected
 */
bool comm_manager_mqtt_connected(void);

/**
 * @brief Check if WiFi is connected
 */
bool comm_manager_wifi_connected(void);

/**
 * @brief Register command callback for remote control via MQTT
 */
typedef void (*mqtt_cmd_callback_t)(const char *topic, const char *data, int data_len);
void comm_manager_register_cmd_callback(mqtt_cmd_callback_t cb);

#ifdef __cplusplus
}
#endif
