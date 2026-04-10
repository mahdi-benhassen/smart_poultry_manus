/**
 * @file main.c
 * @brief Smart Poultry System - Main Application
 *
 * Integrates all components: sensors, actuators, control logic,
 * communication (WiFi/MQTT/HTTP), configuration, and OTA updates.
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "sensor_manager.h"
#include "actuator_manager.h"
#include "control_logic.h"
#include "comm_manager.h"
#include "config_manager.h"
#include "ota_manager.h"

static const char *TAG = "MAIN";

/* ── Shared data protected by mutex ── */
static sensor_data_t   s_sensor_data;
static actuator_state_t s_actuator_state;
static SemaphoreHandle_t s_data_mutex;

/* ══════════════════════════════════════════════════════════════
 *  Alert Callback - forwards alerts to comm_manager
 * ══════════════════════════════════════════════════════════════ */

static void alert_callback(alert_level_t level, const char *message)
{
    ESP_LOGW(TAG, "ALERT [%d]: %s", (int)level, message);
    comm_manager_send_alert(level, message);
}

/* ══════════════════════════════════════════════════════════════
 *  MQTT Command Callback - handles remote control commands
 * ══════════════════════════════════════════════════════════════ */

static void mqtt_cmd_callback(const char *topic, const char *data, int data_len)
{
    ESP_LOGI(TAG, "Command received: topic=%s data=%.*s", topic, data_len, data);

    // Handle OTA command: poultry/cmd/ota
    if (strstr(topic, "cmd/ota") != NULL) {
        char url[256] = {0};
        strncpy(url, data, data_len < 255 ? data_len : 255);
        ESP_LOGI(TAG, "OTA update requested: %s", url);
        ota_manager_start_update(url);
        return;
    }

    // Handle fan speed: poultry/cmd/fan
    if (strstr(topic, "cmd/fan") != NULL) {
        int speed = atoi(data);
        if (speed >= 0 && speed <= 100) {
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            s_actuator_state.fan_speed_pct = (uint8_t)speed;
            xSemaphoreGive(s_data_mutex);
            actuator_manager_apply(&s_actuator_state);
            ESP_LOGI(TAG, "Fan speed set to %d%% via command", speed);
        }
        return;
    }

    // Handle light: poultry/cmd/light
    if (strstr(topic, "cmd/light") != NULL) {
        int level = atoi(data);
        if (level >= 0 && level <= 100) {
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            s_actuator_state.light_level_pct = (uint8_t)level;
            xSemaphoreGive(s_data_mutex);
            actuator_manager_apply(&s_actuator_state);
            ESP_LOGI(TAG, "Light level set to %d%% via command", level);
        }
        return;
    }

    // Handle heater: poultry/cmd/heater
    if (strstr(topic, "cmd/heater") != NULL) {
        bool on = (data[0] == '1' || data[0] == 't');
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        s_actuator_state.heater_on = on;
        xSemaphoreGive(s_data_mutex);
        actuator_manager_apply(&s_actuator_state);
        return;
    }

    // Handle cooler: poultry/cmd/cooler
    if (strstr(topic, "cmd/cooler") != NULL) {
        bool on = (data[0] == '1' || data[0] == 't');
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        s_actuator_state.cooler_on = on;
        xSemaphoreGive(s_data_mutex);
        actuator_manager_apply(&s_actuator_state);
        return;
    }

    ESP_LOGW(TAG, "Unknown command topic: %s", topic);
}

/* ══════════════════════════════════════════════════════════════
 *  Sensor Task - reads all sensors periodically
 * ══════════════════════════════════════════════════════════════ */

static void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started");

    while (1) {
        sensor_data_t data;
        sensor_manager_read_all();
        esp_err_t err = sensor_manager_get_data(&data);
        if (err == ESP_OK) {
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            memcpy(&s_sensor_data, &data, sizeof(sensor_data_t));
            xSemaphoreGive(s_data_mutex);
        } else {
            ESP_LOGW(TAG, "Sensor read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Read every 2 seconds
    }
}

/* ══════════════════════════════════════════════════════════════
 *  Control Task - runs control algorithms
 * ══════════════════════════════════════════════════════════════ */

static void control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Control task started");

    // Wait for first sensor reading
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {
        sensor_data_t sensor_snap;
        actuator_state_t new_state;

        // Get latest sensor data
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        memcpy(&sensor_snap, &s_sensor_data, sizeof(sensor_data_t));
        xSemaphoreGive(s_data_mutex);

        // Run control logic
        memset(&new_state, 0, sizeof(actuator_state_t));
        esp_err_t err = control_logic_run(&sensor_snap, &new_state);
        if (err == ESP_OK) {
            // Apply to actuators
            actuator_manager_apply(&new_state);

            // Store state
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            memcpy(&s_actuator_state, &new_state, sizeof(actuator_state_t));
            xSemaphoreGive(s_data_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Control loop every 1 second
    }
}

/* ══════════════════════════════════════════════════════════════
 *  Communication Task - publishes data via MQTT
 * ══════════════════════════════════════════════════════════════ */

static void comm_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Communication task started");

    // Wait for WiFi and MQTT to connect
    vTaskDelay(pdMS_TO_TICKS(10000));

    while (1) {
        sensor_data_t sensor_snap;
        actuator_state_t actuator_snap;

        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        memcpy(&sensor_snap, &s_sensor_data, sizeof(sensor_data_t));
        memcpy(&actuator_snap, &s_actuator_state, sizeof(actuator_state_t));
        xSemaphoreGive(s_data_mutex);

        // Publish sensor data
        comm_manager_publish_sensor_data(&sensor_snap);

        // Publish actuator state
        comm_manager_publish_actuator_state(&actuator_snap);

        // Publish AQI
        comm_manager_publish_aqi(control_logic_get_aqi(),
                                  control_logic_get_aqi_category());

        vTaskDelay(pdMS_TO_TICKS(5000)); // Publish every 5 seconds
    }
}

/* ══════════════════════════════════════════════════════════════
 *  Monitoring Task - logs system status and checks anomalies
 * ══════════════════════════════════════════════════════════════ */

static void monitoring_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Monitoring task started");

    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1) {
        sensor_data_t sensor_snap;
        actuator_state_t actuator_snap;

        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        memcpy(&sensor_snap, &s_sensor_data, sizeof(sensor_data_t));
        memcpy(&actuator_snap, &s_actuator_state, sizeof(actuator_state_t));
        xSemaphoreGive(s_data_mutex);

        // Log comprehensive status
        ESP_LOGI(TAG, "═══ SYSTEM STATUS ═══");
        ESP_LOGI(TAG, "Sensors: Temp=%.1f°C Hum=%.1f%% NH3=%.1fppm CO=%.1fppm CO2=%.0fppm",
                 sensor_snap.temperature, sensor_snap.humidity,
                 sensor_snap.nh3_ppm, sensor_snap.co_ppm, sensor_snap.co2_ppm);
        ESP_LOGI(TAG, "  CH4=%.0fppm H2S=%.1fppm Dust=%.1fug/m3 Lux=%.0f Water=%.0f%%",
                 sensor_snap.methane_ppm, sensor_snap.h2s_ppm,
                 sensor_snap.dust_ugm3, sensor_snap.light_lux,
                 sensor_snap.water_level_pct);
        ESP_LOGI(TAG, "  Sound=%.1fdB Door=%s",
                 sensor_snap.sound_db,
                 sensor_snap.door_open ? "OPEN" : "CLOSED");
        ESP_LOGI(TAG, "Actuators: Fan=%d%% Exhaust=%s Heater=%s Cooler=%s Light=%d%%",
                 actuator_snap.fan_speed_pct,
                 actuator_snap.exhaust_fan_on ? "ON" : "OFF",
                 actuator_snap.heater_on ? "ON" : "OFF",
                 actuator_snap.cooler_on ? "ON" : "OFF",
                 actuator_snap.light_level_pct);
        ESP_LOGI(TAG, "  Pump=%s Feed=%s Alarm=%d Door=%d°",
                 actuator_snap.water_pump_on ? "ON" : "OFF",
                 actuator_snap.feed_dispenser_on ? "ON" : "OFF",
                 actuator_snap.alarm_pattern,
                 actuator_snap.door_servo_angle);
        ESP_LOGI(TAG, "AQI=%d Alert=%d WiFi=%s MQTT=%s",
                 control_logic_get_aqi(),
                 (int)control_logic_get_alert_level(),
                 comm_manager_wifi_connected() ? "OK" : "NO",
                 comm_manager_mqtt_connected() ? "OK" : "NO");

        // System health check
        size_t free_heap = esp_get_free_heap_size();
        size_t min_heap = esp_get_minimum_free_heap_size();
        ESP_LOGI(TAG, "Heap: free=%u min=%u", (unsigned)free_heap, (unsigned)min_heap);

        if (free_heap < 10000) {
            ESP_LOGW(TAG, "LOW MEMORY WARNING: %u bytes free", (unsigned)free_heap);
            comm_manager_send_alert(ALERT_LEVEL_WARNING, "Low memory");
        }

        vTaskDelay(pdMS_TO_TICKS(10000)); // Monitor every 10 seconds
    }
}

/* ══════════════════════════════════════════════════════════════
 *  Main Entry Point
 * ══════════════════════════════════════════════════════════════ */

void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   Smart Poultry Management System v1.0  ║");
    ESP_LOGI(TAG, "║   ESP32 + ESP-IDF                       ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");

    /* ── 1. Initialize Configuration Manager (NVS) ── */
    ESP_ERROR_CHECK(config_manager_init());
    config_manager_load_defaults();
    ESP_LOGI(TAG, "[1/6] Configuration manager ready");

    /* ── 2. Initialize Sensor Manager ── */
    ESP_ERROR_CHECK(sensor_manager_init());
    ESP_LOGI(TAG, "[2/6] Sensor manager ready");

    /* ── 3. Initialize Actuator Manager ── */
    ESP_ERROR_CHECK(actuator_manager_init());
    ESP_LOGI(TAG, "[3/6] Actuator manager ready");

    /* ── 4. Initialize Control Logic ── */
    ESP_ERROR_CHECK(control_logic_init());
    control_logic_register_alert_callback(alert_callback);
    ESP_LOGI(TAG, "[4/6] Control logic ready");

    /* ── 5. Initialize Communication Manager (WiFi + MQTT + HTTP) ── */
    ESP_ERROR_CHECK(comm_manager_init());
    comm_manager_register_cmd_callback(mqtt_cmd_callback);
    ESP_LOGI(TAG, "[5/6] Communication manager ready");

    /* ── 6. Initialize OTA Manager ── */
    ESP_ERROR_CHECK(ota_manager_init());
    ESP_LOGI(TAG, "[6/6] OTA manager ready");

    /* ── Create shared data mutex ── */
    s_data_mutex = xSemaphoreCreateMutex();
    assert(s_data_mutex != NULL);
    memset(&s_sensor_data, 0, sizeof(s_sensor_data));
    memset(&s_actuator_state, 0, sizeof(s_actuator_state));

    /* ── Start FreeRTOS Tasks ── */
    ESP_LOGI(TAG, "Starting system tasks...");

    xTaskCreate(sensor_task,     "sensor_task",     4096, NULL, 6, NULL);
    xTaskCreate(control_task,    "control_task",    4096, NULL, 5, NULL);
    xTaskCreate(comm_task,       "comm_task",       4096, NULL, 4, NULL);
    xTaskCreate(monitoring_task, "monitoring_task",  4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "System fully initialized and running");
}
