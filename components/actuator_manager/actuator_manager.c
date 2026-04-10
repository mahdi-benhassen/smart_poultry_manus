#include "actuator_manager.h"
#include "actuator_fan.h"
#include "actuator_exhaust_fan.h"
#include "actuator_heater.h"
#include "actuator_cooler.h"
#include "actuator_light.h"
#include "actuator_water_pump.h"
#include "actuator_feed_dispenser.h"
#include "actuator_alarm.h"
#include "actuator_door_servo.h"
#include "config_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "ACT_MGR";

// Default GPIO assignments (configurable via Kconfig)
#ifndef CONFIG_FAN_GPIO
#define CONFIG_FAN_GPIO             25
#endif
#ifndef CONFIG_EXHAUST_FAN_GPIO
#define CONFIG_EXHAUST_FAN_GPIO     26
#endif
#ifndef CONFIG_HEATER_GPIO
#define CONFIG_HEATER_GPIO          27
#endif
#ifndef CONFIG_COOLER_GPIO
#define CONFIG_COOLER_GPIO          14
#endif
#ifndef CONFIG_LIGHT_GPIO
#define CONFIG_LIGHT_GPIO           12
#endif
#ifndef CONFIG_WATER_PUMP_GPIO
#define CONFIG_WATER_PUMP_GPIO      13
#endif
#ifndef CONFIG_FEED_DISPENSER_GPIO
#define CONFIG_FEED_DISPENSER_GPIO  15
#endif
#ifndef CONFIG_ALARM_GPIO
#define CONFIG_ALARM_GPIO           2
#endif
#ifndef CONFIG_DOOR_SERVO_GPIO
#define CONFIG_DOOR_SERVO_GPIO      33
#endif
#ifndef CONFIG_SPS_FEED_DURATION_MS
#define CONFIG_SPS_FEED_DURATION_MS 5000
#endif

static actuator_state_t s_state;
static SemaphoreHandle_t s_mutex = NULL;
static uint32_t s_feed_duration_ms = CONFIG_SPS_FEED_DURATION_MS;

esp_err_t actuator_manager_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    int32_t configured_feed_ms = config_get_int_or("feed_dur_ms", CONFIG_SPS_FEED_DURATION_MS);
    if (configured_feed_ms < 1000) configured_feed_ms = 1000;
    if (configured_feed_ms > 30000) configured_feed_ms = 30000;
    s_feed_duration_ms = (uint32_t)configured_feed_ms;

    memset(&s_state, 0, sizeof(s_state));

    esp_err_t err;

    err = actuator_fan_init(CONFIG_FAN_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Fan init failed: %s", esp_err_to_name(err));

    err = actuator_exhaust_fan_init(CONFIG_EXHAUST_FAN_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Exhaust fan init failed: %s", esp_err_to_name(err));

    err = actuator_heater_init(CONFIG_HEATER_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Heater init failed: %s", esp_err_to_name(err));

    err = actuator_cooler_init(CONFIG_COOLER_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Cooler init failed: %s", esp_err_to_name(err));

    err = actuator_light_init(CONFIG_LIGHT_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Light init failed: %s", esp_err_to_name(err));

    err = actuator_water_pump_init(CONFIG_WATER_PUMP_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Water pump init failed: %s", esp_err_to_name(err));

    err = actuator_feed_dispenser_init(CONFIG_FEED_DISPENSER_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Feed dispenser init failed: %s", esp_err_to_name(err));

    err = actuator_alarm_init(CONFIG_ALARM_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Alarm init failed: %s", esp_err_to_name(err));

    err = actuator_door_servo_init(CONFIG_DOOR_SERVO_GPIO);
    if (err != ESP_OK) ESP_LOGW(TAG, "Door servo init failed: %s", esp_err_to_name(err));

    ESP_LOGI(TAG, "Actuator manager initialized");
    return ESP_OK;
}

esp_err_t actuator_manager_apply(const actuator_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(&s_state, state, sizeof(actuator_state_t));
    xSemaphoreGive(s_mutex);

    actuator_fan_set_speed(state->fan_speed_pct);
    actuator_exhaust_fan_set(state->exhaust_fan_on);
    actuator_heater_set(state->heater_on);
    actuator_cooler_set(state->cooler_on);
    actuator_light_set_level(state->light_level_pct);
    actuator_water_pump_set(state->water_pump_on);

    if (state->feed_dispenser_on && !actuator_feed_dispenser_is_active()) {
        actuator_feed_dispenser_dispense(s_feed_duration_ms);
    }

    actuator_alarm_set_pattern((alarm_pattern_t)state->alarm_pattern);
    actuator_door_servo_set_angle(state->door_servo_angle);

    return ESP_OK;
}

esp_err_t actuator_manager_get_state(actuator_state_t *out_state)
{
    if (out_state == NULL) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out_state, &s_state, sizeof(actuator_state_t));
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t actuator_manager_set_state(const actuator_state_t *state)
{
    return actuator_manager_apply(state);
}
