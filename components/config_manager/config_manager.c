#include "config_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "CFG_MGR";
static const char *NVS_NAMESPACE = "poultry_cfg";

#define CONFIG_SCHEMA_VERSION 2

#ifndef CONFIG_SPS_WIFI_SSID
#define CONFIG_SPS_WIFI_SSID "PoultryNet"
#endif
#ifndef CONFIG_SPS_WIFI_PASSWORD
#define CONFIG_SPS_WIFI_PASSWORD "poultry123"
#endif
#ifndef CONFIG_SPS_MQTT_BROKER_URI
#define CONFIG_SPS_MQTT_BROKER_URI "mqtt://192.168.1.100:1883"
#endif

esp_err_t config_manager_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Config manager initialized");
    return ESP_OK;
}

esp_err_t config_manager_load_defaults(void)
{
    // Set defaults if first boot or schema version changed
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    // Check init + schema version
    int32_t initialized = 0;
    int32_t cfg_ver = 0;
    err = nvs_get_i32(handle, "initialized", &initialized);
    if (err == ESP_OK) {
        nvs_get_i32(handle, "cfg_ver", &cfg_ver);
    }

    if (initialized == 1 && cfg_ver == CONFIG_SCHEMA_VERSION) {
        nvs_close(handle);
        ESP_LOGI(TAG, "Defaults already loaded");
        return ESP_OK;
    }

    // Temperature thresholds (x10)
    nvs_set_i32(handle, "temp_min", 180);
    nvs_set_i32(handle, "temp_max", 320);
    nvs_set_i32(handle, "temp_setpoint", 250);

    // Humidity thresholds (x10)
    nvs_set_i32(handle, "hum_min", 500);
    nvs_set_i32(handle, "hum_max", 700);
    nvs_set_i32(handle, "hum_deadband", 30); // 3.0%

    // Gas thresholds (stored as integer ppm * 10 for precision)
    nvs_set_i32(handle, "nh3_thresh", 250);    // 25.0 ppm
    nvs_set_i32(handle, "co_thresh", 500);      // 50.0 ppm
    nvs_set_i32(handle, "co2_thresh", 2500);    // 2500 ppm
    nvs_set_i32(handle, "ch4_thresh", 1000);    // 1000 ppm
    nvs_set_i32(handle, "h2s_thresh", 100);     // 10.0 ppm
    nvs_set_i32(handle, "dust_thresh", 1500);   // 150.0 ug/m3

    // PID gains (stored as integer * 100)
    nvs_set_i32(handle, "pid_kp", 500);   // 5.00
    nvs_set_i32(handle, "pid_ki", 10);    // 0.10
    nvs_set_i32(handle, "pid_kd", 100);   // 1.00

    // Light schedule
    nvs_set_i32(handle, "light_on_hr", 6);
    nvs_set_i32(handle, "light_off_hr", 22);

    // Feed schedule
    nvs_set_i32(handle, "feed_per_day", 3);
    nvs_set_i32(handle, "feed_dur_ms", 5000);

    // Water level
    nvs_set_i32(handle, "water_min", 20);
    nvs_set_i32(handle, "water_max", 80);

    // Sound alarm (x10)
    nvs_set_i32(handle, "sound_alarm_db", 850);

    // WiFi / MQTT
    nvs_set_str(handle, "wifi_ssid", CONFIG_SPS_WIFI_SSID);
    nvs_set_str(handle, "wifi_pass", CONFIG_SPS_WIFI_PASSWORD);

    // MQTT broker
    nvs_set_str(handle, "mqtt_uri", CONFIG_SPS_MQTT_BROKER_URI);

    // Mark as initialized
    nvs_set_i32(handle, "initialized", 1);
    nvs_set_i32(handle, "cfg_ver", CONFIG_SCHEMA_VERSION);

    err = nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Default configuration loaded");
    return err;
}

esp_err_t config_get_string(const char *key, char *out, size_t max_len)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    err = nvs_get_str(handle, key, out, &max_len);
    nvs_close(handle);
    return err;
}

esp_err_t config_get_float(const char *key, float *out)
{
    int32_t val;
    esp_err_t err = config_get_int(key, &val);
    if (err == ESP_OK && out) {
        *out = (float)val / 10.0f;
    }
    return err;
}

esp_err_t config_get_int(const char *key, int32_t *out)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    err = nvs_get_i32(handle, key, out);
    nvs_close(handle);
    return err;
}

esp_err_t config_set_string(const char *key, const char *value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_str(handle, key, value);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

esp_err_t config_set_float(const char *key, float value)
{
    return config_set_int(key, (int32_t)(value * 10.0f));
}

esp_err_t config_set_int(const char *key, int32_t value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_i32(handle, key, value);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

float config_get_float_or(const char *key, float default_val)
{
    float val;
    if (config_get_float(key, &val) == ESP_OK) return val;
    return default_val;
}

int32_t config_get_int_or(const char *key, int32_t default_val)
{
    int32_t val;
    if (config_get_int(key, &val) == ESP_OK) return val;
    return default_val;
}
