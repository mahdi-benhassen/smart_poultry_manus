#include "comm_manager.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "mqtt_client.h"
#include "config_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

static const char *TAG = "COMM_MGR";

/* ── WiFi config ── */
#ifndef CONFIG_SPS_WIFI_SSID
#define CONFIG_SPS_WIFI_SSID "PoultryNet"
#endif
#ifndef CONFIG_SPS_WIFI_PASSWORD
#define CONFIG_SPS_WIFI_PASSWORD "poultry123"
#endif
#ifndef CONFIG_SPS_MQTT_BROKER_URI
#define CONFIG_SPS_MQTT_BROKER_URI "mqtt://192.168.1.100:1883"
#endif

/* ── MQTT Topics ── */
#define TOPIC_SENSORS     "poultry/sensors"
#define TOPIC_ACTUATORS   "poultry/actuators"
#define TOPIC_ALERTS      "poultry/alerts"
#define TOPIC_AQI         "poultry/aqi"
#define TOPIC_CMD_PREFIX  "poultry/cmd/"

/* ── State ── */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static httpd_handle_t s_httpd = NULL;
static bool s_mqtt_connected = false;
static bool s_wifi_connected = false;
static mqtt_cmd_callback_t s_cmd_cb = NULL;
static int s_retry_count = 0;
#define MAX_RETRY 10

// Cached data for HTTP endpoints
static sensor_data_t s_cached_sensor = {0};
static actuator_state_t s_cached_actuator = {0};
static uint16_t s_cached_aqi = 0;
static char s_wifi_ssid[33] = {0};
static char s_wifi_password[65] = {0};
static char s_mqtt_uri[128] = {0};

static void load_runtime_comm_config(void)
{
    strlcpy(s_wifi_ssid, CONFIG_SPS_WIFI_SSID, sizeof(s_wifi_ssid));
    strlcpy(s_wifi_password, CONFIG_SPS_WIFI_PASSWORD, sizeof(s_wifi_password));
    strlcpy(s_mqtt_uri, CONFIG_SPS_MQTT_BROKER_URI, sizeof(s_mqtt_uri));

    if (config_get_string("wifi_ssid", s_wifi_ssid, sizeof(s_wifi_ssid)) != ESP_OK) {
        strlcpy(s_wifi_ssid, CONFIG_SPS_WIFI_SSID, sizeof(s_wifi_ssid));
    }
    if (config_get_string("wifi_pass", s_wifi_password, sizeof(s_wifi_password)) != ESP_OK) {
        strlcpy(s_wifi_password, CONFIG_SPS_WIFI_PASSWORD, sizeof(s_wifi_password));
    }
    if (config_get_string("mqtt_uri", s_mqtt_uri, sizeof(s_mqtt_uri)) != ESP_OK) {
        strlcpy(s_mqtt_uri, CONFIG_SPS_MQTT_BROKER_URI, sizeof(s_mqtt_uri));
    }

    ESP_LOGI(TAG, "Runtime comm config loaded (ssid=%s, mqtt=%s)", s_wifi_ssid, s_mqtt_uri);
}

static bool json_extract_int(const char *json, const char *key, int *out)
{
    if (json == NULL || key == NULL || out == NULL) return false;

    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *pos = strstr(json, pattern);
    if (pos == NULL) return false;

    pos = strchr(pos, ':');
    if (pos == NULL) return false;
    pos++;

    while (*pos == ' ' || *pos == '\t') pos++;

    char *endptr = NULL;
    long val = strtol(pos, &endptr, 10);
    if (endptr == pos) return false;

    *out = (int)val;
    return true;
}

static bool json_extract_bool(const char *json, const char *key, bool *out)
{
    if (json == NULL || key == NULL || out == NULL) return false;

    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *pos = strstr(json, pattern);
    if (pos == NULL) return false;

    pos = strchr(pos, ':');
    if (pos == NULL) return false;
    pos++;

    while (*pos == ' ' || *pos == '\t') pos++;

    if (*pos == '1') {
        *out = true;
        return true;
    }
    if (*pos == '0') {
        *out = false;
        return true;
    }
    if (strncasecmp(pos, "true", 4) == 0) {
        *out = true;
        return true;
    }
    if (strncasecmp(pos, "false", 5) == 0) {
        *out = false;
        return true;
    }

    return false;
}

/* ══════════════════════════════════════════════════════════════
 *  WiFi Event Handler
 * ══════════════════════════════════════════════════════════════ */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        if (s_retry_count < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGI(TAG, "WiFi retry %d/%d", s_retry_count, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connection failed after %d retries", MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any_id;
    esp_event_handler_instance_t inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                    &wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                    &wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, s_wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, s_wifi_password, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA initialized, connecting to %s", s_wifi_ssid);

    // Wait for connection (with timeout)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }
    ESP_LOGW(TAG, "WiFi connection timeout, will retry in background");
    return ESP_OK; // Non-fatal, MQTT will connect when WiFi is ready
}

/* ══════════════════════════════════════════════════════════════
 *  MQTT Event Handler
 * ══════════════════════════════════════════════════════════════ */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            s_mqtt_connected = true;
            // Subscribe to command topics
            esp_mqtt_client_subscribe(s_mqtt_client, "poultry/cmd/#", 1);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_mqtt_connected = false;
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT data: topic=%.*s", event->topic_len, event->topic);
            if (s_cmd_cb != NULL && event->topic && event->data) {
                // Null-terminate for callback
                char topic_buf[128] = {0};
                char data_buf[512] = {0};
                int tlen = event->topic_len < 127 ? event->topic_len : 127;
                int dlen = event->data_len < 511 ? event->data_len : 511;
                memcpy(topic_buf, event->topic, tlen);
                memcpy(data_buf, event->data, dlen);
                s_cmd_cb(topic_buf, data_buf, dlen);
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;

        default:
            break;
    }
}

static esp_err_t mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = s_mqtt_uri,
        .credentials.client_id = "esp32_poultry",
        .session.keepalive = 60,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT client init failed");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                    mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(s_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(err));
    }

    return err;
}

/* ══════════════════════════════════════════════════════════════
 *  HTTP Server Handlers
 * ══════════════════════════════════════════════════════════════ */

static esp_err_t http_status_handler(httpd_req_t *req)
{
    char buf[1024];
    int len = snprintf(buf, sizeof(buf),
        "{"
        "\"sensors\":{"
            "\"temperature\":%.1f,"
            "\"humidity\":%.1f,"
            "\"nh3_ppm\":%.1f,"
            "\"co_ppm\":%.1f,"
            "\"co2_ppm\":%.0f,"
            "\"methane_ppm\":%.0f,"
            "\"h2s_ppm\":%.1f,"
            "\"dust_ugm3\":%.1f,"
            "\"light_lux\":%.0f,"
            "\"water_level_pct\":%.0f,"
            "\"sound_db\":%.1f,"
            "\"door_open\":%s"
        "},"
        "\"actuators\":{"
            "\"fan_speed\":%d,"
            "\"exhaust_fan\":%s,"
            "\"heater\":%s,"
            "\"cooler\":%s,"
            "\"light_level\":%d,"
            "\"water_pump\":%s,"
            "\"alarm_pattern\":%d,"
            "\"door_angle\":%d"
        "},"
        "\"aqi\":%d,"
        "\"mqtt_connected\":%s,"
        "\"wifi_connected\":%s"
        "}",
        s_cached_sensor.temperature, s_cached_sensor.humidity,
        s_cached_sensor.nh3_ppm, s_cached_sensor.co_ppm,
        s_cached_sensor.co2_ppm, s_cached_sensor.methane_ppm,
        s_cached_sensor.h2s_ppm, s_cached_sensor.dust_ugm3,
        s_cached_sensor.light_lux, s_cached_sensor.water_level_pct,
        s_cached_sensor.sound_db,
        s_cached_sensor.door_open ? "true" : "false",
        s_cached_actuator.fan_speed_pct,
        s_cached_actuator.exhaust_fan_on ? "true" : "false",
        s_cached_actuator.heater_on ? "true" : "false",
        s_cached_actuator.cooler_on ? "true" : "false",
        s_cached_actuator.light_level_pct,
        s_cached_actuator.water_pump_on ? "true" : "false",
        s_cached_actuator.alarm_pattern,
        s_cached_actuator.door_servo_angle,
        s_cached_aqi,
        s_mqtt_connected ? "true" : "false",
        s_wifi_connected ? "true" : "false"
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t http_health_handler(httpd_req_t *req)
{
    const char *resp = "{\"status\":\"ok\",\"system\":\"smart_poultry\",\"version\":\"1.0.0\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

static esp_err_t http_control_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    ESP_LOGI(TAG, "Control request: %s", buf);

    // Parse JSON body and forward as command-topic style callbacks
    if (s_cmd_cb) {
        int int_val = 0;
        bool bool_val = false;
        char payload[16];

        if (json_extract_int(buf, "fan", &int_val)) {
            if (int_val < 0) int_val = 0;
            if (int_val > 100) int_val = 100;
            snprintf(payload, sizeof(payload), "%d", int_val);
            s_cmd_cb("poultry/cmd/fan", payload, strlen(payload));
        }

        if (json_extract_int(buf, "light", &int_val)) {
            if (int_val < 0) int_val = 0;
            if (int_val > 100) int_val = 100;
            snprintf(payload, sizeof(payload), "%d", int_val);
            s_cmd_cb("poultry/cmd/light", payload, strlen(payload));
        }

        if (json_extract_bool(buf, "heater", &bool_val)) {
            snprintf(payload, sizeof(payload), "%d", bool_val ? 1 : 0);
            s_cmd_cb("poultry/cmd/heater", payload, strlen(payload));
        }

        if (json_extract_bool(buf, "cooler", &bool_val)) {
            snprintf(payload, sizeof(payload), "%d", bool_val ? 1 : 0);
            s_cmd_cb("poultry/cmd/cooler", payload, strlen(payload));
        }

        const char *ota_key = "\"ota\"";
        const char *ota_pos = strstr(buf, ota_key);
        if (ota_pos != NULL) {
            ota_pos = strchr(ota_pos, ':');
            if (ota_pos != NULL) {
                ota_pos++;
                while (*ota_pos == ' ' || *ota_pos == '\t') ota_pos++;
                if (*ota_pos == '"') {
                    ota_pos++;
                    const char *ota_end = strchr(ota_pos, '"');
                    if (ota_end != NULL) {
                        size_t len = ota_end - ota_pos;
                        if (len > 0) {
                            char ota_url[256];
                            size_t cpy = (len < sizeof(ota_url) - 1) ? len : sizeof(ota_url) - 1;
                            memcpy(ota_url, ota_pos, cpy);
                            ota_url[cpy] = '\0';
                            s_cmd_cb("poultry/cmd/ota", ota_url, strlen(ota_url));
                        }
                    }
                }
            }
        }
    }

    const char *resp = "{\"status\":\"ok\",\"message\":\"Command received\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

static esp_err_t http_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;

    esp_err_t err = httpd_start(&s_httpd, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = http_status_handler,
    };
    httpd_register_uri_handler(s_httpd, &status_uri);

    httpd_uri_t health_uri = {
        .uri = "/api/health",
        .method = HTTP_GET,
        .handler = http_health_handler,
    };
    httpd_register_uri_handler(s_httpd, &health_uri);

    httpd_uri_t control_uri = {
        .uri = "/api/control",
        .method = HTTP_POST,
        .handler = http_control_handler,
    };
    httpd_register_uri_handler(s_httpd, &control_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════════════ */

esp_err_t comm_manager_init(void)
{
    esp_err_t err;

    load_runtime_comm_config();

    err = wifi_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi init issue (non-fatal): %s", esp_err_to_name(err));
    }

    err = mqtt_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MQTT init issue (non-fatal): %s", esp_err_to_name(err));
    }

    err = http_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HTTP init issue (non-fatal): %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Communication manager initialized");
    return ESP_OK;
}

esp_err_t comm_manager_publish_sensor_data(const sensor_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    memcpy(&s_cached_sensor, data, sizeof(sensor_data_t));

    if (!s_mqtt_connected || s_mqtt_client == NULL) return ESP_ERR_INVALID_STATE;

    char json[512];
    int len = snprintf(json, sizeof(json),
        "{"
        "\"temp\":%.1f,\"hum\":%.1f,"
        "\"nh3\":%.1f,\"co\":%.1f,\"co2\":%.0f,"
        "\"ch4\":%.0f,\"h2s\":%.1f,"
        "\"dust\":%.1f,\"lux\":%.0f,"
        "\"water\":%.0f,\"sound\":%.1f,"
        "\"door\":%s"
        "}",
        data->temperature, data->humidity,
        data->nh3_ppm, data->co_ppm, data->co2_ppm,
        data->methane_ppm, data->h2s_ppm,
        data->dust_ugm3, data->light_lux,
        data->water_level_pct, data->sound_db,
        data->door_open ? "true" : "false"
    );

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, TOPIC_SENSORS, json, len, 0, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t comm_manager_publish_actuator_state(const actuator_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    memcpy(&s_cached_actuator, state, sizeof(actuator_state_t));

    if (!s_mqtt_connected || s_mqtt_client == NULL) return ESP_ERR_INVALID_STATE;

    char json[256];
    int len = snprintf(json, sizeof(json),
        "{"
        "\"fan\":%d,\"exhaust\":%s,"
        "\"heater\":%s,\"cooler\":%s,"
        "\"light\":%d,\"pump\":%s,"
        "\"feed\":%s,\"alarm\":%d,"
        "\"door_angle\":%d"
        "}",
        state->fan_speed_pct,
        state->exhaust_fan_on ? "true" : "false",
        state->heater_on ? "true" : "false",
        state->cooler_on ? "true" : "false",
        state->light_level_pct,
        state->water_pump_on ? "true" : "false",
        state->feed_dispenser_on ? "true" : "false",
        state->alarm_pattern,
        state->door_servo_angle
    );

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, TOPIC_ACTUATORS, json, len, 0, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t comm_manager_send_alert(alert_level_t level, const char *message)
{
    if (!s_mqtt_connected || s_mqtt_client == NULL) return ESP_ERR_INVALID_STATE;

    const char *level_str[] = {"NONE", "INFO", "WARNING", "CRITICAL", "EMERGENCY"};
    const char *lvl = (level <= ALERT_LEVEL_EMERGENCY) ? level_str[level] : "UNKNOWN";

    char json[256];
    int len = snprintf(json, sizeof(json),
        "{\"level\":\"%s\",\"code\":%d,\"message\":\"%s\"}",
        lvl, (int)level, message ? message : "");

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, TOPIC_ALERTS, json, len, 1, 0);
    ESP_LOGI(TAG, "Alert published: %s - %s", lvl, message ? message : "");
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t comm_manager_publish_aqi(uint16_t aqi, aqi_category_t category)
{
    s_cached_aqi = aqi;

    if (!s_mqtt_connected || s_mqtt_client == NULL) return ESP_ERR_INVALID_STATE;

    const char *cat_str[] = {"Good", "Moderate", "UnhealthySensitive",
                              "Unhealthy", "VeryUnhealthy", "Hazardous"};
    const char *cat = (category <= AQI_HAZARDOUS) ? cat_str[category] : "Unknown";

    char json[128];
    int len = snprintf(json, sizeof(json),
        "{\"aqi\":%d,\"category\":\"%s\"}", aqi, cat);

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, TOPIC_AQI, json, len, 0, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

bool comm_manager_mqtt_connected(void)
{
    return s_mqtt_connected;
}

bool comm_manager_wifi_connected(void)
{
    return s_wifi_connected;
}

void comm_manager_register_cmd_callback(mqtt_cmd_callback_t cb)
{
    s_cmd_cb = cb;
}
