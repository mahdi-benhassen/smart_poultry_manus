#include "ota_manager.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "OTA_MGR";

static volatile bool s_updating = false;
static volatile int s_progress = 0;
static char s_url[256] = {0};

static void ota_task(void *arg)
{
    ESP_LOGI(TAG, "Starting OTA update from: %s", s_url);
    s_updating = true;
    s_progress = 0;

    esp_http_client_config_t http_config = {
        .url = s_url,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        s_updating = false;
        vTaskDelete(NULL);
        return;
    }

    int total_size = esp_https_ota_get_image_size(ota_handle);
    int downloaded = 0;

    while (1) {
        err = esp_https_ota_perform(ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;

        downloaded = esp_https_ota_get_image_len_read(ota_handle);
        if (total_size > 0) {
            s_progress = (downloaded * 100) / total_size;
        }
        ESP_LOGI(TAG, "OTA progress: %d%%", s_progress);
    }

    if (err == ESP_OK) {
        err = esp_https_ota_finish(ota_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "OTA update successful, restarting...");
            s_progress = 100;
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        } else {
            ESP_LOGE(TAG, "OTA finish failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "OTA perform failed: %s", esp_err_to_name(err));
        esp_https_ota_abort(ota_handle);
    }

    s_updating = false;
    vTaskDelete(NULL);
}

esp_err_t ota_manager_init(void)
{
    // Validate current firmware
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "Firmware pending verification, marking as valid");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    ESP_LOGI(TAG, "OTA manager initialized");
    return ESP_OK;
}

esp_err_t ota_manager_start_update(const char *url)
{
    if (url == NULL || strlen(url) == 0) return ESP_ERR_INVALID_ARG;
    if (strncmp(url, "https://", 8) != 0) {
        ESP_LOGE(TAG, "OTA URL must use HTTPS");
        return ESP_ERR_INVALID_ARG;
    }
    if (s_updating) {
        ESP_LOGW(TAG, "OTA already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    strncpy(s_url, url, sizeof(s_url) - 1);
    s_url[sizeof(s_url) - 1] = '\0';

    BaseType_t ret = xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create OTA task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

int ota_manager_get_progress(void)
{
    return s_progress;
}

bool ota_manager_is_updating(void)
{
    return s_updating;
}
