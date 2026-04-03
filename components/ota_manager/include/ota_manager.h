#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize OTA manager
 */
esp_err_t ota_manager_init(void);

/**
 * @brief Start OTA firmware update from URL
 * @param url HTTPS URL of firmware binary
 * @return ESP_OK if update started successfully
 */
esp_err_t ota_manager_start_update(const char *url);

/**
 * @brief Get OTA update progress (0-100)
 */
int ota_manager_get_progress(void);

/**
 * @brief Check if OTA update is in progress
 */
bool ota_manager_is_updating(void);

#ifdef __cplusplus
}
#endif
