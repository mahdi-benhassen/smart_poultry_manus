#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize NVS-based configuration manager
 */
esp_err_t config_manager_init(void);

/**
 * @brief Load default configuration values
 */
esp_err_t config_manager_load_defaults(void);

/* ── Getters ── */
esp_err_t config_get_string(const char *key, char *out, size_t max_len);
esp_err_t config_get_float(const char *key, float *out);
esp_err_t config_get_int(const char *key, int32_t *out);

/* ── Setters ── */
esp_err_t config_set_string(const char *key, const char *value);
esp_err_t config_set_float(const char *key, float value);
esp_err_t config_set_int(const char *key, int32_t value);

/* ── Convenience getters with defaults ── */
float config_get_float_or(const char *key, float default_val);
int32_t config_get_int_or(const char *key, int32_t default_val);

#ifdef __cplusplus
}
#endif
