#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ALARM_PATTERN_OFF = 0,
    ALARM_PATTERN_CONTINUOUS,
    ALARM_PATTERN_INTERMITTENT,
    ALARM_PATTERN_SOS,
} alarm_pattern_t;

esp_err_t actuator_alarm_init(int gpio_num);
esp_err_t actuator_alarm_set_pattern(alarm_pattern_t pattern);
esp_err_t actuator_alarm_stop(void);
alarm_pattern_t actuator_alarm_get_pattern(void);

#ifdef __cplusplus
}
#endif
