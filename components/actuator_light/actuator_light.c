#include "actuator_light.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "ACT_LIGHT";

#define LIGHT_LEDC_TIMER     LEDC_TIMER_1
#define LIGHT_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LIGHT_LEDC_CHANNEL   LEDC_CHANNEL_1
#define LIGHT_LEDC_DUTY_RES  LEDC_TIMER_10_BIT
#define LIGHT_LEDC_FREQ_HZ   1000
#define LIGHT_MAX_DUTY       ((1 << LIGHT_LEDC_DUTY_RES) - 1)

static uint8_t s_current_level = 0;
static bool s_initialized = false;

esp_err_t actuator_light_init(int gpio_num)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LIGHT_LEDC_MODE,
        .timer_num       = LIGHT_LEDC_TIMER,
        .duty_resolution = LIGHT_LEDC_DUTY_RES,
        .freq_hz         = LIGHT_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ch_conf = {
        .speed_mode = LIGHT_LEDC_MODE,
        .channel    = LIGHT_LEDC_CHANNEL,
        .timer_sel  = LIGHT_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio_num,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Install fade service for gradual dimming
    ledc_fade_func_install(0);

    s_initialized = true;
    s_current_level = 0;
    ESP_LOGI(TAG, "Light actuator initialized on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t actuator_light_set_level(uint8_t level_pct)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (level_pct > 100) level_pct = 100;

    uint32_t target_duty = (LIGHT_MAX_DUTY * level_pct) / 100;

    // Use fade for smooth transition (500ms)
    esp_err_t err = ledc_set_fade_with_time(LIGHT_LEDC_MODE, LIGHT_LEDC_CHANNEL,
                                             target_duty, 500);
    if (err != ESP_OK) {
        // Fallback to direct set
        ledc_set_duty(LIGHT_LEDC_MODE, LIGHT_LEDC_CHANNEL, target_duty);
        ledc_update_duty(LIGHT_LEDC_MODE, LIGHT_LEDC_CHANNEL);
    } else {
        ledc_fade_start(LIGHT_LEDC_MODE, LIGHT_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
    }

    s_current_level = level_pct;
    ESP_LOGD(TAG, "Light level set to %d%%", level_pct);
    return ESP_OK;
}

uint8_t actuator_light_get_level(void)
{
    return s_current_level;
}
