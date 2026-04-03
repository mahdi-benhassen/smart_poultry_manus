#include "actuator_fan.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "ACT_FAN";

#define FAN_LEDC_TIMER       LEDC_TIMER_0
#define FAN_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define FAN_LEDC_CHANNEL     LEDC_CHANNEL_0
#define FAN_LEDC_DUTY_RES    LEDC_TIMER_10_BIT
#define FAN_LEDC_FREQ_HZ     25000  // 25kHz for PC-style PWM fans
#define FAN_MAX_DUTY         ((1 << FAN_LEDC_DUTY_RES) - 1)

static uint8_t s_current_speed = 0;
static bool s_initialized = false;

esp_err_t actuator_fan_init(int gpio_num)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode      = FAN_LEDC_MODE,
        .timer_num       = FAN_LEDC_TIMER,
        .duty_resolution = FAN_LEDC_DUTY_RES,
        .freq_hz         = FAN_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ch_conf = {
        .speed_mode = FAN_LEDC_MODE,
        .channel    = FAN_LEDC_CHANNEL,
        .timer_sel  = FAN_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio_num,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    s_current_speed = 0;
    ESP_LOGI(TAG, "Fan actuator initialized on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t actuator_fan_set_speed(uint8_t speed_pct)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed_pct > 100) speed_pct = 100;

    uint32_t duty = (FAN_MAX_DUTY * speed_pct) / 100;
    esp_err_t err = ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty);
    if (err != ESP_OK) return err;

    err = ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL);
    if (err != ESP_OK) return err;

    s_current_speed = speed_pct;
    ESP_LOGD(TAG, "Fan speed set to %d%% (duty=%lu)", speed_pct, (unsigned long)duty);
    return ESP_OK;
}

uint8_t actuator_fan_get_speed(void)
{
    return s_current_speed;
}
