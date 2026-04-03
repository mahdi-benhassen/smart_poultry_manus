#include "actuator_door_servo.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "ACT_SERVO";

#define SERVO_LEDC_TIMER     LEDC_TIMER_2
#define SERVO_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL   LEDC_CHANNEL_2
#define SERVO_LEDC_DUTY_RES  LEDC_TIMER_14_BIT
#define SERVO_LEDC_FREQ_HZ   50   // 50Hz for standard servos
#define SERVO_MAX_DUTY       ((1 << SERVO_LEDC_DUTY_RES) - 1)

// Servo pulse width: 500us (0°) to 2500us (180°) at 50Hz (20ms period)
#define SERVO_MIN_PULSEWIDTH_US  500
#define SERVO_MAX_PULSEWIDTH_US  2500

static uint8_t s_current_angle = 0;
static bool s_initialized = false;

static uint32_t angle_to_duty(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint32_t pulse_us = SERVO_MIN_PULSEWIDTH_US +
        ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / 180;
    // Convert pulse width to duty cycle
    // Period = 1000000 / SERVO_LEDC_FREQ_HZ = 20000us
    uint32_t duty = (SERVO_MAX_DUTY * pulse_us) / (1000000 / SERVO_LEDC_FREQ_HZ);
    return duty;
}

esp_err_t actuator_door_servo_init(int gpio_num)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode      = SERVO_LEDC_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_LEDC_DUTY_RES,
        .freq_hz         = SERVO_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ch_conf = {
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_LEDC_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio_num,
        .duty       = angle_to_duty(0),
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    s_current_angle = 0;
    ESP_LOGI(TAG, "Door servo initialized on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t actuator_door_servo_set_angle(uint8_t angle)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (angle > 180) angle = 180;

    uint32_t duty = angle_to_duty(angle);
    esp_err_t err = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (err != ESP_OK) return err;
    err = ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    if (err != ESP_OK) return err;

    s_current_angle = angle;
    ESP_LOGD(TAG, "Servo angle set to %d°", angle);
    return ESP_OK;
}

uint8_t actuator_door_servo_get_angle(void)
{
    return s_current_angle;
}
