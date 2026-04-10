#include "control_logic.h"
#include "actuator_alarm.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <math.h>
#include <string.h>

static const char *TAG = "CTRL_LOGIC";

/* ══════════════════════════════════════════════════════════════
 *  Configuration defaults (overridable via Kconfig / NVS)
 * ══════════════════════════════════════════════════════════════ */

/* Map Kconfig (SPS_*) values to runtime floats/ints used by control logic */
#ifdef CONFIG_SPS_TEMP_SETPOINT
#define CONFIG_TEMP_SETPOINT        ((float)CONFIG_SPS_TEMP_SETPOINT / 10.0f)
#endif
#ifndef CONFIG_TEMP_SETPOINT
#define CONFIG_TEMP_SETPOINT        25.0f
#endif
#ifdef CONFIG_SPS_TEMP_MIN
#define CONFIG_TEMP_MIN             ((float)CONFIG_SPS_TEMP_MIN / 10.0f)
#endif
#ifndef CONFIG_TEMP_MIN
#define CONFIG_TEMP_MIN             18.0f
#endif
#ifdef CONFIG_SPS_TEMP_MAX
#define CONFIG_TEMP_MAX             ((float)CONFIG_SPS_TEMP_MAX / 10.0f)
#endif
#ifndef CONFIG_TEMP_MAX
#define CONFIG_TEMP_MAX             32.0f
#endif
#ifdef CONFIG_SPS_HUMIDITY_MIN
#define CONFIG_HUMIDITY_MIN         ((float)CONFIG_SPS_HUMIDITY_MIN / 10.0f)
#endif
#ifndef CONFIG_HUMIDITY_MIN
#define CONFIG_HUMIDITY_MIN         50.0f
#endif
#ifdef CONFIG_SPS_HUMIDITY_MAX
#define CONFIG_HUMIDITY_MAX         ((float)CONFIG_SPS_HUMIDITY_MAX / 10.0f)
#endif
#ifndef CONFIG_HUMIDITY_MAX
#define CONFIG_HUMIDITY_MAX         70.0f
#endif
#ifndef CONFIG_HUMIDITY_DEADBAND
#define CONFIG_HUMIDITY_DEADBAND    3.0f
#endif
#ifdef CONFIG_SPS_NH3_THRESHOLD
#define CONFIG_NH3_THRESHOLD        ((float)CONFIG_SPS_NH3_THRESHOLD / 10.0f)
#endif
#ifndef CONFIG_NH3_THRESHOLD
#define CONFIG_NH3_THRESHOLD        25.0f
#endif
#ifdef CONFIG_SPS_CO_THRESHOLD
#define CONFIG_CO_THRESHOLD         ((float)CONFIG_SPS_CO_THRESHOLD / 10.0f)
#endif
#ifndef CONFIG_CO_THRESHOLD
#define CONFIG_CO_THRESHOLD         50.0f
#endif
#ifdef CONFIG_SPS_CO2_THRESHOLD
#define CONFIG_CO2_THRESHOLD        ((float)CONFIG_SPS_CO2_THRESHOLD)
#endif
#ifndef CONFIG_CO2_THRESHOLD
#define CONFIG_CO2_THRESHOLD        2500.0f
#endif
#ifdef CONFIG_SPS_METHANE_THRESHOLD
#define CONFIG_METHANE_THRESHOLD    ((float)CONFIG_SPS_METHANE_THRESHOLD)
#endif
#ifndef CONFIG_METHANE_THRESHOLD
#define CONFIG_METHANE_THRESHOLD    1000.0f
#endif
#ifdef CONFIG_SPS_H2S_THRESHOLD
#define CONFIG_H2S_THRESHOLD        ((float)CONFIG_SPS_H2S_THRESHOLD / 10.0f)
#endif
#ifndef CONFIG_H2S_THRESHOLD
#define CONFIG_H2S_THRESHOLD        10.0f
#endif
#ifdef CONFIG_SPS_DUST_THRESHOLD
#define CONFIG_DUST_THRESHOLD       ((float)CONFIG_SPS_DUST_THRESHOLD / 10.0f)
#endif
#ifndef CONFIG_DUST_THRESHOLD
#define CONFIG_DUST_THRESHOLD       150.0f
#endif
#ifdef CONFIG_SPS_WATER_LEVEL_MIN
#define CONFIG_WATER_LEVEL_MIN      ((float)CONFIG_SPS_WATER_LEVEL_MIN)
#endif
#ifndef CONFIG_WATER_LEVEL_MIN
#define CONFIG_WATER_LEVEL_MIN      20.0f
#endif
#ifdef CONFIG_SPS_WATER_LEVEL_MAX
#define CONFIG_WATER_LEVEL_MAX      ((float)CONFIG_SPS_WATER_LEVEL_MAX)
#endif
#ifndef CONFIG_WATER_LEVEL_MAX
#define CONFIG_WATER_LEVEL_MAX      80.0f
#endif
#ifdef CONFIG_SPS_SOUND_ALARM_DB
#define CONFIG_SOUND_ALARM_DB       ((float)CONFIG_SPS_SOUND_ALARM_DB / 10.0f)
#endif
#ifndef CONFIG_SOUND_ALARM_DB
#define CONFIG_SOUND_ALARM_DB       85.0f
#endif

// PID defaults for temperature
#ifdef CONFIG_SPS_PID_KP
#define CONFIG_PID_KP               ((float)CONFIG_SPS_PID_KP / 100.0f)
#endif
#ifndef CONFIG_PID_KP
#define CONFIG_PID_KP               5.0f
#endif
#ifdef CONFIG_SPS_PID_KI
#define CONFIG_PID_KI               ((float)CONFIG_SPS_PID_KI / 100.0f)
#endif
#ifndef CONFIG_PID_KI
#define CONFIG_PID_KI               0.1f
#endif
#ifdef CONFIG_SPS_PID_KD
#define CONFIG_PID_KD               ((float)CONFIG_SPS_PID_KD / 100.0f)
#endif
#ifndef CONFIG_PID_KD
#define CONFIG_PID_KD               1.0f
#endif

// Light schedule (hours in 24h format)
#ifdef CONFIG_SPS_LIGHT_ON_HOUR
#define CONFIG_LIGHT_ON_HOUR        CONFIG_SPS_LIGHT_ON_HOUR
#endif
#ifndef CONFIG_LIGHT_ON_HOUR
#define CONFIG_LIGHT_ON_HOUR        6
#endif
#ifdef CONFIG_SPS_LIGHT_OFF_HOUR
#define CONFIG_LIGHT_OFF_HOUR       CONFIG_SPS_LIGHT_OFF_HOUR
#endif
#ifndef CONFIG_LIGHT_OFF_HOUR
#define CONFIG_LIGHT_OFF_HOUR       22
#endif

// Feed schedule
#ifdef CONFIG_SPS_FEED_TIMES_PER_DAY
#define CONFIG_FEED_TIMES_PER_DAY   CONFIG_SPS_FEED_TIMES_PER_DAY
#endif
#ifndef CONFIG_FEED_TIMES_PER_DAY
#define CONFIG_FEED_TIMES_PER_DAY   3
#endif
#ifdef CONFIG_SPS_FEED_DURATION_MS
#define CONFIG_FEED_DURATION_MS     CONFIG_SPS_FEED_DURATION_MS
#endif
#ifndef CONFIG_FEED_DURATION_MS
#define CONFIG_FEED_DURATION_MS     5000
#endif

/* Immutable defaults derived from Kconfig (fallback when NVS keys are missing) */
static const float DFLT_TEMP_SETPOINT = CONFIG_TEMP_SETPOINT;
static const float DFLT_TEMP_MIN = CONFIG_TEMP_MIN;
static const float DFLT_TEMP_MAX = CONFIG_TEMP_MAX;
static const float DFLT_HUMIDITY_MIN = CONFIG_HUMIDITY_MIN;
static const float DFLT_HUMIDITY_MAX = CONFIG_HUMIDITY_MAX;
static const float DFLT_HUMIDITY_DEADBAND = CONFIG_HUMIDITY_DEADBAND;
static const float DFLT_NH3_THRESHOLD = CONFIG_NH3_THRESHOLD;
static const float DFLT_CO_THRESHOLD = CONFIG_CO_THRESHOLD;
static const float DFLT_CO2_THRESHOLD = CONFIG_CO2_THRESHOLD;
static const float DFLT_METHANE_THRESHOLD = CONFIG_METHANE_THRESHOLD;
static const float DFLT_H2S_THRESHOLD = CONFIG_H2S_THRESHOLD;
static const float DFLT_DUST_THRESHOLD = CONFIG_DUST_THRESHOLD;
static const float DFLT_WATER_LEVEL_MIN = CONFIG_WATER_LEVEL_MIN;
static const float DFLT_WATER_LEVEL_MAX = CONFIG_WATER_LEVEL_MAX;
static const float DFLT_SOUND_ALARM_DB = CONFIG_SOUND_ALARM_DB;
static const float DFLT_PID_KP = CONFIG_PID_KP;
static const float DFLT_PID_KI = CONFIG_PID_KI;
static const float DFLT_PID_KD = CONFIG_PID_KD;
static const int DFLT_LIGHT_ON_HOUR = CONFIG_LIGHT_ON_HOUR;
static const int DFLT_LIGHT_OFF_HOUR = CONFIG_LIGHT_OFF_HOUR;
static const int DFLT_FEED_TIMES_PER_DAY = CONFIG_FEED_TIMES_PER_DAY;

typedef struct {
    float temp_setpoint;
    float temp_min;
    float temp_max;
    float humidity_min;
    float humidity_max;
    float humidity_deadband;
    float nh3_threshold;
    float co_threshold;
    float co2_threshold;
    float methane_threshold;
    float h2s_threshold;
    float dust_threshold;
    float water_level_min;
    float water_level_max;
    float sound_alarm_db;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    int light_on_hour;
    int light_off_hour;
    int feed_times_per_day;
} runtime_cfg_t;

static runtime_cfg_t s_cfg;

static void load_runtime_control_config(void)
{
    s_cfg.temp_setpoint = (float)config_get_int_or("temp_setpoint", (int32_t)(DFLT_TEMP_SETPOINT * 10.0f)) / 10.0f;
    s_cfg.temp_min = (float)config_get_int_or("temp_min", (int32_t)(DFLT_TEMP_MIN * 10.0f)) / 10.0f;
    s_cfg.temp_max = (float)config_get_int_or("temp_max", (int32_t)(DFLT_TEMP_MAX * 10.0f)) / 10.0f;

    s_cfg.humidity_min = (float)config_get_int_or("hum_min", (int32_t)(DFLT_HUMIDITY_MIN * 10.0f)) / 10.0f;
    s_cfg.humidity_max = (float)config_get_int_or("hum_max", (int32_t)(DFLT_HUMIDITY_MAX * 10.0f)) / 10.0f;
    s_cfg.humidity_deadband = (float)config_get_int_or("hum_deadband", (int32_t)(DFLT_HUMIDITY_DEADBAND * 10.0f)) / 10.0f;

    s_cfg.nh3_threshold = (float)config_get_int_or("nh3_thresh", (int32_t)(DFLT_NH3_THRESHOLD * 10.0f)) / 10.0f;
    s_cfg.co_threshold = (float)config_get_int_or("co_thresh", (int32_t)(DFLT_CO_THRESHOLD * 10.0f)) / 10.0f;
    s_cfg.co2_threshold = (float)config_get_int_or("co2_thresh", (int32_t)DFLT_CO2_THRESHOLD);
    s_cfg.methane_threshold = (float)config_get_int_or("ch4_thresh", (int32_t)DFLT_METHANE_THRESHOLD);
    s_cfg.h2s_threshold = (float)config_get_int_or("h2s_thresh", (int32_t)(DFLT_H2S_THRESHOLD * 10.0f)) / 10.0f;
    s_cfg.dust_threshold = (float)config_get_int_or("dust_thresh", (int32_t)(DFLT_DUST_THRESHOLD * 10.0f)) / 10.0f;

    s_cfg.water_level_min = (float)config_get_int_or("water_min", (int32_t)DFLT_WATER_LEVEL_MIN);
    s_cfg.water_level_max = (float)config_get_int_or("water_max", (int32_t)DFLT_WATER_LEVEL_MAX);
    s_cfg.sound_alarm_db = (float)config_get_int_or("sound_alarm_db", (int32_t)(DFLT_SOUND_ALARM_DB * 10.0f)) / 10.0f;

    s_cfg.pid_kp = (float)config_get_int_or("pid_kp", (int32_t)(DFLT_PID_KP * 100.0f)) / 100.0f;
    s_cfg.pid_ki = (float)config_get_int_or("pid_ki", (int32_t)(DFLT_PID_KI * 100.0f)) / 100.0f;
    s_cfg.pid_kd = (float)config_get_int_or("pid_kd", (int32_t)(DFLT_PID_KD * 100.0f)) / 100.0f;

    s_cfg.light_on_hour = (int)config_get_int_or("light_on_hr", DFLT_LIGHT_ON_HOUR);
    s_cfg.light_off_hour = (int)config_get_int_or("light_off_hr", DFLT_LIGHT_OFF_HOUR);
    s_cfg.feed_times_per_day = (int)config_get_int_or("feed_per_day", DFLT_FEED_TIMES_PER_DAY);

    if (s_cfg.feed_times_per_day < 1) s_cfg.feed_times_per_day = 1;
    if (s_cfg.feed_times_per_day > 6) s_cfg.feed_times_per_day = 6;
    if (s_cfg.light_on_hour < 0) s_cfg.light_on_hour = 0;
    if (s_cfg.light_on_hour > 23) s_cfg.light_on_hour = 23;
    if (s_cfg.light_off_hour < 0) s_cfg.light_off_hour = 0;
    if (s_cfg.light_off_hour > 23) s_cfg.light_off_hour = 23;

    ESP_LOGI(TAG,
             "Runtime cfg: Tset=%.1f Tmin=%.1f Tmax=%.1f Hmin=%.1f Hmax=%.1f Feed=%d/day",
             s_cfg.temp_setpoint, s_cfg.temp_min, s_cfg.temp_max,
             s_cfg.humidity_min, s_cfg.humidity_max, s_cfg.feed_times_per_day);
}

#undef CONFIG_TEMP_SETPOINT
#undef CONFIG_TEMP_MIN
#undef CONFIG_TEMP_MAX
#undef CONFIG_HUMIDITY_MIN
#undef CONFIG_HUMIDITY_MAX
#undef CONFIG_HUMIDITY_DEADBAND
#undef CONFIG_NH3_THRESHOLD
#undef CONFIG_CO_THRESHOLD
#undef CONFIG_CO2_THRESHOLD
#undef CONFIG_METHANE_THRESHOLD
#undef CONFIG_H2S_THRESHOLD
#undef CONFIG_DUST_THRESHOLD
#undef CONFIG_WATER_LEVEL_MIN
#undef CONFIG_WATER_LEVEL_MAX
#undef CONFIG_SOUND_ALARM_DB
#undef CONFIG_PID_KP
#undef CONFIG_PID_KI
#undef CONFIG_PID_KD
#undef CONFIG_LIGHT_ON_HOUR
#undef CONFIG_LIGHT_OFF_HOUR
#undef CONFIG_FEED_TIMES_PER_DAY

#define CONFIG_TEMP_SETPOINT        (s_cfg.temp_setpoint)
#define CONFIG_TEMP_MIN             (s_cfg.temp_min)
#define CONFIG_TEMP_MAX             (s_cfg.temp_max)
#define CONFIG_HUMIDITY_MIN         (s_cfg.humidity_min)
#define CONFIG_HUMIDITY_MAX         (s_cfg.humidity_max)
#define CONFIG_HUMIDITY_DEADBAND    (s_cfg.humidity_deadband)
#define CONFIG_NH3_THRESHOLD        (s_cfg.nh3_threshold)
#define CONFIG_CO_THRESHOLD         (s_cfg.co_threshold)
#define CONFIG_CO2_THRESHOLD        (s_cfg.co2_threshold)
#define CONFIG_METHANE_THRESHOLD    (s_cfg.methane_threshold)
#define CONFIG_H2S_THRESHOLD        (s_cfg.h2s_threshold)
#define CONFIG_DUST_THRESHOLD       (s_cfg.dust_threshold)
#define CONFIG_WATER_LEVEL_MIN      (s_cfg.water_level_min)
#define CONFIG_WATER_LEVEL_MAX      (s_cfg.water_level_max)
#define CONFIG_SOUND_ALARM_DB       (s_cfg.sound_alarm_db)
#define CONFIG_PID_KP               (s_cfg.pid_kp)
#define CONFIG_PID_KI               (s_cfg.pid_ki)
#define CONFIG_PID_KD               (s_cfg.pid_kd)
#define CONFIG_LIGHT_ON_HOUR        (s_cfg.light_on_hour)
#define CONFIG_LIGHT_OFF_HOUR       (s_cfg.light_off_hour)
#define CONFIG_FEED_TIMES_PER_DAY   (s_cfg.feed_times_per_day)

/* ══════════════════════════════════════════════════════════════
 *  Internal state
 * ══════════════════════════════════════════════════════════════ */

static pid_controller_t s_temp_pid;
static uint16_t s_aqi = 0;
static aqi_category_t s_aqi_cat = AQI_GOOD;
static alert_level_t s_alert_level = ALERT_LEVEL_NONE;
static alert_callback_t s_alert_cb = NULL;
static int64_t s_last_run_us = 0;
static alert_level_t s_last_notified_level = ALERT_LEVEL_NONE;
static int64_t s_last_alert_notify_us = 0;
static char s_last_alert_msg[256] = {0};

// Anomaly detection state
#define ANOMALY_HISTORY_SIZE 30
static float s_temp_history[ANOMALY_HISTORY_SIZE];
static int s_temp_hist_idx = 0;
static int s_temp_hist_count = 0;
static bool s_water_pump_on = false;

// Feed schedule tracking
static int s_feed_hours[6] = {7, 12, 18, 0, 0, 0};
static int s_last_feed_hour = -1;

static float safe_ratio(float value, float threshold)
{
    if (threshold <= 0.001f) return 0.0f;
    return value / threshold;
}

static int64_t alert_repeat_interval_us(alert_level_t level)
{
    switch (level) {
        case ALERT_LEVEL_WARNING: return 300LL * 1000000LL;
        case ALERT_LEVEL_CRITICAL: return 60LL * 1000000LL;
        case ALERT_LEVEL_EMERGENCY: return 30LL * 1000000LL;
        default: return 0;
    }
}

static bool should_emit_notification(alert_level_t level, const char *msg)
{
    if (level < ALERT_LEVEL_WARNING) return false;

    int64_t now_us = esp_timer_get_time();
    bool level_changed = (level != s_last_notified_level);
    bool msg_changed = (strncmp(msg, s_last_alert_msg, sizeof(s_last_alert_msg)) != 0);
    bool timed_repeat = (now_us - s_last_alert_notify_us) >= alert_repeat_interval_us(level);

    if (!level_changed && !msg_changed && !timed_repeat) {
        return false;
    }

    s_last_notified_level = level;
    s_last_alert_notify_us = now_us;
    strlcpy(s_last_alert_msg, msg, sizeof(s_last_alert_msg));
    return true;
}

/* ══════════════════════════════════════════════════════════════
 *  PID Controller
 * ══════════════════════════════════════════════════════════════ */

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_max = (out_max - out_min) / (ki > 0.001f ? ki : 1.0f);
}

float pid_compute(pid_controller_t *pid, float setpoint, float measured, float dt)
{
    if (dt <= 0.0f) dt = 1.0f;

    float error = setpoint - measured;

    // Proportional
    float p_term = pid->kp * error;

    // Integral with anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    // Derivative (on error)
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;
    pid->prev_error = error;

    // Sum and clamp
    float output = p_term + i_term + d_term;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/* ══════════════════════════════════════════════════════════════
 *  Hysteresis Controller for Humidity
 * ══════════════════════════════════════════════════════════════ */

typedef struct {
    float low_threshold;
    float high_threshold;
    float deadband;
    bool  state; // true = active (e.g., humidifier on)
} hysteresis_t;

static hysteresis_t s_humidity_hyst;

static void hysteresis_init(hysteresis_t *h, float low, float high, float deadband)
{
    h->low_threshold = low;
    h->high_threshold = high;
    h->deadband = deadband;
    h->state = false;
}

static bool hysteresis_update(hysteresis_t *h, float value)
{
    if (h->state) {
        // Currently active, turn off when value reaches target + deadband
        if (value >= h->high_threshold - h->deadband) {
            h->state = false;
        }
    } else {
        // Currently inactive, turn on when value drops below low threshold
        if (value < h->low_threshold) {
            h->state = true;
        }
    }
    return h->state;
}

/* ══════════════════════════════════════════════════════════════
 *  Air Quality Index (AQI) Calculation
 *  Combines NH3, CO, CO2, CH4, H2S, dust into 0-500 index
 * ══════════════════════════════════════════════════════════════ */

static float normalize_gas(float value, float good_limit, float hazardous_limit)
{
    if (value <= 0) return 0;
    if (value >= hazardous_limit) return 500.0f;
    float ratio = value / hazardous_limit;
    return ratio * 500.0f;
}

static uint16_t calculate_aqi(const sensor_data_t *data)
{
    // Individual sub-indices (0-500 scale)
    float nh3_idx     = normalize_gas(data->nh3_ppm,     5.0f,  CONFIG_NH3_THRESHOLD * 2);
    float co_idx      = normalize_gas(data->co_ppm,      5.0f,  CONFIG_CO_THRESHOLD * 2);
    float co2_idx     = normalize_gas(data->co2_ppm,     400.0f, CONFIG_CO2_THRESHOLD * 2);
    float methane_idx = normalize_gas(data->methane_ppm,  100.0f, CONFIG_METHANE_THRESHOLD * 2);
    float h2s_idx     = normalize_gas(data->h2s_ppm,     1.0f,  CONFIG_H2S_THRESHOLD * 2);
    float dust_idx    = normalize_gas(data->dust_ugm3,   12.0f, CONFIG_DUST_THRESHOLD * 2);

    // Weighted combination - NH3 and dust most relevant for poultry
    float aqi = (nh3_idx * 0.25f) + (co_idx * 0.10f) + (co2_idx * 0.20f) +
                (methane_idx * 0.10f) + (h2s_idx * 0.15f) + (dust_idx * 0.20f);

    if (aqi > 500.0f) aqi = 500.0f;
    return (uint16_t)aqi;
}

static aqi_category_t aqi_to_category(uint16_t aqi)
{
    if (aqi <= 50)  return AQI_GOOD;
    if (aqi <= 100) return AQI_MODERATE;
    if (aqi <= 150) return AQI_UNHEALTHY_SENSITIVE;
    if (aqi <= 200) return AQI_UNHEALTHY;
    if (aqi <= 300) return AQI_VERY_UNHEALTHY;
    return AQI_HAZARDOUS;
}

/* ══════════════════════════════════════════════════════════════
 *  Adaptive Ventilation
 *  Adjusts fan speed based on AQI, temperature, and humidity
 * ══════════════════════════════════════════════════════════════ */

static uint8_t compute_adaptive_ventilation(uint16_t aqi, float temp, float humidity,
                                             float pid_output)
{
    // Base fan speed from PID (temperature control)
    float base_speed = pid_output;

    // AQI boost: increase ventilation when air quality is poor
    float aqi_boost = 0.0f;
    if (aqi > 50) {
        aqi_boost = ((float)(aqi - 50) / 450.0f) * 60.0f; // Up to 60% boost
    }

    // Humidity boost: increase ventilation when too humid
    float hum_boost = 0.0f;
    if (humidity > CONFIG_HUMIDITY_MAX) {
        hum_boost = ((humidity - CONFIG_HUMIDITY_MAX) / 30.0f) * 30.0f; // Up to 30% boost
    }

    float total = base_speed + aqi_boost + hum_boost;
    if (total > 100.0f) total = 100.0f;
    if (total < 0.0f) total = 0.0f;

    // Minimum ventilation: always run fans at least 15% for air circulation
    if (total < 15.0f) total = 15.0f;

    return (uint8_t)total;
}

/* ══════════════════════════════════════════════════════════════
 *  Light Schedule Manager
 * ══════════════════════════════════════════════════════════════ */

static uint8_t compute_light_level(int current_hour, int current_minute)
{
    int on_hour = CONFIG_LIGHT_ON_HOUR;
    int off_hour = CONFIG_LIGHT_OFF_HOUR;

    if (current_hour < on_hour || current_hour >= off_hour) {
        return 0; // Lights off
    }

    // Gradual brightening in first hour
    if (current_hour == on_hour) {
        return (uint8_t)((current_minute * 100) / 60);
    }

    // Gradual dimming in last hour
    if (current_hour == off_hour - 1) {
        return (uint8_t)(100 - (current_minute * 100) / 60);
    }

    return 100; // Full brightness
}

/* ══════════════════════════════════════════════════════════════
 *  Anomaly Detection
 * ══════════════════════════════════════════════════════════════ */

typedef struct {
    bool  sensor_failure;
    bool  spike_detected;
    char  message[128];
} anomaly_result_t;

static anomaly_result_t detect_anomalies(const sensor_data_t *data)
{
    anomaly_result_t result = {0};

    // Store temperature history for spike detection
    s_temp_history[s_temp_hist_idx] = data->temperature;
    s_temp_hist_idx = (s_temp_hist_idx + 1) % ANOMALY_HISTORY_SIZE;
    if (s_temp_hist_count < ANOMALY_HISTORY_SIZE) s_temp_hist_count++;

    // Check for sensor failure: stuck values (same reading 30 times)
    if (s_temp_hist_count >= ANOMALY_HISTORY_SIZE) {
        bool all_same = true;
        for (int i = 1; i < ANOMALY_HISTORY_SIZE; i++) {
            if (fabsf(s_temp_history[i] - s_temp_history[0]) > 0.01f) {
                all_same = false;
                break;
            }
        }
        if (all_same) {
            result.sensor_failure = true;
            snprintf(result.message, sizeof(result.message),
                     "Temperature sensor stuck at %.1f°C", data->temperature);
            return result;
        }
    }

    // Check for out-of-range values
    if (data->temperature < -20.0f || data->temperature > 60.0f) {
        result.sensor_failure = true;
        snprintf(result.message, sizeof(result.message),
                 "Temperature out of range: %.1f°C", data->temperature);
        return result;
    }
    if (data->humidity < 0.0f || data->humidity > 100.0f) {
        result.sensor_failure = true;
        snprintf(result.message, sizeof(result.message),
                 "Humidity out of range: %.1f%%", data->humidity);
        return result;
    }

    // Spike detection: sudden change > 5°C in short time
    if (s_temp_hist_count >= 5) {
        int prev_idx = (s_temp_hist_idx - 5 + ANOMALY_HISTORY_SIZE) % ANOMALY_HISTORY_SIZE;
        float diff = fabsf(data->temperature - s_temp_history[prev_idx]);
        if (diff > 5.0f) {
            result.spike_detected = true;
            snprintf(result.message, sizeof(result.message),
                     "Temperature spike: %.1f°C change detected", diff);
        }
    }

    return result;
}

/* ══════════════════════════════════════════════════════════════
 *  Multi-Level Alert System
 * ══════════════════════════════════════════════════════════════ */

static alert_level_t evaluate_alerts(const sensor_data_t *data, uint16_t aqi,
                                      const anomaly_result_t *anomaly)
{
    alert_level_t level = ALERT_LEVEL_NONE;
    char alert_msg[256] = {0};

    float nh3_ratio = safe_ratio(data->nh3_ppm, CONFIG_NH3_THRESHOLD);
    float co_ratio = safe_ratio(data->co_ppm, CONFIG_CO_THRESHOLD);
    float co2_ratio = safe_ratio(data->co2_ppm, CONFIG_CO2_THRESHOLD);
    float ch4_ratio = safe_ratio(data->methane_ppm, CONFIG_METHANE_THRESHOLD);
    float h2s_ratio = safe_ratio(data->h2s_ppm, CONFIG_H2S_THRESHOLD);
    float dust_ratio = safe_ratio(data->dust_ugm3, CONFIG_DUST_THRESHOLD);
    float water_ratio = safe_ratio(data->water_level_pct, CONFIG_WATER_LEVEL_MIN);

    float max_gas_ratio = nh3_ratio;
    const char *max_gas_name = "NH3";
    float max_gas_value = data->nh3_ppm;
    float max_gas_threshold = CONFIG_NH3_THRESHOLD;

    if (co_ratio > max_gas_ratio) {
        max_gas_ratio = co_ratio;
        max_gas_name = "CO";
        max_gas_value = data->co_ppm;
        max_gas_threshold = CONFIG_CO_THRESHOLD;
    }
    if (co2_ratio > max_gas_ratio) {
        max_gas_ratio = co2_ratio;
        max_gas_name = "CO2";
        max_gas_value = data->co2_ppm;
        max_gas_threshold = CONFIG_CO2_THRESHOLD;
    }
    if (ch4_ratio > max_gas_ratio) {
        max_gas_ratio = ch4_ratio;
        max_gas_name = "CH4";
        max_gas_value = data->methane_ppm;
        max_gas_threshold = CONFIG_METHANE_THRESHOLD;
    }
    if (h2s_ratio > max_gas_ratio) {
        max_gas_ratio = h2s_ratio;
        max_gas_name = "H2S";
        max_gas_value = data->h2s_ppm;
        max_gas_threshold = CONFIG_H2S_THRESHOLD;
    }
    if (dust_ratio > max_gas_ratio) {
        max_gas_ratio = dust_ratio;
        max_gas_name = "DUST";
        max_gas_value = data->dust_ugm3;
        max_gas_threshold = CONFIG_DUST_THRESHOLD;
    }

    float temp_high_delta = data->temperature - CONFIG_TEMP_MAX;
    float temp_low_delta = CONFIG_TEMP_MIN - data->temperature;

    if (co_ratio >= 2.0f ||
        h2s_ratio >= 2.0f ||
        temp_high_delta >= 10.0f ||
        temp_low_delta >= 10.0f ||
        aqi >= 450) {
        level = ALERT_LEVEL_EMERGENCY;
        snprintf(alert_msg, sizeof(alert_msg),
                 "EMERGENCY: AQI=%d CO=%.1fx H2S=%.1fx Temp=%.1fC",
                 aqi, co_ratio, h2s_ratio, data->temperature);
    } else if (max_gas_ratio >= 1.5f ||
               aqi > 300 ||
               temp_high_delta >= 5.0f ||
               temp_low_delta >= 5.0f ||
               water_ratio < 0.70f) {
        level = ALERT_LEVEL_CRITICAL;
        snprintf(alert_msg, sizeof(alert_msg),
                 "CRITICAL: %s %.1f/%.1f (%.1fx), AQI=%d Temp=%.1fC Water=%.0f%%",
                 max_gas_name, max_gas_value, max_gas_threshold, max_gas_ratio,
                 aqi, data->temperature, data->water_level_pct);
    } else if (max_gas_ratio >= 1.0f ||
               aqi > 150 ||
               data->temperature > CONFIG_TEMP_MAX ||
               data->temperature < CONFIG_TEMP_MIN ||
               data->water_level_pct < CONFIG_WATER_LEVEL_MIN ||
               anomaly->spike_detected) {
        level = ALERT_LEVEL_WARNING;
        snprintf(alert_msg, sizeof(alert_msg),
                 "WARNING: %s %.1f/%.1f (%.1fx), AQI=%d Temp=%.1fC Water=%.0f%%",
                 max_gas_name, max_gas_value, max_gas_threshold, max_gas_ratio,
                 aqi, data->temperature, data->water_level_pct);
    } else if (aqi > 100 ||
               data->humidity < CONFIG_HUMIDITY_MIN ||
               data->humidity > CONFIG_HUMIDITY_MAX ||
               dust_ratio >= 0.7f) {
        level = ALERT_LEVEL_INFO;
        snprintf(alert_msg, sizeof(alert_msg),
                 "INFO: AQI=%d Hum=%.1f%% Dust=%.1fx",
                 aqi, data->humidity, dust_ratio);
    }

    // Sensor failure always at least WARNING
    if (anomaly->sensor_failure && level < ALERT_LEVEL_WARNING) {
        level = ALERT_LEVEL_WARNING;
        snprintf(alert_msg, sizeof(alert_msg), "SENSOR FAILURE: %s", anomaly->message);
    }

    // Fire callback only for actionable alerts (WARNING+), throttled
    if (level >= ALERT_LEVEL_WARNING && s_alert_cb != NULL &&
        should_emit_notification(level, alert_msg)) {
        s_alert_cb(level, alert_msg);
    }

    if (level < ALERT_LEVEL_WARNING) {
        s_last_notified_level = ALERT_LEVEL_NONE;
        s_last_alert_notify_us = 0;
        s_last_alert_msg[0] = '\0';
    }

    return level;
}

/* ══════════════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════════════ */

esp_err_t control_logic_init(void)
{
    load_runtime_control_config();

    // Initialize PID for temperature -> fan speed
    pid_init(&s_temp_pid, CONFIG_PID_KP, CONFIG_PID_KI, CONFIG_PID_KD, 0.0f, 100.0f);

    // Initialize humidity hysteresis
    hysteresis_init(&s_humidity_hyst, CONFIG_HUMIDITY_MIN, CONFIG_HUMIDITY_MAX,
                    CONFIG_HUMIDITY_DEADBAND);

    // Initialize feed schedule
    s_feed_hours[0] = 7;
    s_feed_hours[1] = 12;
    s_feed_hours[2] = 18;

    s_last_run_us = esp_timer_get_time();
    memset(s_temp_history, 0, sizeof(s_temp_history));
    s_temp_hist_idx = 0;
    s_temp_hist_count = 0;
    s_water_pump_on = false;
    s_last_notified_level = ALERT_LEVEL_NONE;
    s_last_alert_notify_us = 0;
    s_last_alert_msg[0] = '\0';

    ESP_LOGI(TAG, "Control logic initialized (PID: Kp=%.2f Ki=%.2f Kd=%.2f)",
             CONFIG_PID_KP, CONFIG_PID_KI, CONFIG_PID_KD);
    return ESP_OK;
}

esp_err_t control_logic_run(const sensor_data_t *sensor_data, actuator_state_t *actuator_state)
{
    if (sensor_data == NULL || actuator_state == NULL) return ESP_ERR_INVALID_ARG;

    int64_t now_us = esp_timer_get_time();
    float dt = (float)(now_us - s_last_run_us) / 1000000.0f;
    if (dt <= 0.0f) dt = 1.0f;
    if (dt > 10.0f) dt = 10.0f; // Cap dt to avoid integral windup on first run
    s_last_run_us = now_us;

    /* ── 1. Calculate AQI ── */
    s_aqi = calculate_aqi(sensor_data);
    s_aqi_cat = aqi_to_category(s_aqi);

    /* ── 2. Anomaly detection ── */
    anomaly_result_t anomaly = detect_anomalies(sensor_data);
    if (anomaly.sensor_failure) {
        ESP_LOGW(TAG, "Anomaly: %s", anomaly.message);
    }

    /* ── 3. Temperature control via PID ── */
    float pid_output = pid_compute(&s_temp_pid, CONFIG_TEMP_SETPOINT,
                                    sensor_data->temperature, dt);

    // PID output > 0 means too hot -> need cooling (fan)
    // PID output < 0 means too cold -> need heating
    bool need_heating = (sensor_data->temperature < CONFIG_TEMP_MIN);
    bool need_cooling = (sensor_data->temperature > CONFIG_TEMP_SETPOINT + 2.0f);

    actuator_state->heater_on = need_heating;
    actuator_state->cooler_on = need_cooling && (sensor_data->temperature > CONFIG_TEMP_MAX - 2.0f);

    /* ── 4. Adaptive ventilation ── */
    // Convert PID output to fan speed component
    float fan_from_pid = 0;
    if (pid_output > 0) {
        // Too cold: PID positive means we need to heat, reduce fan
        fan_from_pid = 15.0f; // Minimum ventilation
    } else {
        // Too hot: PID negative means we need to cool, increase fan
        fan_from_pid = fabsf(pid_output);
    }
    actuator_state->fan_speed_pct = compute_adaptive_ventilation(
        s_aqi, sensor_data->temperature, sensor_data->humidity, fan_from_pid);

    /* ── 5. Exhaust fan: on when AQI is unhealthy ── */
    actuator_state->exhaust_fan_on = (s_aqi > 150) ||
                                      (sensor_data->nh3_ppm > CONFIG_NH3_THRESHOLD * 0.8f);

    /* ── 6. Humidity control with hysteresis ── */
    // If humidity too low, we might need a humidifier (not modeled, but water pump can help)
    // If humidity too high, increase ventilation (already handled in adaptive vent)
    bool humidity_action = hysteresis_update(&s_humidity_hyst, sensor_data->humidity);
    // humidity_action true means humidity is too low
    (void)humidity_action; // Used indirectly through ventilation boost

    /* ── 7. Light schedule ── */
    // Get approximate hour from uptime (in real system, use SNTP)
    // For now, use a simple counter based on esp_timer
    int simulated_hour = (int)((now_us / 1000000 / 3600) % 24);
    int simulated_minute = (int)((now_us / 1000000 / 60) % 60);
    actuator_state->light_level_pct = compute_light_level(simulated_hour, simulated_minute);

    // Override: if ambient light is already sufficient, reduce artificial light
    if (sensor_data->light_lux > 300.0f && actuator_state->light_level_pct > 0) {
        float reduction = (sensor_data->light_lux - 300.0f) / 500.0f;
        if (reduction > 1.0f) reduction = 1.0f;
        actuator_state->light_level_pct = (uint8_t)(actuator_state->light_level_pct * (1.0f - reduction * 0.7f));
    }

    /* ── 8. Water management ── */
    if (sensor_data->water_level_pct < CONFIG_WATER_LEVEL_MIN) {
        s_water_pump_on = true;
    } else if (sensor_data->water_level_pct > CONFIG_WATER_LEVEL_MAX) {
        s_water_pump_on = false;
    }
    // Between min and max: maintain current state (hysteresis)
    actuator_state->water_pump_on = s_water_pump_on;

    /* ── 9. Feed scheduling ── */
    actuator_state->feed_dispenser_on = false;
    for (int i = 0; i < CONFIG_FEED_TIMES_PER_DAY && i < 6; i++) {
        if (simulated_hour == s_feed_hours[i] && simulated_minute < 5 &&
            s_last_feed_hour != simulated_hour) {
            actuator_state->feed_dispenser_on = true;
            s_last_feed_hour = simulated_hour;
            ESP_LOGI(TAG, "Feed time! Dispensing at hour %d", simulated_hour);
            break;
        }
    }

    /* ── 10. Door control ── */
    // Open door during daytime, close at night; close on emergency
    if (s_alert_level >= ALERT_LEVEL_EMERGENCY || sensor_data->door_open) {
        actuator_state->door_servo_angle = 0; // Close
    } else if (simulated_hour >= CONFIG_LIGHT_ON_HOUR && simulated_hour < CONFIG_LIGHT_OFF_HOUR) {
        actuator_state->door_servo_angle = 90; // Open during day
    } else {
        actuator_state->door_servo_angle = 0; // Closed at night
    }

    /* ── 11. Alert evaluation ── */
    s_alert_level = evaluate_alerts(sensor_data, s_aqi, &anomaly);

    /* ── 12. Alarm based on alert level ── */
    switch (s_alert_level) {
        case ALERT_LEVEL_EMERGENCY:
            actuator_state->alarm_pattern = ALARM_PATTERN_SOS;
            // Emergency: max ventilation, open everything
            actuator_state->fan_speed_pct = 100;
            actuator_state->exhaust_fan_on = true;
            break;
        case ALERT_LEVEL_CRITICAL:
            actuator_state->alarm_pattern = ALARM_PATTERN_CONTINUOUS;
            actuator_state->fan_speed_pct = (actuator_state->fan_speed_pct < 80) ?
                                             80 : actuator_state->fan_speed_pct;
            actuator_state->exhaust_fan_on = true;
            break;
        case ALERT_LEVEL_WARNING:
            actuator_state->alarm_pattern = ALARM_PATTERN_INTERMITTENT;
            break;
        default:
            actuator_state->alarm_pattern = ALARM_PATTERN_OFF;
            break;
    }

    ESP_LOGD(TAG, "Control: AQI=%d Fan=%d%% Heater=%d Cooler=%d Light=%d%% Alert=%d",
             s_aqi, actuator_state->fan_speed_pct, actuator_state->heater_on,
             actuator_state->cooler_on, actuator_state->light_level_pct, s_alert_level);

    return ESP_OK;
}

uint16_t control_logic_get_aqi(void)
{
    return s_aqi;
}

aqi_category_t control_logic_get_aqi_category(void)
{
    return s_aqi_cat;
}

alert_level_t control_logic_get_alert_level(void)
{
    return s_alert_level;
}

void control_logic_register_alert_callback(alert_callback_t cb)
{
    s_alert_cb = cb;
}
