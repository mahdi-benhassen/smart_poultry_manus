#include "control_logic.h"
#include "actuator_alarm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "CTRL_LOGIC";

/* ══════════════════════════════════════════════════════════════
 *  Configuration defaults (overridable via Kconfig / NVS)
 * ══════════════════════════════════════════════════════════════ */

#ifndef CONFIG_TEMP_SETPOINT
#define CONFIG_TEMP_SETPOINT        25.0f
#endif
#ifndef CONFIG_TEMP_MIN
#define CONFIG_TEMP_MIN             18.0f
#endif
#ifndef CONFIG_TEMP_MAX
#define CONFIG_TEMP_MAX             32.0f
#endif
#ifndef CONFIG_HUMIDITY_MIN
#define CONFIG_HUMIDITY_MIN         50.0f
#endif
#ifndef CONFIG_HUMIDITY_MAX
#define CONFIG_HUMIDITY_MAX         70.0f
#endif
#ifndef CONFIG_HUMIDITY_DEADBAND
#define CONFIG_HUMIDITY_DEADBAND    3.0f
#endif
#ifndef CONFIG_NH3_THRESHOLD
#define CONFIG_NH3_THRESHOLD        25.0f
#endif
#ifndef CONFIG_CO_THRESHOLD
#define CONFIG_CO_THRESHOLD         50.0f
#endif
#ifndef CONFIG_CO2_THRESHOLD
#define CONFIG_CO2_THRESHOLD        2500.0f
#endif
#ifndef CONFIG_METHANE_THRESHOLD
#define CONFIG_METHANE_THRESHOLD    1000.0f
#endif
#ifndef CONFIG_H2S_THRESHOLD
#define CONFIG_H2S_THRESHOLD        10.0f
#endif
#ifndef CONFIG_DUST_THRESHOLD
#define CONFIG_DUST_THRESHOLD       150.0f
#endif
#ifndef CONFIG_WATER_LEVEL_MIN
#define CONFIG_WATER_LEVEL_MIN      20.0f
#endif
#ifndef CONFIG_WATER_LEVEL_MAX
#define CONFIG_WATER_LEVEL_MAX      80.0f
#endif
#ifndef CONFIG_SOUND_ALARM_DB
#define CONFIG_SOUND_ALARM_DB       85.0f
#endif

// PID defaults for temperature
#ifndef CONFIG_PID_KP
#define CONFIG_PID_KP               5.0f
#endif
#ifndef CONFIG_PID_KI
#define CONFIG_PID_KI               0.1f
#endif
#ifndef CONFIG_PID_KD
#define CONFIG_PID_KD               1.0f
#endif

// Light schedule (hours in 24h format)
#ifndef CONFIG_LIGHT_ON_HOUR
#define CONFIG_LIGHT_ON_HOUR        6
#endif
#ifndef CONFIG_LIGHT_OFF_HOUR
#define CONFIG_LIGHT_OFF_HOUR       22
#endif

// Feed schedule
#ifndef CONFIG_FEED_TIMES_PER_DAY
#define CONFIG_FEED_TIMES_PER_DAY   3
#endif
#ifndef CONFIG_FEED_DURATION_MS
#define CONFIG_FEED_DURATION_MS     5000
#endif

/* ══════════════════════════════════════════════════════════════
 *  Internal state
 * ══════════════════════════════════════════════════════════════ */

static pid_controller_t s_temp_pid;
static uint16_t s_aqi = 0;
static aqi_category_t s_aqi_cat = AQI_GOOD;
static alert_level_t s_alert_level = ALERT_LEVEL_NONE;
static alert_callback_t s_alert_cb = NULL;
static int64_t s_last_run_us = 0;

// Anomaly detection state
#define ANOMALY_HISTORY_SIZE 30
static float s_temp_history[ANOMALY_HISTORY_SIZE];
static int s_temp_hist_idx = 0;
static int s_temp_hist_count = 0;

// Feed schedule tracking
static int s_feed_hours[6] = {7, 12, 18, 0, 0, 0};
static int s_last_feed_hour = -1;

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

    // EMERGENCY: life-threatening conditions
    if (data->co_ppm > CONFIG_CO_THRESHOLD * 2 ||
        data->h2s_ppm > CONFIG_H2S_THRESHOLD * 2 ||
        data->temperature > CONFIG_TEMP_MAX + 10) {
        level = ALERT_LEVEL_EMERGENCY;
        snprintf(alert_msg, sizeof(alert_msg),
                 "EMERGENCY: CO=%.0f H2S=%.1f Temp=%.1f",
                 data->co_ppm, data->h2s_ppm, data->temperature);
    }
    // CRITICAL: dangerous conditions
    else if (data->nh3_ppm > CONFIG_NH3_THRESHOLD * 1.5f ||
             data->co_ppm > CONFIG_CO_THRESHOLD * 1.5f ||
             aqi > 300 ||
             data->temperature > CONFIG_TEMP_MAX + 5 ||
             data->temperature < CONFIG_TEMP_MIN - 5) {
        level = ALERT_LEVEL_CRITICAL;
        snprintf(alert_msg, sizeof(alert_msg),
                 "CRITICAL: AQI=%d Temp=%.1f NH3=%.0f",
                 aqi, data->temperature, data->nh3_ppm);
    }
    // WARNING: conditions need attention
    else if (data->nh3_ppm > CONFIG_NH3_THRESHOLD ||
             data->co2_ppm > CONFIG_CO2_THRESHOLD ||
             aqi > 150 ||
             data->temperature > CONFIG_TEMP_MAX ||
             data->temperature < CONFIG_TEMP_MIN ||
             data->water_level_pct < CONFIG_WATER_LEVEL_MIN ||
             anomaly->spike_detected) {
        level = ALERT_LEVEL_WARNING;
        snprintf(alert_msg, sizeof(alert_msg),
                 "WARNING: AQI=%d Temp=%.1f Water=%.0f%%",
                 aqi, data->temperature, data->water_level_pct);
    }
    // INFO: minor issues
    else if (aqi > 100 ||
             data->humidity < CONFIG_HUMIDITY_MIN ||
             data->humidity > CONFIG_HUMIDITY_MAX ||
             data->dust_ugm3 > CONFIG_DUST_THRESHOLD * 0.7f) {
        level = ALERT_LEVEL_INFO;
        snprintf(alert_msg, sizeof(alert_msg),
                 "INFO: AQI=%d Humidity=%.1f%% Dust=%.0f",
                 aqi, data->humidity, data->dust_ugm3);
    }

    // Sensor failure always at least WARNING
    if (anomaly->sensor_failure && level < ALERT_LEVEL_WARNING) {
        level = ALERT_LEVEL_WARNING;
        snprintf(alert_msg, sizeof(alert_msg), "SENSOR FAILURE: %s", anomaly->message);
    }

    // Fire callback
    if (level > ALERT_LEVEL_NONE && s_alert_cb != NULL) {
        s_alert_cb(level, alert_msg);
    }

    return level;
}

/* ══════════════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════════════ */

esp_err_t control_logic_init(void)
{
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
        actuator_state->water_pump_on = true;
    } else if (sensor_data->water_level_pct > CONFIG_WATER_LEVEL_MAX) {
        actuator_state->water_pump_on = false;
    }
    // Between min and max: maintain current state (hysteresis)

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
