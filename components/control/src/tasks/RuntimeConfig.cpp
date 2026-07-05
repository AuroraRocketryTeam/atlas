#include "RuntimeConfig.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "IBoardHardware.hpp"
#include "config.h"
#include "esp_log.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "RuntimeConfig";
static const char *NVS_NAMESPACE = "runtime_cfg";
static const char *NVS_KEY_ACTIVE = "active";

static RuntimeConfig s_active_config;
static RuntimeConfig s_flight_snapshot;
static IBoardHardware *s_board = nullptr;
static bool s_loaded = false;
static bool s_flight_snapshot_valid = false;

/*
 * Nominal RuntimeConfig workflow:
 * 1. GroundServicesTask initializes board-owned NVS, then calls
 *    runtime_config_init(board).
 * 2. runtime_config_init() loads the active NVS blob. Empty, corrupt, or
 *    unsupported data is replaced with config.h-backed firmware defaults.
 * 3. The dashboard reads runtime_config_to_json(), runtime_config_schema_json(),
 *    and runtime_config_validation_json() to render editable mission fields,
 *    read-only compiled constants, board hardware metadata, and validation
 *    status.
 * 4. Dashboard edits are applied by runtime_config_update_from_json(), which
 *    accepts only editable RuntimeConfig fields and persists the whole blob
 *    back to NVS with a checksum.
 * 5. Before leaving Ground Services, runtime_config_lock_for_flight() validates
 *    the active config, persists the lock bit, and copies an immutable flight
 *    snapshot for flight tasks.
 */

/** TODO: Important parameters still not exposed but worth considering:
 *    - Airbrakes: AIRBRAKES_OPEN_ALTITUDE_M, AIRBRAKES_CLOSE_ALTITUDE_M, open/close rates.
 *    - Altitude safety/filter internals: MAX_DELTA_P_PER_TICK, apogee detector sample rate, apogee trigger velocity -0.5f, selected barometer BARO_1.
 *    - Calibration: FSM calibration timeout 10000U, REQUIRED_BARO_SAMPLES, REQUIRED_TEMPERATURE_SAMPLES, SENSOR_LOOKUP_MAX_ATTEMPTS, SENSOR_LOOKUP_TIMEOUT.
 *    - GPS init: GPS_FIX_LOOKUP_INTERVAL_MS is not exposed, while GPS_FIX_TIMEOUT_MS and GPS_MIN_FIX are.
 */

/**
 * @brief Build the firmware defaults for editable mission settings.
 *
 * These defaults come from config.h because they are shared with legacy flight
 * code and represent the firmware's compiled baseline. RuntimeConfig persists
 * operator edits in NVS under NVS_NAMESPACE/NVS_KEY_ACTIVE, validates them
 * against the schema emitted by runtime_config_schema_json(), and falls back to
 * these defaults if NVS is empty, corrupt, or from an unsupported schema.
 */
static RuntimeConfig make_default_config()
{
    RuntimeConfig cfg = {};
    cfg.schema_version = 2;
    cfg.config_revision = 1;
    snprintf(cfg.rocket_name, sizeof(cfg.rocket_name), "%s", "Atlas Manny");
    snprintf(cfg.launch_site, sizeof(cfg.launch_site), "%s", "Launch Site");
    snprintf(cfg.operator_note, sizeof(cfg.operator_note), "%s", "Ground services runtime config");
    cfg.launch_site_altitude_m = SEA_LEVEL;
    cfg.sea_level_pressure_hpa = SEA_LEVEL_PRESSURE_HPA;
    cfg.liftoff_accel_threshold_mps2 = LIFTOFF_ACCELERATION_THRESHOLD;
    cfg.liftoff_timeout_ms = LIFTOFF_TIMEOUT_MS;
    cfg.apogee_lockout_ms = APOGEE_LOCKOUT_MS;
    cfg.drogue_apogee_timeout_ms = DROGUE_APOGEE_TIMEOUT;
    cfg.main_altitude_threshold_m = MAIN_ALTITUDE_THRESHOLD;
    cfg.touchdown_velocity_threshold_mps = TOUCHDOWN_VELOCITY_THRESHOLD;
    cfg.launch_to_ballistic_threshold_ms = LAUNCH_TO_BALLISTIC_THRESHOLD;
    cfg.launch_to_apogee_threshold_ms = LAUNCH_TO_APOGEE_THRESHOLD;
    cfg.touchdown_altitude_threshold_m = TOUCHDOWN_ALTITUDE_THRESHOLD;
    cfg.telemetry_enabled = true;
    cfg.telemetry_period_ms = TELEMETRY_INTERVAL_MS;
    cfg.logging_enabled = true;
    cfg.config_locked = false;
    return cfg;
}

static int gpio_to_int(gpio_num_t pin)
{
    return static_cast<int>(pin);
}

static const char *recovery_mode_to_string(RecoveryMode mode)
{
    switch (mode) {
        case RecoveryMode::OneParachuteMode:
            return "one_parachute";
        case RecoveryMode::TwoParachuteMode:
            return "two_parachute";
        default:
            return "unknown";
    }
}

const RuntimeConfig &runtime_config_defaults()
{
    static RuntimeConfig defaults = make_default_config();
    return defaults;
}

/**
 * @brief Small deterministic checksum used to detect stale/corrupt NVS blobs.
 *
 * The checksum is integrity metadata, not a security mechanism. It catches
 * partial writes, schema-size mismatches, and accidental corruption before the
 * config can be activated.
 */
static uint32_t fnv1a32(const void *data, size_t len)
{
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        hash ^= bytes[i];
        hash *= 16777619u;
    }
    return hash;
}

static uint32_t compute_checksum(RuntimeConfig cfg)
{
    cfg.checksum = 0;
    return fnv1a32(&cfg, sizeof(cfg));
}

static void update_checksum(RuntimeConfig *cfg)
{
    if (cfg != nullptr) {
        cfg->checksum = compute_checksum(*cfg);
    }
}

static bool checksum_ok(const RuntimeConfig *cfg)
{
    return cfg != nullptr && cfg->checksum == compute_checksum(*cfg);
}

/**
 * @brief Write a validation error into a caller-owned buffer and return failure.
 */
static esp_err_t validation_fail(char *reason, size_t reason_size, const char *msg)
{
    if (reason != nullptr && reason_size > 0) {
        snprintf(reason, reason_size, "%s", msg);
    }
    return ESP_ERR_INVALID_ARG;
}

struct ValidationAccumulator {
    bool has_error;
    char *reason;
    size_t reason_size;
};

/**
 * @brief Adapter that turns multi-item validation into the first error string.
 */
static void add_validation_item(const RuntimeConfigValidationItem &item, void *ctx)
{
    auto *acc = static_cast<ValidationAccumulator *>(ctx);
    if (acc == nullptr) return;
    if (strcmp(item.severity, "error") == 0) {
        acc->has_error = true;
        if (acc->reason != nullptr && acc->reason_size > 0 && acc->reason[0] == '\0') {
            snprintf(acc->reason, acc->reason_size, "%s", item.message);
        }
    }
}

/**
 * @brief Emit one validation item when the caller requested itemized output.
 */
static void emit_item(RuntimeConfigValidationCallback cb, void *ctx, const char *field, const char *severity, const char *message)
{
    if (cb == nullptr) return;
    RuntimeConfigValidationItem item{field, severity, message};
    cb(item, ctx);
}

esp_err_t runtime_config_validate_each(const RuntimeConfig *cfg, RuntimeConfigValidationCallback cb, void *ctx)
{
    if (cfg == nullptr) {
        emit_item(cb, ctx, "", "error", "config pointer is null");
        return ESP_ERR_INVALID_ARG;
    }

    bool has_error = false;
    auto error = [&](const char *field, const char *message) {
        has_error = true;
        emit_item(cb, ctx, field, "error", message);
    };

    // Keep validation ranges aligned with runtime_config_schema_json(). The
    // dashboard uses schema ranges for client-side checks, but firmware repeats
    // validation here because NVS and HTTP input cannot be trusted.
    if (cfg->schema_version != 2) error("schema_version", "Unsupported runtime config schema");
    if (cfg->rocket_name[0] == '\0') error("identity.rocket_name", "Rocket name is required");
    if (cfg->launch_site[0] == '\0') error("identity.launch_site", "Launch site is required");
    if (!std::isfinite(cfg->launch_site_altitude_m) || cfg->launch_site_altitude_m < 0.0f || cfg->launch_site_altitude_m > 6000.0f) {
        error("environment.launch_site_altitude_m", "Launch site altitude must be between 0 and 6000 m");
    }
    if (!std::isfinite(cfg->sea_level_pressure_hpa) || cfg->sea_level_pressure_hpa < 850.0f || cfg->sea_level_pressure_hpa > 1100.0f) {
        error("environment.sea_level_pressure_hpa", "Sea-level pressure must be between 850 and 1100 hPa");
    }
    if (!std::isfinite(cfg->liftoff_accel_threshold_mps2) || cfg->liftoff_accel_threshold_mps2 < 10.0f || cfg->liftoff_accel_threshold_mps2 > 150.0f) {
        error("flight.liftoff_accel_threshold_mps2", "Liftoff acceleration threshold must be between 10 and 150 m/s^2");
    }
    if (cfg->liftoff_timeout_ms < 20 || cfg->liftoff_timeout_ms > 10000) {
        error("flight.liftoff_timeout_ms", "Liftoff timeout must be between 20 and 10000 ms");
    }
    if (cfg->apogee_lockout_ms > 120000) error("flight.apogee_lockout_ms", "Apogee lockout must be at most 120000 ms");
    if (cfg->drogue_apogee_timeout_ms > 120000) error("flight.drogue_apogee_timeout_ms", "Drogue apogee timeout must be at most 120000 ms");
    if (!std::isfinite(cfg->main_altitude_threshold_m) || cfg->main_altitude_threshold_m < 50.0f || cfg->main_altitude_threshold_m > 1000.0f) {
        error("flight.main_altitude_threshold_m", "Main altitude must be between 50 and 1000 m");
    }
    if (!std::isfinite(cfg->touchdown_velocity_threshold_mps) || cfg->touchdown_velocity_threshold_mps < 0.1f || cfg->touchdown_velocity_threshold_mps > 20.0f) {
        error("flight.touchdown_velocity_threshold_mps", "Touchdown velocity threshold must be between 0.1 and 20 m/s");
    }
    if (cfg->launch_to_ballistic_threshold_ms < 1 || cfg->launch_to_ballistic_threshold_ms > 300000) {
        error("flight.launch_to_ballistic_threshold_ms", "Launch-to-ballistic timeout is outside firmware limits");
    }
    if (cfg->launch_to_apogee_threshold_ms < 1 || cfg->launch_to_apogee_threshold_ms > 300000) {
        error("flight.launch_to_apogee_threshold_ms", "Launch-to-apogee timeout is outside firmware limits");
    }
    if (!std::isfinite(cfg->touchdown_altitude_threshold_m) || cfg->touchdown_altitude_threshold_m < 0.0f || cfg->touchdown_altitude_threshold_m > 200.0f) {
        error("flight.touchdown_altitude_threshold_m", "Touchdown altitude threshold must be between 0 and 200 m");
    }
    if (cfg->telemetry_period_ms < 100 || cfg->telemetry_period_ms > 10000) {
        error("telemetry.telemetry_period_ms", "Telemetry period must be between 100 and 10000 ms");
    }
    if (!cfg->telemetry_enabled) error("telemetry.telemetry_enabled", "Telemetry must remain enabled during flight");
    if (!cfg->logging_enabled) error("logging.logging_enabled", "Logging must remain enabled during flight");

    return has_error ? ESP_ERR_INVALID_ARG : ESP_OK;
}

/**
 * @brief Validate a config and collapse itemized validation to one reason.
 */
esp_err_t runtime_config_validate(const RuntimeConfig *cfg, char *reason, size_t reason_size)
{
    if (reason != nullptr && reason_size > 0) reason[0] = '\0';
    ValidationAccumulator acc{};
    acc.reason = reason;
    acc.reason_size = reason_size;
    runtime_config_validate_each(cfg, add_validation_item, &acc);
    if (acc.has_error) return ESP_ERR_INVALID_ARG;
    if (reason != nullptr && reason_size > 0) snprintf(reason, reason_size, "%s", "ok");
    return ESP_OK;
}

/**
 * @brief Persist a complete validated config snapshot to NVS.
 */
esp_err_t runtime_config_save(const RuntimeConfig *cfg)
{
    if (cfg == nullptr) return ESP_ERR_INVALID_ARG;

    // Persist whole-struct snapshots. This keeps the NVS contract simple and
    // lets checksum/schema validation reject old or partially written blobs.
    RuntimeConfig copy = *cfg;
    char reason[96];
    esp_err_t err = runtime_config_validate(&copy, reason, sizeof(reason));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Refusing invalid config: %s", reason);
        return err;
    }
    update_checksum(&copy);

    nvs_handle_t handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, NVS_KEY_ACTIVE, &copy, sizeof(copy));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        s_active_config = copy;
        s_loaded = true;
    }
    return err;
}

/**
 * @brief Initialize the active config cache from NVS.
 */
esp_err_t runtime_config_init(IBoardHardware *board)
{
    // RuntimeConfig does not own the board. It only uses the interface to expose
    // read-only hardware metadata in JSON responses.
    s_board = board;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        RuntimeConfig cfg = runtime_config_defaults();
        return runtime_config_save(&cfg);
    }

    RuntimeConfig cfg = {};
    size_t size = sizeof(cfg);
    err = nvs_get_blob(handle, NVS_KEY_ACTIVE, &cfg, &size);
    nvs_close(handle);

    char reason[96];
    if (err != ESP_OK || size != sizeof(cfg) || !checksum_ok(&cfg) || runtime_config_validate(&cfg, reason, sizeof(reason)) != ESP_OK) {
        RuntimeConfig defaults = runtime_config_defaults();
        return runtime_config_save(&defaults);
    }

    s_active_config = cfg;
    s_loaded = true;
    if (cfg.config_locked) {
        s_flight_snapshot = cfg;
        s_flight_snapshot_valid = true;
    }
    return ESP_OK;
}

/**
 * @brief Copy the active config cache for callers that need a mutable value.
 */
esp_err_t runtime_config_get(RuntimeConfig *out)
{
    if (out == nullptr) return ESP_ERR_INVALID_ARG;
    if (!s_loaded) return ESP_ERR_INVALID_STATE;
    *out = s_active_config;
    return ESP_OK;
}

/**
 * @brief Check whether dashboard editing is disabled for flight.
 */
bool runtime_config_is_locked()
{
    return s_loaded && s_active_config.config_locked;
}

/**
 * @brief Return the task-facing config values.
 */
const RuntimeConfig &runtime_config_get_flight_snapshot()
{
    // Flight tasks should use this accessor after READY_FOR_LAUNCH so they read
    // the locked launch snapshot, not a later dashboard edit or recovery unlock.
    if (s_flight_snapshot_valid) return s_flight_snapshot;
    return s_loaded ? s_active_config : runtime_config_defaults();
}

/**
 * @brief Restore config.h-backed defaults unless the config is locked.
 */
esp_err_t runtime_config_reset_defaults()
{
    if (runtime_config_is_locked()) return ESP_ERR_INVALID_STATE;
    RuntimeConfig cfg = runtime_config_defaults();
    cfg.config_revision = s_loaded ? s_active_config.config_revision + 1 : 1;
    return runtime_config_save(&cfg);
}

/**
 * @brief Clear the lock bit after the vehicle is no longer in active flight.
 */
esp_err_t runtime_config_unlock_after_recovery()
{
    if (!s_loaded) return ESP_ERR_INVALID_STATE;

    RuntimeConfig cfg = s_active_config;
    cfg.config_locked = false;
    cfg.config_revision++;

    esp_err_t err = runtime_config_save(&cfg);
    if (err == ESP_OK) {
        s_flight_snapshot_valid = false;
    }
    return err;
}

static const char *find_key(const char *json, const char *key)
{
    // Minimal parser for flat dashboard JSON. RuntimeConfig only accepts scalar
    // top-level fields; nested objects and arrays are intentionally ignored.
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (p == nullptr) return nullptr;
    p += strlen(pattern);
    while (*p != '\0' && std::isspace(static_cast<unsigned char>(*p))) ++p;
    if (*p != ':') return nullptr;
    ++p;
    while (*p != '\0' && std::isspace(static_cast<unsigned char>(*p))) ++p;
    return p;
}

/**
 * @brief Read a flat JSON string field into a fixed-size C buffer.
 */
static bool json_get_string(const char *json, const char *key, char *out, size_t out_size)
{
    const char *p = find_key(json, key);
    if (p == nullptr || *p != '"' || out_size == 0) return false;
    ++p;
    size_t i = 0;
    while (*p != '\0' && *p != '"' && i + 1 < out_size) out[i++] = *p++;
    out[i] = '\0';
    return *p == '"';
}

/**
 * @brief Read a flat JSON floating-point field.
 */
static bool json_get_float(const char *json, const char *key, float *out)
{
    const char *p = find_key(json, key);
    if (p == nullptr || out == nullptr) return false;
    char *end = nullptr;
    float value = strtof(p, &end);
    if (end == p) return false;
    *out = value;
    return true;
}

/**
 * @brief Update a float only when the JSON field is present.
 */
static esp_err_t json_update_float_if_present(const char *json, const char *key, float *out)
{
    if (find_key(json, key) == nullptr) return ESP_OK;
    return json_get_float(json, key, out) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

/**
 * @brief Read an unsigned integer field and reject negative values.
 */
static bool json_get_uint(const char *json, const char *key, uint32_t *out)
{
    const char *p = find_key(json, key);
    if (p == nullptr || out == nullptr) return false;
    if (*p == '-') return false;
    char *end = nullptr;
    unsigned long value = strtoul(p, &end, 10);
    if (end == p) return false;
    *out = static_cast<uint32_t>(value);
    return true;
}

/**
 * @brief Update an integer only when the JSON field is present.
 */
static esp_err_t json_update_uint_if_present(const char *json, const char *key, uint32_t *out)
{
    if (find_key(json, key) == nullptr) return ESP_OK;
    return json_get_uint(json, key, out) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

/**
 * @brief Apply dashboard edits to the active config.
 */
esp_err_t runtime_config_update_from_json(const char *json, RuntimeConfig *updated_out)
{
    if (json == nullptr) return ESP_ERR_INVALID_ARG;
    RuntimeConfig cfg;
    esp_err_t err = runtime_config_get(&cfg);
    if (err != ESP_OK) return err;
    if (cfg.config_locked) return ESP_ERR_INVALID_STATE;

    // Only editable RuntimeConfig fields are copied from JSON. Read-only values
    // such as recovery_mode, telemetry addresses, board pins, and compiled
    // algorithm constants are returned by runtime_config_to_json() but ignored
    // on update.
    json_get_string(json, "rocket_name", cfg.rocket_name, sizeof(cfg.rocket_name));
    json_get_string(json, "launch_site", cfg.launch_site, sizeof(cfg.launch_site));
    json_get_string(json, "operator_note", cfg.operator_note, sizeof(cfg.operator_note));
    if (json_update_float_if_present(json, "launch_site_altitude_m", &cfg.launch_site_altitude_m) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_float_if_present(json, "sea_level_pressure_hpa", &cfg.sea_level_pressure_hpa) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_float_if_present(json, "liftoff_accel_threshold_mps2", &cfg.liftoff_accel_threshold_mps2) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_uint_if_present(json, "liftoff_timeout_ms", &cfg.liftoff_timeout_ms) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_uint_if_present(json, "apogee_lockout_ms", &cfg.apogee_lockout_ms) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_uint_if_present(json, "drogue_apogee_timeout_ms", &cfg.drogue_apogee_timeout_ms) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_float_if_present(json, "main_altitude_threshold_m", &cfg.main_altitude_threshold_m) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_float_if_present(json, "touchdown_velocity_threshold_mps", &cfg.touchdown_velocity_threshold_mps) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_uint_if_present(json, "launch_to_ballistic_threshold_ms", &cfg.launch_to_ballistic_threshold_ms) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_uint_if_present(json, "launch_to_apogee_threshold_ms", &cfg.launch_to_apogee_threshold_ms) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_float_if_present(json, "touchdown_altitude_threshold_m", &cfg.touchdown_altitude_threshold_m) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (json_update_uint_if_present(json, "telemetry_period_ms", &cfg.telemetry_period_ms) != ESP_OK) return ESP_ERR_INVALID_ARG;
    cfg.telemetry_enabled = true;
    cfg.logging_enabled = true;
    cfg.config_revision++;

    err = runtime_config_save(&cfg);
    if (err == ESP_OK && updated_out != nullptr) *updated_out = cfg;
    return err;
}

/**
 * @brief Build the dashboard runtime/config response.
 */
esp_err_t runtime_config_to_json(const RuntimeConfig *cfg, char *out, size_t out_size)
{
    if (cfg == nullptr || out == nullptr) return ESP_ERR_INVALID_ARG;

    const char *board_name = s_board != nullptr ? s_board->get_board_name() : "unknown";
    const int i2c_sda_pin = s_board != nullptr ? gpio_to_int(s_board->get_i2c_sda_pin()) : -1;
    const int i2c_scl_pin = s_board != nullptr ? gpio_to_int(s_board->get_i2c_scl_pin()) : -1;
    const int spi_mosi_pin = s_board != nullptr ? gpio_to_int(s_board->get_spi_mosi_pin()) : -1;
    const int spi_miso_pin = s_board != nullptr ? gpio_to_int(s_board->get_spi_miso_pin()) : -1;
    const int spi_clk_pin = s_board != nullptr ? gpio_to_int(s_board->get_spi_clk_pin()) : -1;
    const int flash_cs_pin = s_board != nullptr ? gpio_to_int(s_board->get_flash_cs_pin()) : -1;
    const int barometer_cs_pin = s_board != nullptr ? gpio_to_int(s_board->get_barometer_cs_pin()) : -1;
    const int barometer2_cs_pin = s_board != nullptr ? gpio_to_int(s_board->get_barometer2_cs_pin()) : -1;
    const int lora_aux_pin = s_board != nullptr ? gpio_to_int(s_board->get_lora_aux_pin()) : -1;
    const int lora_tx_pin = s_board != nullptr ? gpio_to_int(s_board->get_lora_tx_pin()) : -1;
    const int lora_rx_pin = s_board != nullptr ? gpio_to_int(s_board->get_lora_rx_pin()) : -1;
    const int lora_m0_pin = s_board != nullptr ? gpio_to_int(s_board->get_lora_m0_pin()) : -1;
    const int lora_m1_pin = s_board != nullptr ? gpio_to_int(s_board->get_lora_m1_pin()) : -1;
    const int main_actuator_pin = s_board != nullptr ? gpio_to_int(s_board->get_main_actuator_pin()) : -1;
    const int drogue_actuator_pin = s_board != nullptr ? gpio_to_int(s_board->get_drogue_actuator_pin()) : -1;
    const unsigned bno055_i2c_addr = s_board != nullptr ? static_cast<unsigned>(s_board->get_bno055_i2c_address()) : 0;
    const char *recovery_mode = recovery_mode_to_string(AURORA_RECOVERY_MODE);

    int n = snprintf(out, out_size,
        "{\"ok\":true,\"schema_version\":%lu,\"config_revision\":%lu,"
        "\"config_locked\":%s,"
        "\"rocket_name\":\"%s\",\"launch_site\":\"%s\",\"operator_note\":\"%s\","
        "\"launch_site_altitude_m\":%.3f,\"sea_level_pressure_hpa\":%.3f,"
        "\"liftoff_accel_threshold_mps2\":%.3f,\"liftoff_timeout_ms\":%lu,"
        "\"apogee_lockout_ms\":%lu,\"drogue_apogee_timeout_ms\":%lu,"
        "\"main_altitude_threshold_m\":%.3f,\"touchdown_velocity_threshold_mps\":%.3f,"
        "\"launch_to_ballistic_threshold_ms\":%lu,\"launch_to_apogee_threshold_ms\":%lu,"
        "\"touchdown_altitude_threshold_m\":%.3f,"
        "\"telemetry_enabled\":%s,\"telemetry_period_ms\":%lu,"
        "\"logging_enabled\":%s,"
        "\"ms56_i2c_addr_1\":\"0x%02X\",\"ms56_i2c_addr_2\":\"0x%02X\",\"bno055_i2c_addr\":\"0x%02X\","
        "\"espnow_channel\":%u,\"espnow_peer_mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"lora_channel\":%u,\"lora_addh\":\"0x%02X\",\"lora_addl\":\"0x%02X\",\"lora_receiver_addh\":\"0x%02X\",\"lora_receiver_addl\":\"0x%02X\","
        "\"recovery_mode\":\"%s\","
        "\"altitude_filter_window\":%u,\"apogee_detection_window_size\":%u,"
        "\"num_calibration_samples\":%u,\"std_threshold\":%.3f,\"gps_fix_timeout_ms\":%lu,\"gps_min_fix\":%u,"
        "\"imu_minimum_calibration\":%u,\"h_bias_pressure_sensor\":%.3f,\"gps_bias\":%.3f,"
        "\"batch_size\":%u,\"max_log_entries\":%u,"
        "\"board_name\":\"%s\",\"i2c_sda_pin\":%d,\"i2c_scl_pin\":%d,"
        "\"spi_mosi_pin\":%d,\"spi_miso_pin\":%d,\"spi_clk_pin\":%d,"
        "\"flash_cs_pin\":%d,\"barometer_cs_pin\":%d,\"barometer2_cs_pin\":%d,"
        "\"lora_aux_pin\":%d,\"lora_tx_pin\":%d,\"lora_rx_pin\":%d,\"lora_m0_pin\":%d,\"lora_m1_pin\":%d,"
        "\"main_actuator_pin\":%d,\"drogue_actuator_pin\":%d}",
        static_cast<unsigned long>(cfg->schema_version),
        static_cast<unsigned long>(cfg->config_revision),
        cfg->config_locked ? "true" : "false",
        cfg->rocket_name, cfg->launch_site, cfg->operator_note,
        static_cast<double>(cfg->launch_site_altitude_m),
        static_cast<double>(cfg->sea_level_pressure_hpa),
        static_cast<double>(cfg->liftoff_accel_threshold_mps2),
        static_cast<unsigned long>(cfg->liftoff_timeout_ms),
        static_cast<unsigned long>(cfg->apogee_lockout_ms),
        static_cast<unsigned long>(cfg->drogue_apogee_timeout_ms),
        static_cast<double>(cfg->main_altitude_threshold_m),
        static_cast<double>(cfg->touchdown_velocity_threshold_mps),
        static_cast<unsigned long>(cfg->launch_to_ballistic_threshold_ms),
        static_cast<unsigned long>(cfg->launch_to_apogee_threshold_ms),
        static_cast<double>(cfg->touchdown_altitude_threshold_m),
        cfg->telemetry_enabled ? "true" : "false",
        static_cast<unsigned long>(cfg->telemetry_period_ms),
        cfg->logging_enabled ? "true" : "false",
        MS56_I2C_ADDR_1, MS56_I2C_ADDR_2, bno055_i2c_addr,
        ESPNOW_CHANNEL,
        RECEIVER_MAC_ADDRESS[0], RECEIVER_MAC_ADDRESS[1], RECEIVER_MAC_ADDRESS[2],
        RECEIVER_MAC_ADDRESS[3], RECEIVER_MAC_ADDRESS[4], RECEIVER_MAC_ADDRESS[5],
        LORA_CHANNEL, LORA_ADDH, LORA_ADDL, LORA_RECEIVER_ADDH, LORA_RECEIVER_ADDL,
        recovery_mode,
        ALTITUDE_FILTER_WINDOW, APOGEE_DETECTION_WINDOW_SIZE,
        NUM_CALIBRATION_SAMPLES, static_cast<double>(STD_THRESHOLD),
        static_cast<unsigned long>(GPS_FIX_TIMEOUT_MS), GPS_MIN_FIX,
        IMU_MINIMUM_CALIBRATION, static_cast<double>(H_BIAS_PRESSURE_SENSOR), static_cast<double>(GPS_BIAS),
        BATCH_SIZE, MAX_LOG_ENTRIES,
        board_name,
        i2c_sda_pin, i2c_scl_pin,
        spi_mosi_pin, spi_miso_pin, spi_clk_pin,
        flash_cs_pin, barometer_cs_pin, barometer2_cs_pin,
        lora_aux_pin, lora_tx_pin, lora_rx_pin, lora_m0_pin, lora_m1_pin,
        main_actuator_pin, drogue_actuator_pin
    );
    return (n > 0 && static_cast<size_t>(n) < out_size) ? ESP_OK : ESP_ERR_NO_MEM;
}

/**
 * @brief Emit metadata used by Ground Services to render and validate fields.
 *
 * The schema combines editable RuntimeConfig fields, read-only compile-time
 * values from config.h, and read-only board hardware values supplied through
 * IBoardHardware. Only editable RuntimeConfig fields are accepted by
 * runtime_config_update_from_json() and persisted back to NVS.
 */
esp_err_t runtime_config_schema_json(char *out, size_t out_size)
{
    if (out == nullptr) return ESP_ERR_INVALID_ARG;
    int n = snprintf(out, out_size,
        "{\"ok\":true,\"fields\":["
        "{\"key\":\"rocket_name\",\"label\":\"Rocket name\",\"group\":\"Identity\",\"editable\":true,\"locked_after_ready\":true,\"description\":\"Mission display name.\"},"
        "{\"key\":\"launch_site\",\"label\":\"Launch site\",\"group\":\"Identity\",\"editable\":true,\"locked_after_ready\":true,\"description\":\"Human-readable launch location.\"},"
        "{\"key\":\"operator_note\",\"label\":\"Operator note\",\"group\":\"Identity\",\"editable\":true,\"locked_after_ready\":true,\"description\":\"Short mission note stored with the configuration.\"},"
        "{\"key\":\"launch_site_altitude_m\",\"label\":\"Launch site altitude\",\"group\":\"Environment\",\"unit\":\"m ASL\",\"editable\":true,\"locked_after_ready\":true,\"min\":0,\"max\":6000,\"description\":\"Launch site elevation above sea level.\"},"
        "{\"key\":\"sea_level_pressure_hpa\",\"label\":\"Sea-level pressure\",\"group\":\"Environment\",\"unit\":\"hPa\",\"editable\":true,\"locked_after_ready\":true,\"min\":850,\"max\":1100,\"description\":\"QNH reference for altitude calculations.\"},"
        "{\"key\":\"liftoff_accel_threshold_mps2\",\"label\":\"Liftoff acceleration threshold\",\"group\":\"Mission Flight\",\"unit\":\"m/s^2\",\"editable\":true,\"locked_after_ready\":true,\"min\":10,\"max\":150,\"description\":\"Acceleration magnitude required before liftoff timing starts.\"},"
        "{\"key\":\"liftoff_timeout_ms\",\"label\":\"Liftoff debounce\",\"group\":\"Mission Flight\",\"unit\":\"ms\",\"editable\":true,\"locked_after_ready\":true,\"min\":20,\"max\":10000,\"description\":\"Time the liftoff acceleration threshold must remain exceeded.\"},"
        "{\"key\":\"apogee_lockout_ms\",\"label\":\"Apogee lockout\",\"group\":\"Mission Flight\",\"unit\":\"ms\",\"editable\":true,\"locked_after_ready\":true,\"min\":0,\"max\":120000,\"description\":\"Time after liftoff during which sensor apogee detection is ignored.\"},"
        "{\"key\":\"drogue_apogee_timeout_ms\",\"label\":\"Drogue apogee delay\",\"group\":\"Mission Flight\",\"unit\":\"ms\",\"editable\":true,\"locked_after_ready\":true,\"min\":0,\"max\":120000,\"description\":\"Delay before drogue-ready transition after apogee.\"},"
        "{\"key\":\"launch_to_ballistic_threshold_ms\",\"label\":\"Launch-to-ballistic timeout\",\"group\":\"Mission Flight\",\"unit\":\"ms\",\"editable\":true,\"locked_after_ready\":true,\"min\":1,\"max\":300000,\"description\":\"Timeout for accelerated-flight exit.\"},"
        "{\"key\":\"launch_to_apogee_threshold_ms\",\"label\":\"Launch-to-apogee timeout\",\"group\":\"Mission Flight\",\"unit\":\"ms\",\"editable\":true,\"locked_after_ready\":true,\"min\":1,\"max\":300000,\"description\":\"Maximum time to apogee detection.\"},"
        "{\"key\":\"main_altitude_threshold_m\",\"label\":\"Main deployment altitude\",\"group\":\"Mission Flight\",\"unit\":\"m AGL\",\"editable\":true,\"locked_after_ready\":true,\"min\":50,\"max\":1000,\"description\":\"Altitude below which the main parachute may deploy.\"},"
        "{\"key\":\"touchdown_velocity_threshold_mps\",\"label\":\"Touchdown velocity threshold\",\"group\":\"Mission Flight\",\"unit\":\"m/s\",\"editable\":true,\"locked_after_ready\":true,\"min\":0.1,\"max\":20,\"description\":\"Vertical speed threshold used by touchdown logic.\"},"
        "{\"key\":\"touchdown_altitude_threshold_m\",\"label\":\"Touchdown altitude threshold\",\"group\":\"Mission Flight\",\"unit\":\"m AGL\",\"editable\":true,\"locked_after_ready\":true,\"min\":0,\"max\":200,\"description\":\"Altitude threshold used by touchdown logic.\"},"
        "{\"key\":\"telemetry_enabled\",\"label\":\"Telemetry enabled\",\"group\":\"Telemetry\",\"editable\":false,\"description\":\"Always enabled during flight.\"},"
        "{\"key\":\"telemetry_period_ms\",\"label\":\"Telemetry period\",\"group\":\"Telemetry\",\"unit\":\"ms\",\"editable\":true,\"locked_after_ready\":true,\"min\":100,\"max\":10000,\"description\":\"Telemetry transmit interval.\"},"
        "{\"key\":\"logging_enabled\",\"label\":\"Logging enabled\",\"group\":\"Logging\",\"editable\":false,\"description\":\"Always enabled during flight.\"},"
        "{\"key\":\"ms56_i2c_addr_1\",\"label\":\"MS56 address 1\",\"group\":\"Device Addresses\",\"editable\":false,\"description\":\"Primary MS56 I2C address.\"},"
        "{\"key\":\"ms56_i2c_addr_2\",\"label\":\"MS56 address 2\",\"group\":\"Device Addresses\",\"editable\":false,\"description\":\"Secondary MS56 I2C address.\"},"
        "{\"key\":\"bno055_i2c_addr\",\"label\":\"BNO055 address\",\"group\":\"Device Addresses\",\"editable\":false,\"description\":\"BNO055 I2C address from board hardware.\"},"
        "{\"key\":\"espnow_channel\",\"label\":\"ESP-NOW channel\",\"group\":\"Telemetry\",\"editable\":false,\"description\":\"Compiled ESP-NOW channel.\"},"
        "{\"key\":\"espnow_peer_mac\",\"label\":\"ESP-NOW peer MAC\",\"group\":\"Telemetry\",\"editable\":false,\"description\":\"Compiled ESP-NOW receiver MAC.\"},"
        "{\"key\":\"lora_channel\",\"label\":\"LoRa channel\",\"group\":\"Telemetry\",\"editable\":false,\"description\":\"Compiled LoRa channel.\"},"
        "{\"key\":\"lora_addh\",\"label\":\"LoRa ADDH\",\"group\":\"Telemetry\",\"editable\":false,\"description\":\"Compiled local LoRa high address.\"},"
        "{\"key\":\"lora_addl\",\"label\":\"LoRa ADDL\",\"group\":\"Telemetry\",\"editable\":false,\"description\":\"Compiled local LoRa low address.\"},"
        "{\"key\":\"lora_receiver_addh\",\"label\":\"LoRa receiver ADDH\",\"group\":\"Telemetry\",\"editable\":false,\"description\":\"Compiled receiver high address.\"},"
        "{\"key\":\"lora_receiver_addl\",\"label\":\"LoRa receiver ADDL\",\"group\":\"Telemetry\",\"editable\":false,\"description\":\"Compiled receiver low address.\"},"
        "{\"key\":\"recovery_mode\",\"label\":\"Recovery mode\",\"group\":\"Mission Flight\",\"editable\":false,\"description\":\"Compiled parachute recovery mode.\"},"
        "{\"key\":\"altitude_filter_window\",\"label\":\"Altitude filter window\",\"group\":\"Algorithms\",\"editable\":false,\"description\":\"Median filter window for altitude smoothing.\"},"
        "{\"key\":\"apogee_detection_window_size\",\"label\":\"Apogee detection window\",\"group\":\"Algorithms\",\"editable\":false,\"description\":\"Apogee detector window size.\"},"
        "{\"key\":\"num_calibration_samples\",\"label\":\"Calibration samples\",\"group\":\"Algorithms\",\"editable\":false,\"description\":\"Calibration sample count.\"},"
        "{\"key\":\"std_threshold\",\"label\":\"Calibration std threshold\",\"group\":\"Algorithms\",\"editable\":false,\"description\":\"Calibration stability threshold.\"},"
        "{\"key\":\"gps_fix_timeout_ms\",\"label\":\"GPS fix timeout\",\"group\":\"Algorithms\",\"unit\":\"ms\",\"editable\":false,\"description\":\"GPS fix lookup timeout.\"},"
        "{\"key\":\"gps_min_fix\",\"label\":\"GPS minimum fix\",\"group\":\"Algorithms\",\"editable\":false,\"description\":\"Minimum GPS fix type.\"},"
        "{\"key\":\"imu_minimum_calibration\",\"label\":\"IMU minimum calibration\",\"group\":\"Algorithms\",\"editable\":false,\"description\":\"Minimum IMU calibration level.\"},"
        "{\"key\":\"h_bias_pressure_sensor\",\"label\":\"Pressure sensor bias\",\"group\":\"Algorithms\",\"editable\":false,\"description\":\"Compiled pressure sensor bias.\"},"
        "{\"key\":\"gps_bias\",\"label\":\"GPS bias\",\"group\":\"Algorithms\",\"editable\":false,\"description\":\"Compiled GPS bias.\"},"
        "{\"key\":\"batch_size\",\"label\":\"Log batch size\",\"group\":\"Logging\",\"editable\":false,\"description\":\"Log entries batched before storage write.\"},"
        "{\"key\":\"max_log_entries\",\"label\":\"Max log entries\",\"group\":\"Logging\",\"editable\":false,\"description\":\"Maximum queued log entries before protection.\"},"
        "{\"key\":\"board_name\",\"label\":\"Board\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"Compiled board target.\"},"
        "{\"key\":\"i2c_sda_pin\",\"label\":\"I2C SDA pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"Primary I2C SDA GPIO.\"},"
        "{\"key\":\"i2c_scl_pin\",\"label\":\"I2C SCL pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"Primary I2C SCL GPIO.\"},"
        "{\"key\":\"spi_mosi_pin\",\"label\":\"SPI MOSI pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"SPI MOSI GPIO.\"},"
        "{\"key\":\"spi_miso_pin\",\"label\":\"SPI MISO pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"SPI MISO GPIO.\"},"
        "{\"key\":\"spi_clk_pin\",\"label\":\"SPI clock pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"SPI clock GPIO.\"},"
        "{\"key\":\"flash_cs_pin\",\"label\":\"Flash CS pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"External flash chip-select GPIO.\"},"
        "{\"key\":\"barometer_cs_pin\",\"label\":\"Barometer CS pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"Primary barometer chip-select GPIO.\"},"
        "{\"key\":\"barometer2_cs_pin\",\"label\":\"Barometer 2 CS pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"Secondary barometer chip-select GPIO.\"},"
        "{\"key\":\"lora_aux_pin\",\"label\":\"LoRa AUX pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"LoRa AUX GPIO.\"},"
        "{\"key\":\"lora_tx_pin\",\"label\":\"LoRa TX pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"LoRa UART TX GPIO.\"},"
        "{\"key\":\"lora_rx_pin\",\"label\":\"LoRa RX pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"LoRa UART RX GPIO.\"},"
        "{\"key\":\"lora_m0_pin\",\"label\":\"LoRa M0 pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"LoRa mode pin M0.\"},"
        "{\"key\":\"lora_m1_pin\",\"label\":\"LoRa M1 pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"LoRa mode pin M1.\"},"
        "{\"key\":\"main_actuator_pin\",\"label\":\"Main actuator pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"Main parachute actuator GPIO.\"},"
        "{\"key\":\"drogue_actuator_pin\",\"label\":\"Drogue actuator pin\",\"group\":\"Board Hardware\",\"editable\":false,\"description\":\"Drogue actuator GPIO.\"}"
        "]}");
    return (n > 0 && static_cast<size_t>(n) < out_size) ? ESP_OK : ESP_ERR_NO_MEM;
}

struct ValidationJsonContext {
    char *out;
    size_t out_size;
    size_t offset;
    bool first;
    bool overflow;
};

/**
 * @brief Append one validation item to the dashboard JSON response.
 */
static void append_validation_json(const RuntimeConfigValidationItem &item, void *ctx)
{
    auto *json = static_cast<ValidationJsonContext *>(ctx);
    if (json == nullptr || json->overflow) return;
    int n = snprintf(json->out + json->offset,
                     json->out_size > json->offset ? json->out_size - json->offset : 0,
                     "%s{\"field\":\"%s\",\"severity\":\"%s\",\"message\":\"%s\"}",
                     json->first ? "" : ",",
                     item.field,
                     item.severity,
                     item.message);
    if (n < 0 || static_cast<size_t>(n) >= json->out_size - json->offset) {
        json->overflow = true;
        return;
    }
    json->offset += static_cast<size_t>(n);
    json->first = false;
}

esp_err_t runtime_config_validation_json(char *out, size_t out_size)
{
    if (out == nullptr || out_size == 0) return ESP_ERR_INVALID_ARG;
    RuntimeConfig cfg;
    esp_err_t err = runtime_config_get(&cfg);
    if (err != ESP_OK) return err;

    ValidationJsonContext ctx{out, out_size, 0, true, false};
    const esp_err_t validation = runtime_config_validate_each(&cfg, nullptr, nullptr);
    int n = snprintf(out, out_size, "{\"ok\":%s,\"items\":[", validation == ESP_OK ? "true" : "false");
    if (n < 0 || static_cast<size_t>(n) >= out_size) return ESP_ERR_NO_MEM;
    ctx.offset = static_cast<size_t>(n);
    (void)runtime_config_validate_each(&cfg, append_validation_json, &ctx);
    n = snprintf(out + ctx.offset, out_size - ctx.offset, "]}");
    if (ctx.overflow || n < 0 || static_cast<size_t>(n) >= out_size - ctx.offset) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

/**
 * @brief Freeze the current active config for flight task consumption.
 */
esp_err_t runtime_config_lock_for_flight(char *reason, size_t reason_size)
{
    if (!s_loaded) return validation_fail(reason, reason_size, "runtime config is not loaded");
    RuntimeConfig locked = s_active_config;

    esp_err_t err = runtime_config_validate(&locked, reason, reason_size);
    if (err != ESP_OK) return err;

    // Locking persists the active config and captures the immutable flight
    // snapshot. This is the handoff point from editable ground configuration to
    // task-owned flight behavior.
    locked.config_locked = true;
    locked.config_revision++;
    err = runtime_config_save(&locked);
    if (err != ESP_OK) {
        validation_fail(reason, reason_size, "failed to persist locked flight config");
        return err;
    }

    s_flight_snapshot = locked;
    s_flight_snapshot_valid = true;
    if (reason != nullptr && reason_size > 0) snprintf(reason, reason_size, "%s", "ok");
    return ESP_OK;
}
