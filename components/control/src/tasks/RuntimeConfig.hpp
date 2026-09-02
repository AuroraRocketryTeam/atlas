#pragma once

#include <cstddef>
#include <cstdint>
#include "esp_err.h"
#include "AirbrakesTask.hpp"
#include "AltitudeTask.hpp"
#include "TelemetryTask.hpp"
#include "FlightParametersConfig.hpp"

class IBoardHardware;

/**
 * @brief Maintainer guide for adding RuntimeConfig/dashboard parameters.
 *
 * RuntimeConfig intentionally separates editable mission parameters from
 * read-only firmware or hardware facts. Keep that distinction clear whenever
 * adding fields:
 *
 * Persisted flight-parameter workflow:
 * 1. Add the field to the owning area struct (for example AirbrakesConfig or
 *    AltitudeConfig), then compose that struct in RuntimeConfig. Prefer
 *    fixed-size primitive types or char arrays because the whole value is one
 *    NVS blob.
 * 2. Increment the schema_version assigned in make_default_config().
 * 3. Set a safe firmware default in make_default_config(), usually from
 *    config.h while legacy code is still sharing those constants.
 * 4. Add validation in runtime_config_validate_each(). Keep the validation
 *    limits aligned with the schema min/max values.
 * 5. For an operator-editable field in MissionConfig, FlightParametersConfig,
 *    AltitudeConfig, or RecoveryConfig, add parsing in
 *    runtime_config_update_from_json(). Calibration and telemetry remain
 *    inspection-only unless deliberately promoted later.
 * 6. Add serialization in runtime_config_to_json().
 * 7. Add field metadata in runtime_config_schema_json() with its owning source,
 *    useful unit/min/max values, and editable=true plus
 *    locked_after_ready=true only for launch-ramp adjustments.
 * 8. Update any task/class to read the value from
 *    runtime_config_get_flight_snapshot(), not directly from config.h.
 *
 * Read-only dashboard/checklist parameter workflow:
 * 1. Do not add the value to RuntimeConfig unless it must be persisted or edited.
 * 2. Add serialization in runtime_config_to_json() from the owning source:
 *    config.h for compiled constants, IBoardHardware for board facts, or a
 *    task-owned helper when the value belongs to one task.
 * 3. Add field metadata in runtime_config_schema_json() with editable=false
 *    and its owning source.
 *    This is the right path for values such as the selected barometer,
 *    compiled recovery mode, protocol addresses, or board pins.
 * 4. If operators must verify the value before launch, add a checklist item in
 *    GroundServicesTask::buildPrelaunchChecklistJson(). Use severity="info"
 *    for visibility-only items, "warning" for review items, and "error" only
 *    for blockers that should prevent normal ready-for-launch.
 *
 * Misuse-proofing rules:
 * - Flight-critical editable values must be consumed from the locked flight
 *   snapshot so later dashboard edits or recovery unlocks cannot affect an
 *   active flight.
 * - Read-only values may be visible in the dashboard without being editable;
 *   this is preferred for compiled constants like AIRBRAKES_OPEN_ALTITUDE_M
 *   until there is a clear need to tune them from NVS.
 * - Keep the dashboard essential. Expose values that support flight decisions,
 *   checklist review, debugging, or hardware verification. Avoid surfacing
 *   internal buffer sizes and protocol implementation details unless they are
 *   useful to an operator.
 * - Any field shown in the schema should also be present in
 *   runtime_config_to_json(); otherwise the web UI will render an empty value.
 * - Any editable field accepted from JSON must be validated before saving.
 */

/**
 * @brief Runtime mission configuration persisted in NVS.
 *
 * The struct is a small composition of area-owned values. Mission, flight,
 * altitude, and recovery values are editable before the flight lock and saved
 * as one NVS snapshot. Calibration and telemetry are inspection-only; board
 * facts and driver internals remain outside this type.
 */
struct MissionConfig {
    char rocket_name[32];
    char launch_site[32];
    char operator_note[96];
    float launch_site_altitude_m;
    float sea_level_pressure_hpa;
};

struct RuntimeConfig {
    uint32_t schema_version;
    uint32_t config_revision;
    MissionConfig mission;
    FlightParametersConfig flight;
    AirbrakesConfig airbrakes;
    AltitudeConfig altitude;
    CalibrationConfig calibration;
    RecoveryConfig recovery;
    TelemetryConfig telemetry;
    bool config_locked;
    uint32_t checksum;
};

struct RuntimeConfigValidationItem {
    /** JSON/schema field path, for example "flight.main_altitude_threshold_m". */
    const char *field;
    /** Validation severity consumed by the dashboard, currently "error" or "warning". */
    const char *severity;
    /** Human-readable validation result. */
    const char *message;
};

/**
 * @brief Callback invoked once for each validation item.
 *
 * The callback is used by both text validation and JSON validation output so
 * all validation rules stay in runtime_config_validate_each().
 */
using RuntimeConfigValidationCallback = void (*)(const RuntimeConfigValidationItem &item, void *ctx);

/**
 * @brief Load RuntimeConfig from NVS or create defaults.
 *
 * @param board Board hardware provider used for read-only board metadata in
 *              JSON responses. The pointer is not owned by RuntimeConfig and
 *              must outlive calls to runtime_config_to_json().
 * @return ESP_OK on load/create success, otherwise the NVS or validation error.
 */
esp_err_t runtime_config_init(IBoardHardware *board);

/**
 * @brief Copy the active in-memory RuntimeConfig.
 *
 * runtime_config_init() must succeed before this function is used.
 *
 * @param out Destination for the active config.
 * @return ESP_OK, ESP_ERR_INVALID_ARG for null output, or ESP_ERR_INVALID_STATE
 *         when the config has not been initialized.
 */
esp_err_t runtime_config_get(RuntimeConfig *out);

/**
 * @brief Return whether the active config has been locked for flight.
 *
 * A locked config rejects dashboard edits and represents the values that should
 * be used once the vehicle transitions out of Ground Services.
 */
bool runtime_config_is_locked();

/**
 * @brief Return a synchronized copy of the immutable flight snapshot when available.
 *
 * The nominal workflow is: edit config in Ground Services, validate checklist,
 * lock for flight, then tasks read this snapshot so later unlock/reset actions
 * cannot mutate parameters used by an active flight. Returning by value avoids
 * retaining a reference that changes meaning when the flight lock is created.
 */
RuntimeConfig runtime_config_get_flight_snapshot();

/**
 * @brief Validate, checksum, persist, and activate a RuntimeConfig.
 *
 * The config is stored as a single NVS blob in the RuntimeConfig namespace.
 * Callers should prefer update/reset helpers unless they already have a fully
 * validated RuntimeConfig instance.
 */
esp_err_t runtime_config_save(const RuntimeConfig *cfg);

/**
 * @brief Replace the active config with firmware defaults.
 *
 * This is rejected while the config is locked for flight.
 */
esp_err_t runtime_config_reset_defaults();

/**
 * @brief Clear only the flight lock after recovery or inspection.
 *
 * The saved mission values remain in NVS; only config_locked and the in-memory
 * flight snapshot state are changed.
 */
esp_err_t runtime_config_unlock_after_recovery();

/**
 * @brief Apply editable fields from dashboard JSON and persist the result.
 *
 * Read-only firmware, board, and schema values in the incoming JSON are ignored.
 * This intentionally accepts only the editable RuntimeConfig fields.
 */
esp_err_t runtime_config_update_from_json(const char *json, RuntimeConfig *updated_out);

/**
 * @brief Serialize RuntimeConfig plus read-only firmware and board metadata.
 *
 * Editable fields come from the provided RuntimeConfig. Compiled constants come
 * from config.h, and board details come from the IBoardHardware pointer passed
 * to runtime_config_init().
 */
esp_err_t runtime_config_to_json(const RuntimeConfig *cfg, char *out, size_t out_size);

/**
 * @brief Serialize the dashboard schema for editable and read-only fields.
 *
 * The web UI uses this schema to group fields, choose controls, show ranges,
 * and mark values as read-only or locked after ready-for-launch.
 */
esp_err_t runtime_config_schema_json(char *out, size_t out_size);

/**
 * @brief Validate a RuntimeConfig and return the first error as text.
 */
esp_err_t runtime_config_validate(const RuntimeConfig *cfg, char *reason, size_t reason_size);

/**
 * @brief Validate a RuntimeConfig and emit every validation item.
 *
 * This is the source of truth for validation rules. Passing a null callback is
 * allowed when the caller only needs the aggregate ESP_OK/error result.
 */
esp_err_t runtime_config_validate_each(const RuntimeConfig *cfg, RuntimeConfigValidationCallback cb, void *ctx);

/**
 * @brief Serialize validation results for the active RuntimeConfig.
 */
esp_err_t runtime_config_validation_json(char *out, size_t out_size);

/**
 * @brief Validate, persist, and snapshot the active config for flight.
 *
 * On success, config_locked is set, the revision is incremented, and the flight
 * snapshot becomes the authoritative task configuration.
 */
esp_err_t runtime_config_lock_for_flight(char *reason, size_t reason_size);

/**
 * @brief Return the firmware default RuntimeConfig.
 */
const RuntimeConfig &runtime_config_defaults();
