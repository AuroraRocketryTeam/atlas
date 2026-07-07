#include "GroundServicesTask.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <climits>

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "mbedtls/md.h"
#include "mbedtls/sha256.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "config.h"
#include "IGroundTestRunner.hpp"
#include "SerialLogger.hpp"
#include "utils.h"

static const char *TAG = "GroundServices";

#ifndef CONFIG_GROUND_SERVICES_AUTH_TOKEN
#error "CONFIG_GROUND_SERVICES_AUTH_TOKEN must be configured. It is used as the HMAC shared secret for Ground Services."
#endif

static constexpr size_t OTA_UPLOAD_BUFFER_SIZE = 1024;
static constexpr size_t AUTH_HEADER_MAX_LEN = 96;
static constexpr size_t AUTH_SHA256_HEX_LEN = 64;
static constexpr size_t AUTH_SIGNATURE_HEX_LEN = 64;
static constexpr size_t BODY_MAX_LEN = 4096;
static constexpr size_t RESPONSE_BUFFER_SIZE = 12288;
static constexpr size_t LOG_RESPONSE_BUFFER_SIZE = 4096;
static constexpr uint32_t AUTH_NONCE_TTL_MS = 30000;

static char s_otaUploadBuffer[OTA_UPLOAD_BUFFER_SIZE];
static char s_bodyBuffer[BODY_MAX_LEN + 1];
static char s_responseBuffer[RESPONSE_BUFFER_SIZE];
static char s_logResponseBuffer[LOG_RESPONSE_BUFFER_SIZE];
extern const unsigned char ground_index_html_start[]     asm("_binary_index_html_start");
extern const unsigned char ground_index_html_end[]       asm("_binary_index_html_end");
extern const unsigned char ground_style_css_start[]      asm("_binary_style_css_start");
extern const unsigned char ground_style_css_end[]        asm("_binary_style_css_end");
extern const unsigned char ground_app_js_start[]         asm("_binary_app_js_start");
extern const unsigned char ground_app_js_end[]           asm("_binary_app_js_end");
extern const unsigned char ground_sdkconfig_start[]      asm("_binary_embedded_sdkconfig_txt_start");
extern const unsigned char ground_sdkconfig_end[]        asm("_binary_embedded_sdkconfig_txt_end");

GroundServicesTask::GroundServicesTask(std::shared_ptr<RocketModel> rocketModel,
                                       std::shared_ptr<RocketLogger> logger,
                                       IBoardHardware* board,
                                       IStateMachine* fsm,
                                       std::shared_ptr<IGroundTestRunner> testRunner)
    : BaseTask("GroundServicesTask"),
      _rocketModel(rocketModel),
      _logger(logger),
      _testRunner(testRunner),
      _board(board),
      _fsm(fsm),
      _server(nullptr),
      _softApAcquired(false),
      _otaMutex(nullptr),
      _authMutex(nullptr)
{
}

GroundServicesTask::~GroundServicesTask()
{
    stopServer();
}

static const char *otaStateToString(GroundServicesTask::OtaState state)
{
    switch (state) {
        case GroundServicesTask::OtaState::IDLE: return "idle";
        case GroundServicesTask::OtaState::WRITING: return "writing";
        case GroundServicesTask::OtaState::READY_TO_REBOOT: return "ready_to_reboot";
        case GroundServicesTask::OtaState::FAILED: return "failed";
        default: return "unknown";
    }
}

static const char *resetReasonToString(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON: return "power_on";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt_watchdog";
        case ESP_RST_TASK_WDT: return "task_watchdog";
        case ESP_RST_WDT: return "watchdog";
        case ESP_RST_DEEPSLEEP: return "deep_sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        default: return "unknown";
    }
}

static const char *chipModelToString(esp_chip_model_t model)
{
    switch (model) {
        case CHIP_ESP32: return "ESP32";
        case CHIP_ESP32S2: return "ESP32-S2";
        case CHIP_ESP32S3: return "ESP32-S3";
        case CHIP_ESP32C3: return "ESP32-C3";
        case CHIP_ESP32C2: return "ESP32-C2";
        case CHIP_ESP32C6: return "ESP32-C6";
        case CHIP_ESP32H2: return "ESP32-H2";
        case CHIP_ESP32P4: return "ESP32-P4";
        default: return "unknown";
    }
}

static void chipFeaturesToJson(uint32_t features, char *out, size_t out_size)
{
    if (out == nullptr || out_size == 0) return;
    size_t offset = 0;
    out[offset++] = '[';
    out[offset] = '\0';

    auto add = [&](const char *name) {
        if (offset + strlen(name) + 4 >= out_size) return;
        offset += snprintf(out + offset, out_size - offset, "%s\"%s\"", offset > 1 ? "," : "", name);
    };

    if (features & CHIP_FEATURE_WIFI_BGN) add("Wi-Fi");
    if (features & CHIP_FEATURE_BLE) add("BLE");
    if (features & CHIP_FEATURE_BT) add("BT");
    if (features & CHIP_FEATURE_EMB_FLASH) add("embedded_flash");
    if (features & CHIP_FEATURE_EMB_PSRAM) add("embedded_psram");
    if (features & CHIP_FEATURE_IEEE802154) add("802.15.4");
    if (offset + 2 <= out_size) snprintf(out + offset, out_size - offset, "]");
}

static bool secureStringEqual(const char *a, const char *b)
{
    if (a == nullptr || b == nullptr) return false;
    const size_t len_a = strlen(a);
    const size_t len_b = strlen(b);
    if (len_a != len_b) return false;
    unsigned char diff = 0;
    for (size_t i = 0; i < len_a; ++i) diff |= static_cast<unsigned char>(a[i] ^ b[i]);
    return diff == 0;
}

static void bytesToHex(const uint8_t *bytes, size_t len, char *out, size_t out_size)
{
    static constexpr char HEX_ALPHABET[] = "0123456789abcdef";
    if (out == nullptr || out_size < (len * 2U + 1U)) return;
    for (size_t i = 0; i < len; ++i) {
        out[i * 2U] = HEX_ALPHABET[(bytes[i] >> 4U) & 0x0F];
        out[i * 2U + 1U] = HEX_ALPHABET[bytes[i] & 0x0F];
    }
    out[len * 2U] = '\0';
}

static bool isLowerHex(const char *text, size_t expected_len)
{
    if (text == nullptr || strlen(text) != expected_len) return false;
    for (size_t i = 0; i < expected_len; ++i) {
        const char c = text[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
}

static bool hmacSha256Hex(const char *secret, const char *message, char out[65])
{
    if (secret == nullptr || message == nullptr || out == nullptr) return false;
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (info == nullptr) return false;
    uint8_t digest[32];
    const int rc = mbedtls_md_hmac(info,
                                   reinterpret_cast<const unsigned char *>(secret), strlen(secret),
                                   reinterpret_cast<const unsigned char *>(message), strlen(message),
                                   digest);
    if (rc != 0) return false;
    bytesToHex(digest, sizeof(digest), out, 65);
    return true;
}

static const char *methodToString(httpd_method_t method)
{
    switch (method) {
        case HTTP_GET: return "GET";
        case HTTP_POST: return "POST";
        case HTTP_PUT: return "PUT";
        case HTTP_DELETE: return "DELETE";
        default: return "UNKNOWN";
    }
}

static esp_err_t sendJson(httpd_req_t *req, const char *status, const char *json)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    (void)httpd_resp_sendstr(req, json);
    return ESP_OK;
}

static esp_err_t sendErrorJson(httpd_req_t *req, const char *status, const char *error, esp_err_t err = ESP_OK)
{
    char body[256];
    snprintf(body, sizeof(body),
             "{\"ok\":false,\"error\":\"%s\",\"esp_err\":\"%s\"}",
             error, err == ESP_OK ? "ESP_OK" : esp_err_to_name(err));
    return sendJson(req, status, body);
}

static esp_err_t discardBody(httpd_req_t *req)
{
    char tmp[128];
    size_t remaining = req->content_len;
    while (remaining > 0) {
        const size_t to_read = std::min(remaining, sizeof(tmp));
        int received = httpd_req_recv(req, tmp, static_cast<int>(to_read));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
            return ESP_FAIL;
        }
        remaining -= static_cast<size_t>(received);
    }
    return ESP_OK;
}

static bool getHeader(httpd_req_t *req, const char *name, char *out, size_t out_size)
{
    if (out == nullptr || out_size == 0) return false;
    out[0] = '\0';
    return httpd_req_get_hdr_value_str(req, name, out, out_size) == ESP_OK;
}

static bool getSignedBodyHash(httpd_req_t *req, char out[65])
{
    char value[AUTH_HEADER_MAX_LEN] = {};
    if (!getHeader(req, "X-Auth-Body-SHA256", value, sizeof(value))) return false;
    if (!isLowerHex(value, AUTH_SHA256_HEX_LEN)) return false;
    memcpy(out, value, 65);
    return true;
}

static esp_err_t readBody(httpd_req_t *req, char *out, size_t out_size)
{
    if (out == nullptr || out_size == 0) {
        discardBody(req);
        return ESP_ERR_INVALID_ARG;
    }
    out[0] = '\0';
    if (req->content_len >= out_size || req->content_len > BODY_MAX_LEN) {
        discardBody(req);
        return ESP_ERR_INVALID_SIZE;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);

    size_t remaining = req->content_len;
    size_t offset = 0;
    while (remaining > 0) {
        int received = httpd_req_recv(req, out + offset, static_cast<int>(remaining));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
            mbedtls_sha256_free(&sha);
            return ESP_FAIL;
        }
        mbedtls_sha256_update(&sha, reinterpret_cast<const uint8_t *>(out + offset), static_cast<size_t>(received));
        offset += static_cast<size_t>(received);
        remaining -= static_cast<size_t>(received);
    }
    out[offset] = '\0';

    uint8_t digest[32];
    char actual_hash[65];
    char signed_hash[65];
    mbedtls_sha256_finish(&sha, digest);
    mbedtls_sha256_free(&sha);
    bytesToHex(digest, sizeof(digest), actual_hash, sizeof(actual_hash));

    if (!getSignedBodyHash(req, signed_hash)) return ESP_ERR_INVALID_ARG;
    if (!secureStringEqual(actual_hash, signed_hash)) return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}

static bool authOk(httpd_req_t *req)
{
    auto *self = static_cast<GroundServicesTask *>(req ? req->user_ctx : nullptr);
    return self != nullptr && self->authenticateRequest(req);
}

static bool authOrSend(httpd_req_t *req)
{
    if (!authOk(req)) {
        sendErrorJson(req, "401 Unauthorized", "unauthorized");
        return false;
    }
    return true;
}

static bool headerEquals(httpd_req_t *req, const char *name, const char *expected)
{
    char value[128] = {};
    return httpd_req_get_hdr_value_str(req, name, value, sizeof(value)) == ESP_OK && strcmp(value, expected) == 0;
}

static bool bodyGetInt(const char *body, const char *key, int *out)
{
    if (body == nullptr || key == nullptr || out == nullptr) return false;
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(body, pattern);
    if (p == nullptr) return false;
    p = strchr(p, ':');
    if (p == nullptr) return false;
    ++p;
    char *end = nullptr;
    long value = strtol(p, &end, 10);
    if (end == p) return false;
    *out = static_cast<int>(value);
    return true;
}

static bool bodyGetString(const char *body, const char *key, char *out, size_t out_size)
{
    if (body == nullptr || key == nullptr || out == nullptr || out_size == 0) return false;
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(body, pattern);
    if (p == nullptr) return false;
    p = strchr(p, ':');
    if (p == nullptr) return false;
    ++p;
    while (*p != '\0' && isspace(static_cast<unsigned char>(*p))) ++p;
    if (*p != '"') return false;
    ++p;
    size_t i = 0;
    while (*p != '\0' && *p != '"' && i + 1 < out_size) out[i++] = *p++;
    out[i] = '\0';
    return *p == '"';
}

static uint32_t queryGetUint32(httpd_req_t *req, const char *key, uint32_t fallback)
{
    if (req == nullptr || key == nullptr) return fallback;

    const char *query = strchr(req->uri, '?');
    if (query == nullptr) return fallback;
    query++;

    const size_t key_len = strlen(key);
    while (*query != '\0') {
        if (strncmp(query, key, key_len) == 0 && query[key_len] == '=') {
            char *end = nullptr;
            unsigned long value = strtoul(query + key_len + 1, &end, 10);
            if (end != query + key_len + 1) {
                return static_cast<uint32_t>(value);
            }
        }

        query = strchr(query, '&');
        if (query == nullptr) break;
        query++;
    }

    return fallback;
}

static const char *boolJson(bool value)
{
    return value ? "true" : "false";
}

static const char *sensorStatusToString(SensorReadStatus status);

static esp_err_t sendAsset(httpd_req_t *req, const unsigned char *start, const unsigned char *end, const char *content_type)
{
    size_t len = static_cast<size_t>(end - start);
    if (len > 0 && start[len - 1] == '\0') len--;
    httpd_resp_set_type(req, content_type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    (void)httpd_resp_send(req, reinterpret_cast<const char *>(start), len);
    return ESP_OK;
}

void GroundServicesTask::onTaskStart()
{
    _otaMutex = xSemaphoreCreateMutex();
    _authMutex = xSemaphoreCreateMutex();
    if (_otaMutex == nullptr || _authMutex == nullptr) {
        LOG_ERROR(TAG, "Failed to create Ground Services mutexes");
        return;
    }

    if (_board != nullptr) {
        if (!_board->initNvs()) {
            LOG_ERROR(TAG, "Failed to initialize board NVS");
            return;
        }
    }

    ESP_ERROR_CHECK(runtime_config_init(_board));

    if (_board != nullptr) {
        if (!_board->startWifiSoftAp()) {
            LOG_ERROR(TAG, "Failed to start Ground Services SoftAP");
            return;
        }
        _softApAcquired = true;
    }

    esp_err_t err = startServer();
    if (err != ESP_OK) {
        LOG_ERROR(TAG, "Failed to start server: %s", esp_err_to_name(err));
    }
}

void GroundServicesTask::onTaskStop()
{
    stopServer();
    if (_softApAcquired) {
        if (_board != nullptr && !_board->stopWifi()) {
            LOG_ERROR(TAG, "Failed to release Ground Services SoftAP");
        }
        _softApAcquired = false;
    }
    if (_otaMutex != nullptr) {
        vSemaphoreDelete(_otaMutex);
        _otaMutex = nullptr;
    }
    if (_authMutex != nullptr) {
        vSemaphoreDelete(_authMutex);
        _authMutex = nullptr;
    }
}

void GroundServicesTask::taskFunction()
{
    while (running) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t GroundServicesTask::startServer()
{
    if (_server != nullptr) return ESP_OK;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 30;
    config.stack_size = 12288;
    config.max_open_sockets = 7;
    config.backlog_conn = 4;
    config.recv_wait_timeout = 20;
    config.send_wait_timeout = 20;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t err = httpd_start(&_server, &config);
    if (err != ESP_OK) return err;
    registerHandlers();
    LOG_INFO(TAG, "HTTP dashboard started at http://%s/", _board ? _board->getWifiIpAddress() : "");
    return ESP_OK;
}

void GroundServicesTask::stopServer()
{
    if (_server != nullptr) {
        httpd_stop(_server);
        _server = nullptr;
    }
}

GroundServicesTask* GroundServicesTask::fromReq(httpd_req_t *req)
{
    return static_cast<GroundServicesTask*>(req->user_ctx);
}

void GroundServicesTask::registerHandlers()
{
    auto add = [this](const char *uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *)) {
        httpd_uri_t h = {};
        h.uri = uri;
        h.method = method;
        h.handler = handler;
        h.user_ctx = this;
        ESP_ERROR_CHECK(httpd_register_uri_handler(_server, &h));
    };

    add("/", HTTP_GET, rootGetHandler);
    add("/style.css", HTTP_GET, styleGetHandler);
    add("/app.js", HTTP_GET, appJsGetHandler);
    add("/favicon.ico", HTTP_GET, faviconGetHandler);
    add("/api/auth/nonce", HTTP_GET, authNonceGetHandler);
    add("/api/status", HTTP_GET, statusGetHandler);
    add("/api/health", HTTP_GET, healthGetHandler);
    add("/api/live-data", HTTP_GET, liveDataGetHandler);
    add("/api/logs", HTTP_GET, logsGetHandler);
    add("/api/config/runtime", HTTP_GET, runtimeConfigGetHandler);
    add("/api/config/runtime", HTTP_PUT, runtimeConfigPutHandler);
    add("/api/config/schema", HTTP_GET, runtimeConfigSchemaGetHandler);
    add("/api/config/validation", HTTP_GET, runtimeConfigValidationGetHandler);
    add("/api/config/reset-defaults", HTTP_POST, runtimeConfigResetHandler);
    add("/api/config/unlock-after-recovery", HTTP_POST, runtimeConfigUnlockHandler);
    add("/api/config/sdkconfig", HTTP_GET, sdkconfigGetHandler);
    add("/api/prelaunch/checklist", HTTP_GET, prelaunchChecklistGetHandler);
    // add("/api/fsm/start-calibration", HTTP_POST, startCalibrationPostHandler);
    add("/api/fsm/ready-for-launch", HTTP_POST, readyForLaunchPostHandler);
    add("/api/tests", HTTP_GET, testsListGetHandler);
    add("/api/tests/status", HTTP_GET, testsStatusGetHandler);
    add("/api/tests/start", HTTP_POST, testsStartPostHandler);
    add("/api/tests/verdict", HTTP_POST, testsVerdictPostHandler);
    add("/api/ota/status", HTTP_GET, otaStatusGetHandler);
    add("/api/ota/upload", HTTP_POST, otaUploadPostHandler);
    add("/api/ota/reboot", HTTP_POST, otaRebootPostHandler);
    add("/*", HTTP_GET, redirectToRootHandler);
}

void GroundServicesTask::setOtaStatus(OtaState state, size_t written, size_t total, esp_err_t err, const char *message, const char *sha256)
{
    if (_otaMutex != nullptr) xSemaphoreTake(_otaMutex, portMAX_DELAY);
    _otaStatus.state = state;
    _otaStatus.bytes_written = written;
    _otaStatus.total_size = total;
    _otaStatus.last_error = err;
    snprintf(_otaStatus.message, sizeof(_otaStatus.message), "%s", message ? message : "");
    if (sha256 != nullptr) {
        snprintf(_otaStatus.sha256, sizeof(_otaStatus.sha256), "%s", sha256);
    } else if (state == OtaState::IDLE || state == OtaState::WRITING || state == OtaState::FAILED) {
        _otaStatus.sha256[0] = '\0';
    }
    if (_otaMutex != nullptr) xSemaphoreGive(_otaMutex);
}

GroundServicesTask::OtaStatus GroundServicesTask::getOtaStatus() const
{
    OtaStatus copy;
    if (_otaMutex != nullptr) xSemaphoreTake(_otaMutex, portMAX_DELAY);
    copy = _otaStatus;
    if (_otaMutex != nullptr) xSemaphoreGive(_otaMutex);
    return copy;
}


bool GroundServicesTask::issueAuthNonce(char out[33], uint32_t ttl_ms)
{
    if (out == nullptr || _authMutex == nullptr) return false;

    uint8_t random_bytes[16];
    for (size_t i = 0; i < sizeof(random_bytes); i += 4) {
        const uint32_t r = esp_random();
        memcpy(random_bytes + i, &r, std::min(sizeof(r), sizeof(random_bytes) - i));
    }
    bytesToHex(random_bytes, sizeof(random_bytes), out, 33);

    const int64_t now_ms = esp_timer_get_time() / 1000;
    const int64_t expires = now_ms + ttl_ms;

    xSemaphoreTake(_authMutex, portMAX_DELAY);
    size_t slot = 0;
    int64_t oldest = INT64_MAX;
    for (size_t i = 0; i < AUTH_NONCE_COUNT; ++i) {
        if (_authNonces[i].used || _authNonces[i].expires_at_ms <= now_ms) {
            slot = i;
            oldest = INT64_MIN;
            break;
        }
        if (_authNonces[i].expires_at_ms < oldest) {
            oldest = _authNonces[i].expires_at_ms;
            slot = i;
        }
    }

    snprintf(_authNonces[slot].value, sizeof(_authNonces[slot].value), "%s", out);
    _authNonces[slot].expires_at_ms = expires;
    _authNonces[slot].used = false;
    xSemaphoreGive(_authMutex);
    return true;
}

bool GroundServicesTask::consumeAuthNonce(const char *nonce)
{
    if (nonce == nullptr || _authMutex == nullptr) return false;
    const int64_t now_ms = esp_timer_get_time() / 1000;
    bool ok = false;

    xSemaphoreTake(_authMutex, portMAX_DELAY);
    for (size_t i = 0; i < AUTH_NONCE_COUNT; ++i) {
        if (!_authNonces[i].used && _authNonces[i].expires_at_ms > now_ms && secureStringEqual(_authNonces[i].value, nonce)) {
            _authNonces[i].used = true;
            ok = true;
            break;
        }
    }
    xSemaphoreGive(_authMutex);
    return ok;
}

bool GroundServicesTask::authenticateRequest(httpd_req_t *req)
{
    char nonce[AUTH_HEADER_MAX_LEN] = {};
    char body_hash[AUTH_HEADER_MAX_LEN] = {};
    char signature[AUTH_HEADER_MAX_LEN] = {};

    if (!getHeader(req, "X-Auth-Nonce", nonce, sizeof(nonce))) return false;
    if (!getHeader(req, "X-Auth-Body-SHA256", body_hash, sizeof(body_hash))) return false;
    if (!getHeader(req, "X-Auth-Signature", signature, sizeof(signature))) return false;

    if (!isLowerHex(nonce, 32) || !isLowerHex(body_hash, AUTH_SHA256_HEX_LEN) || !isLowerHex(signature, AUTH_SIGNATURE_HEX_LEN)) {
        return false;
    }

    // Consume nonce before checking the signature to avoid online guesses with the same nonce.
    if (!consumeAuthNonce(nonce)) return false;

    char confirm[AUTH_HEADER_MAX_LEN] = {};
    getHeader(req, "X-Confirm", confirm, sizeof(confirm));

    char canonical[384];
    const int n = snprintf(canonical, sizeof(canonical), "%s\n%s\n%s\n%s\n%s",
                           methodToString(static_cast<httpd_method_t>(req->method)),
                           req->uri,
                           body_hash,
                           nonce,
                           confirm);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(canonical)) return false;

    char expected[65];
    if (!hmacSha256Hex(CONFIG_GROUND_SERVICES_AUTH_TOKEN, canonical, expected)) return false;
    return secureStringEqual(expected, signature);
}

esp_err_t GroundServicesTask::rootGetHandler(httpd_req_t *req) { return sendAsset(req, ground_index_html_start, ground_index_html_end, "text/html"); }
esp_err_t GroundServicesTask::styleGetHandler(httpd_req_t *req) { return sendAsset(req, ground_style_css_start, ground_style_css_end, "text/css"); }
esp_err_t GroundServicesTask::appJsGetHandler(httpd_req_t *req) { return sendAsset(req, ground_app_js_start, ground_app_js_end, "application/javascript"); }
esp_err_t GroundServicesTask::faviconGetHandler(httpd_req_t *req) { httpd_resp_set_status(req, "204 No Content"); httpd_resp_set_hdr(req, "Connection", "close"); (void)httpd_resp_send(req, nullptr, 0); return ESP_OK; }

esp_err_t GroundServicesTask::redirectToRootHandler(httpd_req_t *req)
{
    char host[96] = {};
    const bool has_host = getHeader(req, "Host", host, sizeof(host));
    char location[128];
    snprintf(location, sizeof(location), "http://%s/",
             has_host && host[0] != '\0' ? host : "192.168.4.1");

    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", location);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    (void)httpd_resp_sendstr(req, "Redirecting to Ground Services dashboard.");
    return ESP_OK;
}

esp_err_t GroundServicesTask::authNonceGetHandler(httpd_req_t *req)
{
    auto *self = fromReq(req);
    char nonce[33] = {};
    if (self == nullptr || !self->issueAuthNonce(nonce, AUTH_NONCE_TTL_MS)) {
        return sendErrorJson(req, "503 Service Unavailable", "failed to issue auth nonce");
    }

    char body[192];
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"nonce\":\"%s\",\"ttl_ms\":%lu,\"algorithm\":\"HMAC-SHA256\"}",
             nonce, static_cast<unsigned long>(AUTH_NONCE_TTL_MS));
    return sendJson(req, "200 OK", body);
}

esp_err_t GroundServicesTask::statusGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    auto *self = fromReq(req);
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    esp_chip_info_t chip = {};
    esp_chip_info(&chip);
    uint32_t flash_size = 0;
    uint32_t flash_id = 0;
    esp_flash_get_size(nullptr, &flash_size);
    esp_flash_read_id(nullptr, &flash_id);
    RuntimeConfig cfg = {};
    runtime_config_get(&cfg);
    char features[192];
    chipFeaturesToJson(chip.features, features, sizeof(features));

    char *body = s_responseBuffer;
    snprintf(body, RESPONSE_BUFFER_SIZE,
        "{\"ok\":true,\"fsm_state\":\"%s\",\"uptime_ms\":%lld,"
        "\"app\":{\"project_name\":\"%s\",\"version\":\"%s\",\"date\":\"%s\",\"time\":\"%s\",\"idf_version\":\"%s\"},"
        "\"chip\":{\"model\":\"%s\",\"model_code\":%d,\"cores\":%d,\"revision\":%d,\"features\":%s},"
        "\"flash\":{\"size_bytes\":%lu,\"id\":\"0x%06lx\"},"
        "\"memory\":{\"heap_total\":%lu,\"heap_free\":%lu,\"heap_min_free\":%lu,\"psram_total\":%lu,\"psram_free\":%lu},"
        "\"reset\":{\"code\":%d,\"reason\":\"%s\"},"
        "\"partitions\":{\"running\":{\"label\":\"%s\",\"address\":%lu,\"size\":%lu},"
        "\"configured_boot\":{\"label\":\"%s\",\"address\":%lu,\"size\":%lu}},"
        "\"network\":{\"ssid\":\"%s\",\"ip\":\"%s\",\"mac\":\"%s\"},"
        "\"security\":{\"secure_boot\":%s,\"flash_encryption\":%s},"
        "\"hil\":{\"support\":%s,\"simulation\":%s,\"mode\":\"%s\"},"
        "\"runtime_config\":{\"schema_version\":%lu,\"config_revision\":%lu,\"config_locked\":%s}}",
        self->_fsm ? rocketStateToString(self->_fsm->getCurrentState()) : "unknown",
        static_cast<long long>(esp_timer_get_time() / 1000),
        app ? app->project_name : "unknown",
        app ? app->version : "unknown",
        app ? app->date : "unknown",
        app ? app->time : "unknown",
        app ? app->idf_ver : esp_get_idf_version(),
        chipModelToString(chip.model), chip.model, chip.cores, chip.revision, features,
        static_cast<unsigned long>(flash_size), static_cast<unsigned long>(flash_id),
        static_cast<unsigned long>(heap_caps_get_total_size(MALLOC_CAP_8BIT)),
        static_cast<unsigned long>(esp_get_free_heap_size()),
        static_cast<unsigned long>(esp_get_minimum_free_heap_size()),
        static_cast<unsigned long>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM)),
        static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
        static_cast<int>(esp_reset_reason()),
        resetReasonToString(esp_reset_reason()),
        running ? running->label : "none",
        running ? static_cast<unsigned long>(running->address) : 0UL,
        running ? static_cast<unsigned long>(running->size) : 0UL,
        boot ? boot->label : "none",
        boot ? static_cast<unsigned long>(boot->address) : 0UL,
        boot ? static_cast<unsigned long>(boot->size) : 0UL,
        CONFIG_ROCKET_AP_SSID,
        self->_board ? self->_board->getWifiIpAddress() : "",
        self->_board ? self->_board->getWifiMacAddress() : "",
#ifdef CONFIG_SECURE_BOOT
        "true",
#else
        "false",
#endif
#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
        "true",
#else
        "false",
#endif
#if CONFIG_AURORA_HIL_SUPPORT
        "true",
#else
        "false",
#endif
#if CONFIG_AURORA_HIL_SIMULATION
        "true",
        "simulation",
#else
        "false",
        "flight",
#endif
        static_cast<unsigned long>(cfg.schema_version),
        static_cast<unsigned long>(cfg.config_revision),
        cfg.config_locked ? "true" : "false");
    return sendJson(req, "200 OK", body);
}

esp_err_t GroundServicesTask::healthGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    auto *self = fromReq(req);

    IMUData imu = {};
    AccelerometerSensorData acc = {};
    PressureSensorData baro1 = {};
    PressureSensorData baro2 = {};
    GPSData gps = {};

    SensorReadStatus imuStatus = self->_rocketModel ? self->_rocketModel->getBNO055Data(imu) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus accStatus = self->_rocketModel ? self->_rocketModel->getLIS3DHTRData(acc) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus baro1Status = self->_rocketModel ? self->_rocketModel->getMS561101BA03Data_1(baro1) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus baro2Status = self->_rocketModel ? self->_rocketModel->getMS561101BA03Data_2(baro2) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus gpsStatus = self->_rocketModel ? self->_rocketModel->getGPSData(gps) : SensorReadStatus::NOT_PRESENT;

    uint32_t flash_size = 0;
    esp_err_t flash_err = esp_flash_get_size(nullptr, &flash_size);
    const size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const bool externalFlashReady = self->_rocketModel && self->_rocketModel->isExternalFlashInitialized();
    const bool sdReady = self->_rocketModel && self->_rocketModel->isSdInitialized();

    char *body = s_responseBuffer;
    snprintf(body, RESPONSE_BUFFER_SIZE,
        "{\"ok\":true,"
        "\"memory\":{\"psram\":{\"present\":%s,\"expected\":false,\"status\":\"%s\",\"total_bytes\":%lu,\"free_bytes\":%lu}},"
        "\"internal_flash\":{\"present\":%s,\"size_bytes\":%lu,\"expected_size_bytes\":16777216,\"size_ok\":%s},"
        "\"external_flash\":{\"present\":%s,\"status\":\"%s\"},"
        "\"storage\":{\"sd\":{\"present\":%s,\"status\":\"%s\"}},"
        "\"sensors\":{"
        "\"imu_bno055\":{\"present\":%s,\"status\":\"%s\"},"
        "\"barometer_ms5611_primary\":{\"present\":%s,\"status\":\"%s\"},"
        "\"barometer_ms5611_secondary\":{\"present\":%s,\"status\":\"%s\"},"
        "\"accelerometer_lis3dhtr\":{\"present\":%s,\"status\":\"%s\"},"
        "\"gps\":{\"present\":%s,\"status\":\"%s\"}"
        "}}",
        psram_total > 0 ? "true" : "false",
        psram_total > 0 ? "ok" : "not_installed",
        static_cast<unsigned long>(psram_total),
        static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
        flash_err == ESP_OK ? "true" : "false",
        static_cast<unsigned long>(flash_size),
        flash_size == 16UL * 1024UL * 1024UL ? "true" : "false",
        externalFlashReady ? "true" : "false",
        externalFlashReady ? "ok" : "not_ready",
        sdReady ? "true" : "false",
        sdReady ? "ok" : "not_ready",
        imuStatus == SensorReadStatus::OK ? "true" : "false",
        sensorStatusToString(imuStatus),
        baro1Status == SensorReadStatus::OK ? "true" : "false",
        sensorStatusToString(baro1Status),
        baro2Status == SensorReadStatus::OK ? "true" : "false",
        sensorStatusToString(baro2Status),
        accStatus == SensorReadStatus::OK ? "true" : "false",
        sensorStatusToString(accStatus),
        gpsStatus == SensorReadStatus::OK ? "true" : "false",
        sensorStatusToString(gpsStatus));
    return sendJson(req, "200 OK", body);
}

static const char *sensorStatusToString(SensorReadStatus status)
{
    switch (status) {
        case SensorReadStatus::OK: return "ok";
        case SensorReadStatus::MUTEX_TIMEOUT: return "mutex_timeout";
        case SensorReadStatus::SENSOR_ERROR: return "sensor_error";
        case SensorReadStatus::NOT_PRESENT: return "not_present";
        default: return "unknown";
    }
}

esp_err_t GroundServicesTask::liveDataGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    auto *self = fromReq(req);
    IMUData imu = {};
    PressureSensorData baro = {};
    GPSData gps = {};
    SensorReadStatus imuStatus = self->_rocketModel ? self->_rocketModel->getBNO055Data(imu) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus baroStatus = self->_rocketModel ? self->_rocketModel->getMS561101BA03Data_1(baro) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus gpsStatus = self->_rocketModel ? self->_rocketModel->getGPSData(gps) : SensorReadStatus::NOT_PRESENT;

    char *body = s_responseBuffer;
    snprintf(body, RESPONSE_BUFFER_SIZE,
        "{\"ok\":true,\"fsm_state\":\"%s\",\"calibration\":{\"imu\":%s,\"barometer\":%s,\"barometer_samples\":%d},"
        "\"flight\":{\"height_m\":%.3f,\"vertical_speed_mps\":%.3f,\"is_rising\":%s},"
        "\"sensors\":{\"imu\":{\"status\":\"%s\",\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,\"temperature_c\":%.2f,\"timestamp\":%lu},"
        "\"barometer\":{\"status\":\"%s\",\"pressure\":%.3f,\"temperature_c\":%.3f,\"timestamp\":%lu},"
        "\"gps\":{\"status\":\"%s\",\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.2f,\"fix\":%s,\"satellites\":%u,\"timestamp\":%lu}}}",
        self->_fsm ? rocketStateToString(self->_fsm->getCurrentState()) : "unknown",
        self->_rocketModel && self->_rocketModel->isSensorSystemCalibrated() ? "true" : "false",
        self->_rocketModel && self->_rocketModel->isBarometerZeroed() ? "true" : "false",
        self->_rocketModel ? self->_rocketModel->getBarometerSampleCount() : 0,
        static_cast<double>(self->_rocketModel ? self->_rocketModel->getCurrentHeight() : 0.0f),
        static_cast<double>(self->_rocketModel ? self->_rocketModel->getHeightGainSpeed() : 0.0f),
        self->_rocketModel && self->_rocketModel->getIsRising() ? "true" : "false",
        sensorStatusToString(imuStatus),
        static_cast<double>(imu.acceleration_x), static_cast<double>(imu.acceleration_y), static_cast<double>(imu.acceleration_z),
        static_cast<double>(imu.temperature), static_cast<unsigned long>(imu.timestamp),
        sensorStatusToString(baroStatus),
        static_cast<double>(baro.pressure), static_cast<double>(baro.temperature), static_cast<unsigned long>(baro.timestamp),
        sensorStatusToString(gpsStatus),
        static_cast<double>(gps.latitude), static_cast<double>(gps.longitude), static_cast<double>(gps.altitude),
        gps.fixType >= 2 ? "true" : "false", gps.satellites, static_cast<unsigned long>(gps.timestamp));
    return sendJson(req, "200 OK", body);
}

esp_err_t GroundServicesTask::logsGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    const uint32_t since_seq = queryGetUint32(req, "since", 0);
    uint32_t latest_seq = since_seq;
    SerialLogger::getRecentLogsSince(since_seq, s_logResponseBuffer, LOG_RESPONSE_BUFFER_SIZE, &latest_seq);
    char latest_header[16];
    snprintf(latest_header, sizeof(latest_header), "%lu", static_cast<unsigned long>(latest_seq));
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "X-Log-Latest-Seq", latest_header);
    (void)httpd_resp_sendstr(req, s_logResponseBuffer);
    return ESP_OK;
}

esp_err_t GroundServicesTask::runtimeConfigGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    RuntimeConfig cfg;
    if (runtime_config_get(&cfg) != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_get failed");
    if (runtime_config_to_json(&cfg, s_responseBuffer, RESPONSE_BUFFER_SIZE) != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_to_json failed");
    return sendJson(req, "200 OK", s_responseBuffer);
}

esp_err_t GroundServicesTask::runtimeConfigPutHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    if (runtime_config_is_locked()) {
        discardBody(req);
        return sendErrorJson(req, "423 Locked", "configuration is locked for flight");
    }
    esp_err_t err = readBody(req, s_bodyBuffer, sizeof(s_bodyBuffer));
    if (err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid config body", err);
    RuntimeConfig updated;
    err = runtime_config_update_from_json(s_bodyBuffer, &updated);
    if (err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "config validation or save failed", err);
    runtime_config_to_json(&updated, s_responseBuffer, RESPONSE_BUFFER_SIZE);
    return sendJson(req, "200 OK", s_responseBuffer);
}

esp_err_t GroundServicesTask::runtimeConfigValidationGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    if (runtime_config_validation_json(s_responseBuffer, RESPONSE_BUFFER_SIZE) != ESP_OK) {
        return sendErrorJson(req, "500 Internal Server Error", "runtime_config_validation_json failed");
    }
    return sendJson(req, "200 OK", s_responseBuffer);
}

esp_err_t GroundServicesTask::runtimeConfigSchemaGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    if (runtime_config_schema_json(s_responseBuffer, RESPONSE_BUFFER_SIZE) != ESP_OK) {
        return sendErrorJson(req, "500 Internal Server Error", "runtime_config_schema_json response too large");
    }
    return sendJson(req, "200 OK", s_responseBuffer);
}

esp_err_t GroundServicesTask::runtimeConfigResetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    if (runtime_config_is_locked()) {
        discardBody(req);
        return sendErrorJson(req, "423 Locked", "configuration is locked for flight");
    }
    discardBody(req);
    esp_err_t err = runtime_config_reset_defaults();
    if (err != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_reset_defaults failed", err);
    RuntimeConfig cfg;
    if (runtime_config_get(&cfg) != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_get failed");
    if (runtime_config_to_json(&cfg, s_responseBuffer, RESPONSE_BUFFER_SIZE) != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_to_json failed");
    return sendJson(req, "200 OK", s_responseBuffer);
}

esp_err_t GroundServicesTask::runtimeConfigUnlockHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    auto *self = fromReq(req);
    if (!headerEquals(req, "X-Confirm", "UNLOCK_AFTER_RECOVERY")) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: UNLOCK_AFTER_RECOVERY header");
    }

    esp_err_t body_err = readBody(req, s_bodyBuffer, sizeof(s_bodyBuffer));
    if (body_err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid unlock body", body_err);
    if (strstr(s_bodyBuffer, "UNLOCK_AFTER_RECOVERY") == nullptr) {
        return sendErrorJson(req, "400 Bad Request", "unlock body must contain UNLOCK_AFTER_RECOVERY");
    }
    if (self == nullptr || self->_fsm == nullptr || self->_fsm->getCurrentState() != RocketState::GROUND_SERVICES) {
        return sendErrorJson(req, "409 Conflict", "unlock is only available in GROUND_SERVICES");
    }
    if (!runtime_config_is_locked()) {
        return sendErrorJson(req, "409 Conflict", "configuration is already unlocked");
    }

    esp_err_t err = runtime_config_unlock_after_recovery();
    if (err != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_unlock_after_recovery failed", err);

    RuntimeConfig cfg;
    if (runtime_config_get(&cfg) != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_get failed");
    if (runtime_config_to_json(&cfg, s_responseBuffer, RESPONSE_BUFFER_SIZE) != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_to_json failed");
    return sendJson(req, "200 OK", s_responseBuffer);
}

static bool appendChecklistItem(char *body, size_t body_size, size_t *offset, bool *all_ok,
                                bool ok, const char *key, const char *label, const char *severity, const char *message)
{
    if (body == nullptr || offset == nullptr || all_ok == nullptr) return false;
    if (!ok && strcmp(severity, "error") == 0) *all_ok = false;
    const int written = snprintf(body + *offset, body_size - *offset,
                                 "%s{\"key\":\"%s\",\"label\":\"%s\",\"ok\":%s,\"severity\":\"%s\",\"message\":\"%s\"}",
                                 *offset > 32 ? "," : "",
                                 key,
                                 label,
                                 boolJson(ok),
                                 severity,
                                 message);
    if (written < 0 || static_cast<size_t>(written) >= body_size - *offset) return false;
    *offset += static_cast<size_t>(written);
    return true;
}

static const char *recoveryModeToChecklistMessage(RecoveryMode mode)
{
    switch (mode) {
        case RecoveryMode::OneParachuteMode:
            return "Compiled recovery mode: one parachute";
        case RecoveryMode::TwoParachuteMode:
            return "Compiled recovery mode: two parachutes";
        default:
            return "Compiled recovery mode: unknown";
    }
}

esp_err_t GroundServicesTask::buildPrelaunchChecklistJson(GroundServicesTask *self, char *body, size_t body_size, bool *ok_out)
{
    if (self == nullptr || body == nullptr || ok_out == nullptr) return ESP_ERR_INVALID_ARG;

    RuntimeConfig cfg;
    char reason[128] = {};
    const bool config_loaded = runtime_config_get(&cfg) == ESP_OK;
    const bool config_valid = config_loaded && runtime_config_validate(&cfg, reason, sizeof(reason)) == ESP_OK;
    const bool config_unlocked = config_loaded && !cfg.config_locked;
    const bool barometer_ready = self->_rocketModel && self->_rocketModel->isBarometerZeroed();
    const bool imu_ready = self->_rocketModel && self->_rocketModel->isSensorSystemCalibrated();
    const bool telemetry_enabled = config_loaded && cfg.telemetry_enabled;
    const bool logging_enabled = config_loaded && cfg.logging_enabled;

    bool all_ok = true;
    size_t offset = 0;
    int written = snprintf(body, body_size, "{\"items\":[");
    if (written < 0 || static_cast<size_t>(written) >= body_size) return ESP_ERR_NO_MEM;
    offset = static_cast<size_t>(written);

    if (!appendChecklistItem(body, body_size, &offset, &all_ok, config_valid, "runtime_config_valid", "Runtime config valid", "error", config_valid ? "Runtime config validation passed" : reason)) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, config_unlocked, "config_unlocked", "Config not already locked", "error", config_unlocked ? "Configuration can be locked for flight" : "Configuration is already locked")) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, barometer_ready, "barometer_baseline", "Barometer baseline available", "error", barometer_ready ? "Barometer baseline is available" : "Barometer baseline is not ready")) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, imu_ready, "imu_calibrated", "IMU calibrated", "error", imu_ready ? "IMU calibration is ready" : "IMU calibration is not ready")) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, telemetry_enabled, "telemetry_enabled", "Telemetry enabled", "error", telemetry_enabled ? "Telemetry is enabled for flight" : "Telemetry must be enabled for flight")) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, logging_enabled, "logging_enabled", "Logging enabled", "error", logging_enabled ? "Logging is enabled for flight" : "Logging must be enabled for flight")) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, true, "recovery_mode", "Recovery mode", "info", recoveryModeToChecklistMessage(AURORA_RECOVERY_MODE))) return ESP_ERR_NO_MEM;

    written = snprintf(body + offset, body_size - offset, "],\"ok\":%s}", all_ok ? "true" : "false");
    if (written < 0 || static_cast<size_t>(written) >= body_size - offset) return ESP_ERR_NO_MEM;
    *ok_out = all_ok;
    return ESP_OK;
}

esp_err_t GroundServicesTask::prelaunchChecklistGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    bool checklist_ok = false;
    if (buildPrelaunchChecklistJson(fromReq(req), s_responseBuffer, RESPONSE_BUFFER_SIZE, &checklist_ok) != ESP_OK) {
        return sendErrorJson(req, "500 Internal Server Error", "prelaunch checklist response too large");
    }
    return sendJson(req, "200 OK", s_responseBuffer);
}


static constexpr size_t SDKCONFIG_CHUNK_SIZE = 1024;
static char s_sdkconfigChunk[SDKCONFIG_CHUNK_SIZE];

static bool containsLiteral(const char *data, size_t len, const char *needle)
{
    const size_t needle_len = strlen(needle);

    if (data == nullptr || needle == nullptr) {
        return false;
    }

    if (needle_len == 0 || len < needle_len) {
        return false;
    }

    for (size_t i = 0; i + needle_len <= len; ++i) {
        if (memcmp(data + i, needle, needle_len) == 0) {
            return true;
        }
    }

    return false;
}

esp_err_t GroundServicesTask::sdkconfigGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;

    size_t len = static_cast<size_t>(ground_sdkconfig_end - ground_sdkconfig_start);
    if (len > 0 && ground_sdkconfig_start[len - 1] == '\0') {
        len--;
    }

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");

    const char *text = reinterpret_cast<const char *>(ground_sdkconfig_start);

    size_t pos = 0;
    size_t out_len = 0;

    auto flush = [&]() -> esp_err_t {
        if (out_len == 0) return ESP_OK;

        esp_err_t err = httpd_resp_send_chunk(req, s_sdkconfigChunk, out_len);
        out_len = 0;

        // Client closed the connection. This is not a firmware fault.
        // Return ESP_OK to avoid noisy "uri handler execution failed".
        if (err != ESP_OK) {
            return ESP_FAIL;
        }

        return ESP_OK;
    };

    auto append = [&](const char *data, size_t data_len) -> esp_err_t {
        while (data_len > 0) {
            const size_t room = sizeof(s_sdkconfigChunk) - out_len;
            if (room == 0) {
                esp_err_t err = flush();
                if (err != ESP_OK) return err;
                continue;
            }

            const size_t copy_len = std::min(room, data_len);
            memcpy(s_sdkconfigChunk + out_len, data, copy_len);
            out_len += copy_len;
            data += copy_len;
            data_len -= copy_len;
        }

        return ESP_OK;
    };

    while (pos < len) {
        const size_t line_start = pos;
        size_t line_len = 0;

        while (pos + line_len < len && text[pos + line_len] != '\n') {
            line_len++;
        }

        const bool sensitive =
            containsLiteral(text + line_start, line_len, "PASSWORD") ||
            containsLiteral(text + line_start, line_len, "AUTH_TOKEN") ||
            containsLiteral(text + line_start, line_len, "SECRET") ||
            containsLiteral(text + line_start, line_len, "PRIVATE_KEY");

        if (sensitive) {
            const char *eq = static_cast<const char *>(memchr(text + line_start, '=', line_len));
            if (eq != nullptr) {
                const size_t prefix_len = static_cast<size_t>(eq - (text + line_start)) + 1;
                if (append(text + line_start, prefix_len) != ESP_OK) return ESP_OK;
                if (append("\"<redacted>\"", strlen("\"<redacted>\"")) != ESP_OK) return ESP_OK;
            } else {
                if (append("<redacted>", strlen("<redacted>")) != ESP_OK) return ESP_OK;
            }
        } else {
            if (append(text + line_start, line_len) != ESP_OK) return ESP_OK;
        }

        if (pos + line_len < len && text[pos + line_len] == '\n') {
            if (append("\n", 1) != ESP_OK) return ESP_OK;
            pos += line_len + 1;
        } else {
            pos += line_len;
        }
    }

    if (flush() != ESP_OK) {
        return ESP_OK;
    }

    return httpd_resp_send_chunk(req, nullptr, 0);
}

// esp_err_t GroundServicesTask::startCalibrationPostHandler(httpd_req_t *req)
// {
//     if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
//     auto *self = fromReq(req);
//     if (!headerEquals(req, "X-Confirm", "START_CALIBRATION")) {
//         discardBody(req);
//         return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: START_CALIBRATION header");
//     }
//     discardBody(req);
//     if (self->_fsm == nullptr || self->_fsm->getCurrentState() != RocketState::GROUND_SERVICES) {
//         return sendErrorJson(req, "409 Conflict", "start calibration is only available in GROUND_SERVICES");
//     }
//     if (!self->_fsm->sendEvent(FSMEvent::START_CALIBRATION)) {
//         return sendErrorJson(req, "500 Internal Server Error", "failed to queue START_CALIBRATION");
//     }
//     return sendJson(req, "200 OK", "{\"ok\":true,\"queued\":\"START_CALIBRATION\"}");
// }

esp_err_t GroundServicesTask::readyForLaunchPostHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    auto *self = fromReq(req);
    if (!headerEquals(req, "X-Confirm", "READY_FOR_LAUNCH")) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: READY_FOR_LAUNCH header");
    }
    esp_err_t body_err = readBody(req, s_bodyBuffer, sizeof(s_bodyBuffer));
    if (body_err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid ready-for-launch body", body_err);
    if (self->_fsm == nullptr || self->_fsm->getCurrentState() != RocketState::GROUND_SERVICES) {
        return sendErrorJson(req, "409 Conflict", "ready-for-launch is only available in GROUND_SERVICES");
    }

    bool checklist_ok = false;
    if (buildPrelaunchChecklistJson(self, s_responseBuffer, RESPONSE_BUFFER_SIZE, &checklist_ok) != ESP_OK) {
        return sendErrorJson(req, "500 Internal Server Error", "prelaunch checklist response too large");
    }
    const bool manual_override = strstr(s_bodyBuffer, "READY_FOR_LAUNCH") != nullptr;
    if (!checklist_ok && !manual_override) {
        return sendJson(req, "409 Conflict", s_responseBuffer);
    }
    if (!checklist_ok && manual_override) {
        LOG_WARNING(TAG, "READY_FOR_LAUNCH manual checklist override accepted");
    }

    char reason[128] = {};
    esp_err_t err = runtime_config_lock_for_flight(reason, sizeof(reason));
    if (err != ESP_OK) {
        char body[256];
        snprintf(body, sizeof(body),
                 "{\"ok\":false,\"error\":\"failed to lock flight configuration\",\"reason\":\"%s\"}",
                 reason);
        return sendJson(req, "400 Bad Request", body);
    }

    if (!self->_fsm->sendEvent(FSMEvent::START_READY_FOR_LAUNCH)) {
        return sendErrorJson(req, "500 Internal Server Error", "failed to queue START_READY_FOR_LAUNCH");
    }
    return sendJson(req, "200 OK", manual_override
        ? "{\"ok\":true,\"queued\":\"START_READY_FOR_LAUNCH\",\"config_locked\":true,\"manual_override\":true}"
        : "{\"ok\":true,\"queued\":\"START_READY_FOR_LAUNCH\",\"config_locked\":true,\"manual_override\":false}");
}

esp_err_t GroundServicesTask::testsListGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    auto *self = fromReq(req);
    if (!self->_testRunner) {
        return sendJson(req, "200 OK", "{\"ok\":true,\"available\":false,\"tests\":[]}");
    }

    char *body = s_responseBuffer;
    size_t offset = 0;
    int written = snprintf(body + offset, RESPONSE_BUFFER_SIZE - offset,
                           "{\"ok\":true,\"available\":true,\"tests\":[");
    if (written < 0) return ESP_FAIL;
    offset += static_cast<size_t>(written);

    for (size_t i = 0; i < self->_testRunner->getGroundTestCount(); ++i) {
        const GroundTestDescriptor *test = self->_testRunner->getGroundTest(i);
        if (test == nullptr) continue;
        written = snprintf(body + offset, RESPONSE_BUFFER_SIZE - offset,
                           "%s{\"id\":%d,\"group\":\"%s\",\"name\":\"%s\",\"description\":\"%s\","
                           "\"destructive\":%s,\"confirmation\":\"%s\"}",
                           i == 0 ? "" : ",",
                           test->id,
                           test->group,
                           test->name,
                           test->description,
                           boolJson(test->destructive),
                           test->confirmation ? test->confirmation : "");
        if (written < 0 || static_cast<size_t>(written) >= RESPONSE_BUFFER_SIZE - offset) {
            return sendErrorJson(req, "500 Internal Server Error", "test list response too large");
        }
        offset += static_cast<size_t>(written);
    }

    written = snprintf(body + offset, RESPONSE_BUFFER_SIZE - offset, "]}");
    if (written < 0 || static_cast<size_t>(written) >= RESPONSE_BUFFER_SIZE - offset) {
        return sendErrorJson(req, "500 Internal Server Error", "test list response too large");
    }
    return sendJson(req, "200 OK", body);
}

esp_err_t GroundServicesTask::testsStatusGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    auto *self = fromReq(req);
    if (!self->_testRunner) {
        return sendJson(req, "200 OK", "{\"ok\":true,\"available\":false,\"state\":\"unavailable\"}");
    }

    GroundTestStatus status;
    self->_testRunner->getGroundTestStatus(&status);
    char body[768];
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"available\":true,\"running\":%s,\"waiting_for_verdict\":%s,"
             "\"last_passed\":%s,\"active_id\":%d,\"active_name\":\"%s\","
             "\"state\":\"%s\",\"prompt\":\"%s\",\"message\":\"%s\","
             "\"started_ms\":%lu,\"finished_ms\":%lu}",
             boolJson(status.running),
             boolJson(status.waiting_for_verdict),
             boolJson(status.last_passed),
             status.active_id,
             status.active_name,
             status.state,
             status.prompt,
             status.message,
             static_cast<unsigned long>(status.started_ms),
             static_cast<unsigned long>(status.finished_ms));
    return sendJson(req, "200 OK", body);
}

esp_err_t GroundServicesTask::testsStartPostHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    auto *self = fromReq(req);
    if (!self->_testRunner) {
        discardBody(req);
        return sendErrorJson(req, "503 Service Unavailable", "test runner unavailable");
    }
    if (!self->_fsm || self->_fsm->getCurrentState() != RocketState::GROUND_SERVICES) {
        discardBody(req);
        return sendErrorJson(req, "409 Conflict", "tests are only available in GROUND_SERVICES");
    }

    esp_err_t err = readBody(req, s_bodyBuffer, sizeof(s_bodyBuffer));
    if (err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid test start body", err);

    int id = 0;
    if (!bodyGetInt(s_bodyBuffer, "id", &id)) {
        return sendErrorJson(req, "400 Bad Request", "missing numeric test id");
    }

    const GroundTestDescriptor *descriptor = nullptr;
    for (size_t i = 0; i < self->_testRunner->getGroundTestCount(); ++i) {
        const GroundTestDescriptor *candidate = self->_testRunner->getGroundTest(i);
        if (candidate && candidate->id == id) {
            descriptor = candidate;
            break;
        }
    }
    if (descriptor == nullptr) {
        return sendErrorJson(req, "404 Not Found", "unknown test id");
    }
    if (descriptor->destructive && !headerEquals(req, "X-Confirm", descriptor->confirmation)) {
        return sendErrorJson(req, "400 Bad Request", "missing required X-Confirm header for destructive test");
    }

    char error[128] = {};
    if (!self->_testRunner->startGroundTest(id, error, sizeof(error))) {
        return sendErrorJson(req, "409 Conflict", error[0] ? error : "failed to start test");
    }

    char response[256];
    snprintf(response, sizeof(response), "{\"ok\":true,\"started\":%d,\"name\":\"%s\"}", id, descriptor->name);
    return sendJson(req, "200 OK", response);
}

esp_err_t GroundServicesTask::testsVerdictPostHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    auto *self = fromReq(req);
    if (!self->_testRunner) {
        discardBody(req);
        return sendErrorJson(req, "503 Service Unavailable", "test runner unavailable");
    }

    esp_err_t err = readBody(req, s_bodyBuffer, sizeof(s_bodyBuffer));
    if (err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid verdict body", err);

    char verdict_text[24] = {};
    if (!bodyGetString(s_bodyBuffer, "verdict", verdict_text, sizeof(verdict_text))) {
        return sendErrorJson(req, "400 Bad Request", "missing verdict");
    }

    GroundTestVerdict verdict = GroundTestVerdict::FAILED;
    if (strcmp(verdict_text, "passed") == 0) {
        verdict = GroundTestVerdict::PASSED;
    } else if (strcmp(verdict_text, "failed") == 0) {
        verdict = GroundTestVerdict::FAILED;
    } else if (strcmp(verdict_text, "retry") == 0) {
        verdict = GroundTestVerdict::RETRY;
    } else if (strcmp(verdict_text, "exit") == 0) {
        verdict = GroundTestVerdict::EXIT;
    } else if (strcmp(verdict_text, "reboot") == 0) {
        if (!headerEquals(req, "X-Confirm", "REBOOT_FROM_TEST")) {
            return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: REBOOT_FROM_TEST header");
        }
        verdict = GroundTestVerdict::REBOOT;
    } else {
        return sendErrorJson(req, "400 Bad Request", "unknown verdict");
    }

    char error[128] = {};
    if (!self->_testRunner->submitGroundTestVerdict(verdict, error, sizeof(error))) {
        return sendErrorJson(req, "409 Conflict", error[0] ? error : "failed to submit verdict");
    }
    return sendJson(req, "200 OK", "{\"ok\":true}");
}

esp_err_t GroundServicesTask::otaStatusGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    auto status = fromReq(req)->getOtaStatus();
    int progress = status.total_size > 0 ? static_cast<int>((status.bytes_written * 100) / status.total_size) : 0;
    char body[512];
    snprintf(body, sizeof(body),
        "{\"ok\":true,\"state\":\"%s\",\"bytes_written\":%lu,\"total_size\":%lu,\"progress\":%d,\"last_error\":\"%s\",\"message\":\"%s\",\"sha256\":\"%s\"}",
        otaStateToString(status.state),
        static_cast<unsigned long>(status.bytes_written),
        static_cast<unsigned long>(status.total_size),
        progress,
        status.last_error == ESP_OK ? "ESP_OK" : esp_err_to_name(status.last_error),
        status.message,
        status.sha256);
    return sendJson(req, "200 OK", body);
}

esp_err_t GroundServicesTask::otaUploadPostHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    auto *self = fromReq(req);

    if (!self->_fsm || self->_fsm->getCurrentState() != RocketState::GROUND_SERVICES) {
        discardBody(req);
        return sendErrorJson(req, "409 Conflict", "OTA is only available in GROUND_SERVICES");
    }

    if (!headerEquals(req, "X-Confirm", "START_OTA")) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: START_OTA header");
    }

    char signed_body_hash[65];
    if (!getSignedBodyHash(req, signed_body_hash)) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing or invalid X-Auth-Body-SHA256 header");
    }

    char firmware_hash[AUTH_HEADER_MAX_LEN] = {};
    if (!getHeader(req, "X-Firmware-SHA256", firmware_hash, sizeof(firmware_hash)) ||
        !isLowerHex(firmware_hash, AUTH_SHA256_HEX_LEN) ||
        !secureStringEqual(firmware_hash, signed_body_hash)) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing or unsigned X-Firmware-SHA256 header");
    }

    if (self->getOtaStatus().state == OtaState::WRITING) {
        discardBody(req);
        return sendErrorJson(req, "409 Conflict", "OTA already in progress");
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);
    if (update_partition == nullptr) return sendErrorJson(req, "500 Internal Server Error", "no OTA update partition available");
    if (req->content_len == 0) return sendErrorJson(req, "400 Bad Request", "empty firmware image");
    if (req->content_len > update_partition->size) {
        discardBody(req);
        return sendErrorJson(req, "413 Payload Too Large", "firmware image larger than update partition");
    }

    self->setOtaStatus(OtaState::WRITING, 0, req->content_len, ESP_OK, "writing firmware");
    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, req->content_len, &ota_handle);
    if (err != ESP_OK) {
        self->setOtaStatus(OtaState::FAILED, 0, req->content_len, err, "esp_ota_begin failed");
        discardBody(req);
        return sendErrorJson(req, "500 Internal Server Error", "esp_ota_begin failed", err);
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);

    size_t remaining = req->content_len;
    size_t written = 0;
    while (remaining > 0) {
        const size_t to_read = std::min(remaining, OTA_UPLOAD_BUFFER_SIZE);
        int received = httpd_req_recv(req, s_otaUploadBuffer, static_cast<int>(to_read));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
            mbedtls_sha256_free(&sha);
            esp_ota_abort(ota_handle);
            self->setOtaStatus(OtaState::FAILED, written, req->content_len, ESP_FAIL, "http receive failed");
            return sendErrorJson(req, "500 Internal Server Error", "http receive failed");
        }

        mbedtls_sha256_update(&sha, reinterpret_cast<const uint8_t *>(s_otaUploadBuffer), static_cast<size_t>(received));

        err = esp_ota_write(ota_handle, s_otaUploadBuffer, static_cast<size_t>(received));
        if (err != ESP_OK) {
            mbedtls_sha256_free(&sha);
            esp_ota_abort(ota_handle);
            self->setOtaStatus(OtaState::FAILED, written, req->content_len, err, "esp_ota_write failed");
            return sendErrorJson(req, "500 Internal Server Error", "esp_ota_write failed", err);
        }
        written += static_cast<size_t>(received);
        remaining -= static_cast<size_t>(received);
        self->setOtaStatus(OtaState::WRITING, written, req->content_len, ESP_OK, "writing firmware");
        taskYIELD();
    }

    uint8_t digest[32];
    char actual_hash[65];
    mbedtls_sha256_finish(&sha, digest);
    mbedtls_sha256_free(&sha);
    bytesToHex(digest, sizeof(digest), actual_hash, sizeof(actual_hash));

    if (!secureStringEqual(actual_hash, firmware_hash)) {
        esp_ota_abort(ota_handle);
        self->setOtaStatus(OtaState::FAILED, written, req->content_len, ESP_ERR_INVALID_STATE, "firmware SHA-256 mismatch", actual_hash);
        return sendErrorJson(req, "400 Bad Request", "firmware SHA-256 mismatch", ESP_ERR_INVALID_STATE);
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        self->setOtaStatus(OtaState::FAILED, written, req->content_len, err, "esp_ota_end image validation failed", actual_hash);
        return sendErrorJson(req, "400 Bad Request", "esp_ota_end image validation failed", err);
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        self->setOtaStatus(OtaState::FAILED, written, req->content_len, err, "esp_ota_set_boot_partition failed", actual_hash);
        return sendErrorJson(req, "500 Internal Server Error", "esp_ota_set_boot_partition failed", err);
    }

    self->setOtaStatus(OtaState::READY_TO_REBOOT, written, req->content_len, ESP_OK, "image valid; reboot required", actual_hash);
    char body[384];
    snprintf(body, sizeof(body), "{\"ok\":true,\"bytes_written\":%lu,\"sha256\":\"%s\",\"next_boot_partition\":\"%s\",\"reboot_required\":true}",
             static_cast<unsigned long>(written), actual_hash, update_partition->label);
    return sendJson(req, "200 OK", body);
}

static void delayedRebootTask(void *)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

esp_err_t GroundServicesTask::otaRebootPostHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    auto *self = fromReq(req);
    if (!headerEquals(req, "X-Confirm", "REBOOT_TO_NEW_FIRMWARE")) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: REBOOT_TO_NEW_FIRMWARE header");
    }
    esp_err_t err = readBody(req, s_bodyBuffer, sizeof(s_bodyBuffer));
    if (err != ESP_OK || strstr(s_bodyBuffer, "REBOOT_TO_NEW_FIRMWARE") == nullptr) {
        return sendErrorJson(req, "400 Bad Request", "body must contain REBOOT_TO_NEW_FIRMWARE", err);
    }
    if (self->getOtaStatus().state != OtaState::READY_TO_REBOOT) {
        return sendErrorJson(req, "409 Conflict", "no uploaded firmware is ready to boot");
    }
    if (xTaskCreate(delayedRebootTask, "ota_reboot", 2048, nullptr, 5, nullptr) != pdPASS) {
        return sendErrorJson(req, "500 Internal Server Error", "failed to create reboot task", ESP_ERR_NO_MEM);
    }
    return sendJson(req, "200 OK", "{\"ok\":true,\"rebooting\":true}");
}
