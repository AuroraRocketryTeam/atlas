/*
 * Ground Services is the pre-flight operator interface and runs only while its
 * FSM state is active. BoardHardware provides idempotent NVS and shared SoftAP
 * access (HIL may request the same AP); this task owns the ESP-IDF HTTP server.
 *
 * The browser app is compiled into the firmware. Short HTTP requests provide
 * status and perform actions; /ws/live-data and /ws/logs provide public,
 * read-only, best-effort streams. The Ground Services task requests updates,
 * httpd_queue_work() moves them into the HTTPD task, and at most one update is
 * pending. All socket access stays in HTTPD context. Static buffers, fixed client
 * limits, short send timeouts, and dropping slow WebSockets bound resource use.
 *
 * WiFi access and operator authentication are separate. Critical mutations use
 * a short-lived, single-use nonce and HMAC-SHA256 over method, path, body hash,
 * nonce, and X-Confirm. Monitoring needs no operator secret. Synchronous OTA is
 * exclusive: streaming pauses and other mutations are rejected while writing.
 *
 * RuntimeConfig remains owned by its module and persisted through BoardHardware
 * NVS. READY performs authenticated checks and only queues an FSM event; the FSM
 * is the authority that validates, locks the flight snapshot, and transitions.
 * Shutdown stops HTTPD before releasing WiFi and synchronization resources.
 *
 * Pages:
 * - Info: identity, FSM state, firmware details, and pre-launch checklist.
 * - Health: sensor, hardware, task, memory, HTTP, and WebSocket diagnostics.
 * - Live Data: current sensor values, time series, acceleration, and attitude.
 * - Config: launch-site edits and organized inspection of the complete setup.
 * - OTA: authenticated firmware upload, validation, and reboot.
 * - Tests: authenticated guided hardware tests with live operator logs.
 * - Serial Monitor: public live device logs with pause, filter, and copy tools.
 * - Files: authenticated listing, streaming download, and deletion of flight data.
 */
#include "GroundServicesTask.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <climits>
#include <new>
#include <sys/socket.h>
#include <sys/time.h>

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
#include "esp_wifi.h"
#include "config.h"
#include "IGroundTestRunner.hpp"
#include "SerialLogger.hpp"
#include "utils.h"

static const char *TAG = "GroundServices";
// Live Data is best-effort operator monitoring; 10 Hz is responsive without
// creating unnecessary HTTPD and Wi-Fi load on the flight controller.
static constexpr uint32_t GROUND_SERVICES_LOOP_PERIOD_MS = 100;
static constexpr uint32_t GROUND_STACK_LOG_PERIOD_MS = 5000;

#ifndef CONFIG_GROUND_SERVICES_AUTH_TOKEN
#error "CONFIG_GROUND_SERVICES_AUTH_TOKEN must be configured. It is used as the HMAC shared secret for Ground Services."
#endif

static constexpr size_t AUTH_HEADER_MAX_LEN = 96;
static constexpr size_t AUTH_SHA256_HEX_LEN = 64;
static constexpr size_t AUTH_SIGNATURE_HEX_LEN = 64;
static constexpr size_t BODY_MAX_LEN = 4096;
static constexpr size_t RESPONSE_BUFFER_SIZE = 12288;
static constexpr size_t STREAM_CHUNK_SIZE = 4096;
static constexpr size_t LOG_RESPONSE_BUFFER_SIZE = 4096;
static constexpr size_t MAX_HTTP_CLIENTS = 4;
static constexpr size_t MAX_WS_CLIENTS = 2;
static constexpr size_t REGISTERED_URI_HANDLERS = 31;
static constexpr size_t HTTPD_STACK_SIZE = 5120;
static constexpr size_t LOG_WS_PREFIX_SPACE = 16;
static constexpr suseconds_t WS_SEND_TIMEOUT_US = 200000;
static constexpr uint32_t AUTH_NONCE_TTL_MS = 30000;
static_assert(MAX_HTTP_CLIENTS >= MAX_WS_CLIENTS + 2, "WebSockets must leave room for configuration HTTP requests");
static_assert(STREAM_CHUNK_SIZE <= RESPONSE_BUFFER_SIZE, "Stream chunks must fit in the shared response buffer");

// HTTP handlers and queued WebSocket work execute serially in HTTPD context,
// so request, response, and log payloads can share one buffer.
alignas(StorageFileInfo) static char s_responseBuffer[RESPONSE_BUFFER_SIZE];
static std::atomic_bool s_wsBroadcastQueued{false};
static std::atomic_bool s_httpdStackWarningIssued{false};
static size_t s_nextLogClient = 0;
static std::atomic_uint32_t s_wsSendFailures{0};
static std::atomic_uint32_t s_wsSlowClientDrops{0};
static std::atomic_uint32_t s_broadcastQueueFailures{0};
static std::atomic_uint32_t s_broadcastCoalesced{0};
static std::atomic_uint32_t s_wsLimitRejects{0};
static std::atomic_uint32_t s_httpSessionsOpened{0};
static std::atomic_uint32_t s_httpCapacityEvents{0};
extern const unsigned char ground_index_html_start[]     asm("_binary_index_html_start");
extern const unsigned char ground_index_html_end[]       asm("_binary_index_html_end");
extern const unsigned char ground_style_css_start[]      asm("_binary_style_css_start");
extern const unsigned char ground_style_css_end[]        asm("_binary_style_css_end");
extern const unsigned char ground_app_js_start[]         asm("_binary_app_js_start");
extern const unsigned char ground_app_js_end[]           asm("_binary_app_js_end");
extern const unsigned char ground_sdkconfig_start[]      asm("_binary_embedded_sdkconfig_txt_start");
extern const unsigned char ground_sdkconfig_end[]        asm("_binary_embedded_sdkconfig_txt_end");

enum class WsStream : uint8_t { LIVE_DATA, LOGS };

struct WsSession {
    bool active = false;
    WsStream stream = WsStream::LIVE_DATA;
    int socket = -1;
    uint32_t last_log_seq = 0;
};

static WsSession s_wsSessions[MAX_WS_CLIENTS];

struct ClientCounts {
    size_t http = 0;
    size_t websocket = 0;
    size_t live = 0;
    size_t logs = 0;
};

static ClientCounts getClientCounts(httpd_handle_t server)
{
    ClientCounts result;
    if (server == nullptr) return result;
    size_t count = MAX_HTTP_CLIENTS;
    int clients[MAX_HTTP_CLIENTS] = {};
    if (httpd_get_client_list(server, &count, clients) != ESP_OK) return result;
    result.http = count;
    for (size_t i = 0; i < count; ++i) {
        if (httpd_ws_get_fd_info(server, clients[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        result.websocket++;
        auto *session = static_cast<WsSession *>(httpd_sess_get_ctx(server, clients[i]));
        if (session == nullptr || !session->active) continue;
        if (session->stream == WsStream::LIVE_DATA) result.live++;
        else result.logs++;
    }
    return result;
}

static void releaseWsSession(void *ctx)
{
    static_cast<WsSession *>(ctx)->active = false;
}

static bool sendWsText(httpd_handle_t server, int socket, char *payload, size_t length)
{
    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = reinterpret_cast<uint8_t *>(payload);
    frame.len = length;
    // Despite its name this low-level API sends immediately. This runs as
    // queued HTTP-server work, so socket access stays in the server task.
    if (httpd_ws_send_frame_async(server, socket, &frame) == ESP_OK) {
        httpd_sess_update_lru_counter(server, socket);
        return true;
    }
    const int send_errno = errno;
    s_wsSendFailures.fetch_add(1, std::memory_order_relaxed);
    const bool slow = send_errno == EAGAIN || send_errno == EWOULDBLOCK || send_errno == ETIMEDOUT;
    if (slow) {
        s_wsSlowClientDrops.fetch_add(1, std::memory_order_relaxed);
    }
    LOG_WARNING(TAG, "Dropping WebSocket fd=%d after send failure: errno=%d%s",
                socket, send_errno, slow ? " (slow client/timeout)" : "");
    shutdown(socket, SHUT_RDWR);
    return false;
}

static esp_err_t httpClientOpened(httpd_handle_t server, int)
{
    s_httpSessionsOpened.fetch_add(1, std::memory_order_relaxed);
    size_t count = MAX_HTTP_CLIENTS;
    int clients[MAX_HTTP_CLIENTS] = {};
    if (httpd_get_client_list(server, &count, clients) == ESP_OK && count >= MAX_HTTP_CLIENTS) {
        s_httpCapacityEvents.fetch_add(1, std::memory_order_relaxed);
        LOG_WARNING(TAG, "HTTP client capacity reached (%u/%u); HTTPD may purge the LRU session",
                    static_cast<unsigned>(count), static_cast<unsigned>(MAX_HTTP_CLIENTS));
    }
    return ESP_OK;
}

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
      _authMutex(nullptr),
      _otaExclusive(false)
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

static bool getSafeFileName(httpd_req_t *req, char *out, size_t outSize)
{
    if (req == nullptr || out == nullptr || outSize == 0) return false;
    char query[128] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "name", out, outSize) != ESP_OK || out[0] == '\0' ||
        strstr(out, "..") != nullptr) {
        return false;
    }
    for (const unsigned char *p = reinterpret_cast<const unsigned char *>(out); *p != '\0'; ++p) {
        const unsigned char c = *p;
        if (!(std::isalnum(c) || c == '.' || c == '_' || c == '-')) return false;
    }
    return true;
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
    // BaseTask calls onTaskStop() only after taskFunction() has returned, so
    // queueWebSocketBroadcast() can no longer race httpd_stop() or mutex teardown.
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
    uint32_t lastStackLogMs = millis() - GROUND_STACK_LOG_PERIOD_MS;
    while (running) {
        esp_task_wdt_reset();
        queueWebSocketBroadcast();
        const uint32_t now = millis();
        if (now - lastStackLogMs >= GROUND_STACK_LOG_PERIOD_MS) {
            LOG_INFO(TAG, "Stack remaining: %lu bytes", static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)));
            lastStackLogMs = now;
        }
        vTaskDelay(pdMS_TO_TICKS(GROUND_SERVICES_LOOP_PERIOD_MS));
    }
}

void GroundServicesTask::queueWebSocketBroadcast()
{
    if (_server == nullptr || _otaExclusive.load(std::memory_order_acquire)) return;
    if (s_wsBroadcastQueued.exchange(true, std::memory_order_acq_rel)) {
        s_broadcastCoalesced.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (httpd_queue_work(_server, webSocketBroadcastWork, this) != ESP_OK) {
        s_broadcastQueueFailures.fetch_add(1, std::memory_order_relaxed);
        s_wsBroadcastQueued.store(false, std::memory_order_release);
        LOG_WARNING(TAG, "Failed to queue WebSocket broadcast; broadcasting remains enabled");
    }
}

void GroundServicesTask::webSocketBroadcastWork(void *arg)
{
    auto *self = static_cast<GroundServicesTask *>(arg);
    static uint32_t lastStackCheckMs = 0;
    const uint32_t now = Utils::realMillis();
    if (now - lastStackCheckMs >= 5000) {
        lastStackCheckMs = now;
        const uint32_t stackRemaining = uxTaskGetStackHighWaterMark(nullptr);
        if (stackRemaining * 100U < HTTPD_STACK_SIZE * 15U &&
            !s_httpdStackWarningIssued.exchange(true, std::memory_order_relaxed)) {
            LOG_WARNING(TAG, "HTTPD stack usage above 85%% (remaining=%lu/%u bytes)",
                        static_cast<unsigned long>(stackRemaining), static_cast<unsigned>(HTTPD_STACK_SIZE));
        }
    }
    if (self->_server != nullptr && !self->_otaExclusive.load(std::memory_order_acquire)) {
        self->broadcastLiveData();
        self->broadcastLogs();
    }
    s_wsBroadcastQueued.store(false, std::memory_order_release);
}

esp_err_t GroundServicesTask::startServer()
{
    if (_server != nullptr) return ESP_OK;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = REGISTERED_URI_HANDLERS;
    config.stack_size = HTTPD_STACK_SIZE;
    config.max_open_sockets = MAX_HTTP_CLIENTS;
    config.backlog_conn = 2;
    config.recv_wait_timeout = 20;
    config.send_wait_timeout = 1;
    config.lru_purge_enable = true;
    config.open_fn = httpClientOpened;
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
    _otaExclusive.store(false, std::memory_order_release);
    s_wsBroadcastQueued.store(false, std::memory_order_release);
    s_nextLogClient = 0;
    for (auto &session : s_wsSessions) session.active = false;
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
    auto add_ws = [this](const char *uri, esp_err_t (*pre_handshake)(httpd_req_t *)) {
        httpd_uri_t h = {};
        h.uri = uri;
        h.method = HTTP_GET;
        h.handler = readOnlyWsHandler;
        h.user_ctx = this;
        h.is_websocket = true;
        h.handle_ws_control_frames = false;
        h.ws_pre_handshake_cb = pre_handshake;
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

    add_ws("/ws/live-data", liveDataWsPreHandshake);
    add_ws("/ws/logs", logsWsPreHandshake);
    add("/api/logs", HTTP_GET, logsGetHandler);
    add("/api/config/runtime", HTTP_GET, runtimeConfigGetHandler);
    add("/api/config/runtime", HTTP_PUT, runtimeConfigPutHandler);
    add("/api/config/schema", HTTP_GET, runtimeConfigSchemaGetHandler);
    add("/api/config/validation", HTTP_GET, runtimeConfigValidationGetHandler);
    add("/api/config/reset-defaults", HTTP_POST, runtimeConfigResetHandler);
    add("/api/config/unlock-after-recovery", HTTP_POST, runtimeConfigUnlockHandler);
    add("/api/info/sdkconfig", HTTP_GET, sdkconfigGetHandler);
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
    add("/api/files", HTTP_GET, filesListGetHandler);
    add("/api/files", HTTP_DELETE, fileDeleteHandler);
    add("/api/files/download", HTTP_GET, fileDownloadGetHandler);
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
    _otaExclusive.store(state == OtaState::WRITING, std::memory_order_release);
}

GroundServicesTask::OtaStatus GroundServicesTask::getOtaStatus() const
{
    OtaStatus copy;
    if (_otaMutex != nullptr) xSemaphoreTake(_otaMutex, portMAX_DELAY);
    copy = _otaStatus;
    if (_otaMutex != nullptr) xSemaphoreGive(_otaMutex);
    return copy;
}


static void randomHex128(char out[33])
{
    uint8_t random_bytes[16];
    for (size_t i = 0; i < sizeof(random_bytes); i += 4) {
        const uint32_t r = esp_random();
        memcpy(random_bytes + i, &r, std::min(sizeof(r), sizeof(random_bytes) - i));
    }
    bytesToHex(random_bytes, sizeof(random_bytes), out, 33);
}

bool GroundServicesTask::issueAuthNonce(char out[33], uint32_t ttl_ms)
{
    if (out == nullptr || _authMutex == nullptr) return false;
    randomHex128(out);

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

bool GroundServicesTask::isAuthNonceValid(const char *nonce)
{
    if (nonce == nullptr || _authMutex == nullptr) return false;
    const int64_t now_ms = esp_timer_get_time() / 1000;
    bool valid = false;

    xSemaphoreTake(_authMutex, portMAX_DELAY);
    for (size_t i = 0; i < AUTH_NONCE_COUNT; ++i) {
        if (!_authNonces[i].used && _authNonces[i].expires_at_ms > now_ms &&
            secureStringEqual(_authNonces[i].value, nonce)) {
            valid = true;
            break;
        }
    }
    xSemaphoreGive(_authMutex);
    return valid;
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

    if (!isAuthNonceValid(nonce)) return false;

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
    if (!secureStringEqual(expected, signature)) return false;

    // The compare above does not consume invalid attempts. This final locked
    // consume is the single-use gate if two valid requests race the same nonce.
    return consumeAuthNonce(nonce);
}

bool GroundServicesTask::rejectMutationDuringOta(httpd_req_t *req, GroundServicesTask *self)
{
    if (self == nullptr || !self->_otaExclusive.load(std::memory_order_acquire)) return false;
    discardBody(req);
    sendErrorJson(req, "409 Conflict", "OTA upload in progress; mutable Ground Services operations are paused");
    return true;
}

esp_err_t GroundServicesTask::rootGetHandler(httpd_req_t *req) { return sendAsset(req, ground_index_html_start, ground_index_html_end, "text/html"); }
esp_err_t GroundServicesTask::styleGetHandler(httpd_req_t *req) { return sendAsset(req, ground_style_css_start, ground_style_css_end, "text/css"); }
esp_err_t GroundServicesTask::appJsGetHandler(httpd_req_t *req) { return sendAsset(req, ground_app_js_start, ground_app_js_end, "application/javascript"); }
esp_err_t GroundServicesTask::faviconGetHandler(httpd_req_t *req) { httpd_resp_set_status(req, "204 No Content"); httpd_resp_set_hdr(req, "Connection", "close"); (void)httpd_resp_send(req, nullptr, 0); return ESP_OK; }

esp_err_t GroundServicesTask::redirectToRootHandler(httpd_req_t *req)
{
    static const char *dashboard_routes[] = {
        "/health", "/live-data", "/config", "/ota", "/tests", "/serial-monitor", "/files"
    };
    for (const char *route : dashboard_routes) {
        if (strcmp(req->uri, route) == 0) return rootGetHandler(req);
    }

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
    auto *self = fromReq(req);
    if (!self) return ESP_ERR_INVALID_ARG;

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
    const ClientCounts clients = getClientCounts(self->_server);
    const OtaStatus ota = self->getOtaStatus();
    wifi_sta_list_t stations = {};
    const uint16_t station_count = esp_wifi_ap_get_sta_list(&stations) == ESP_OK ? stations.num : 0;
    const uint32_t internal_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const size_t internal_free = heap_caps_get_free_size(internal_caps);
    const size_t internal_min = heap_caps_get_minimum_free_size(internal_caps);
    const size_t internal_largest = heap_caps_get_largest_free_block(internal_caps);
    const size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    const bool externalFlashReady = self->_rocketModel && self->_rocketModel->isExternalFlashInitialized();
    const bool sdReady = self->_rocketModel && self->_rocketModel->isSdInitialized();

    char *body = s_responseBuffer;
    snprintf(body, RESPONSE_BUFFER_SIZE,
        "{\"ok\":true,"
        "\"ground_services\":{"
        "\"http\":{\"clients\":%u,\"capacity\":%u,\"sessions_opened\":%lu,\"capacity_events\":%lu},"
        "\"websocket\":{\"clients\":%u,\"capacity\":%u,\"live_data\":%u,\"logs\":%u,"
        "\"send_failures\":%lu,\"slow_client_drops\":%lu,\"limit_rejects\":%lu},"
        "\"broadcast\":{\"queue_failures\":%lu,\"coalesced\":%lu,\"pending\":%s},"
        "\"task\":{\"stack_high_water_bytes\":%lu},\"ota_state\":\"%s\",\"softap_stations\":%u},"
        "\"memory\":{"
        "\"internal\":{\"free_bytes\":%lu,\"minimum_free_bytes\":%lu,\"largest_free_block_bytes\":%lu},"
        "\"psram\":{\"present\":%s,\"expected\":false,\"status\":\"%s\",\"total_bytes\":%lu,"
        "\"free_bytes\":%lu,\"largest_free_block_bytes\":%lu}},"
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
        static_cast<unsigned>(clients.http), static_cast<unsigned>(MAX_HTTP_CLIENTS),
        static_cast<unsigned long>(s_httpSessionsOpened.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_httpCapacityEvents.load(std::memory_order_relaxed)),
        static_cast<unsigned>(clients.websocket), static_cast<unsigned>(MAX_WS_CLIENTS),
        static_cast<unsigned>(clients.live), static_cast<unsigned>(clients.logs),
        static_cast<unsigned long>(s_wsSendFailures.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_wsSlowClientDrops.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_wsLimitRejects.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_broadcastQueueFailures.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_broadcastCoalesced.load(std::memory_order_relaxed)),
        s_wsBroadcastQueued.load(std::memory_order_relaxed) ? "true" : "false",
        static_cast<unsigned long>(self->getStackHighWaterMark()), otaStateToString(ota.state),
        static_cast<unsigned>(station_count),
        static_cast<unsigned long>(internal_free), static_cast<unsigned long>(internal_min),
        static_cast<unsigned long>(internal_largest),
        psram_total > 0 ? "true" : "false",
        psram_total > 0 ? "ok" : "not_installed",
        static_cast<unsigned long>(psram_total),
        static_cast<unsigned long>(psram_free), static_cast<unsigned long>(psram_largest),
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
        case SensorReadStatus::NO_DATA: return "no_data";
        case SensorReadStatus::NOT_PRESENT: return "not_present";
        default: return "unknown";
    }
}

size_t GroundServicesTask::buildLiveDataJson(char *body, size_t body_size) const
{
    if (body == nullptr || body_size == 0) return 0;
    IMUData imu = {};
    AccelerometerSensorData acc = {};
    PressureSensorData baro = {};
    GPSData gps = {};
    SensorReadStatus imuStatus = _rocketModel ? _rocketModel->getBNO055Data(imu) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus accStatus = _rocketModel ? _rocketModel->getLIS3DHTRData(acc) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus baroStatus = _rocketModel ? _rocketModel->getMS561101BA03Data_1(baro) : SensorReadStatus::NOT_PRESENT;
    SensorReadStatus gpsStatus = _rocketModel ? _rocketModel->getGPSData(gps) : SensorReadStatus::NOT_PRESENT;

    int written = snprintf(body, body_size,
        "{\"ok\":true,\"fsm_state\":\"%s\",\"calibration\":{\"imu\":%s,\"barometer\":%s,\"barometer_samples\":%d},"
        "\"flight\":{\"height_m\":%.3f,\"vertical_speed_mps\":%.3f,\"is_rising\":%s},"
        "\"sensors\":{\"imu\":{\"status\":\"%s\","
        "\"calibration\":{\"system\":%u,\"gyro\":%u,\"accelerometer\":%u,\"magnetometer\":%u},"
        "\"orientation_deg\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
        "\"quaternion\":{\"w\":%.7f,\"x\":%.7f,\"y\":%.7f,\"z\":%.7f},"
        "\"angular_velocity_rad_s\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
        "\"linear_acceleration_m_s2\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
        "\"acceleration_m_s2\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
        "\"gravity_m_s2\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
        "\"magnetometer_ut\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
        "\"temperature_c\":%.2f,\"timestamp\":%lu},"
        "\"accelerometer\":{\"status\":\"%s\",\"x\":%.5f,\"y\":%.5f,\"z\":%.5f,\"timestamp\":%lu},"
        "\"barometer\":{\"status\":\"%s\",\"pressure\":%.3f,\"temperature_c\":%.3f,\"timestamp\":%lu},"
        "\"gps\":{\"status\":\"%s\",\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.2f,\"fix\":%s,\"fix_type\":%u,\"satellites\":%u,\"ground_speed_mps\":%.3f,\"hdop\":%.3f,\"timestamp\":%lu}}}",
        _fsm ? rocketStateToString(_fsm->getCurrentState()) : "unknown",
        _rocketModel && _rocketModel->isSensorSystemCalibrated() ? "true" : "false",
        _rocketModel && _rocketModel->isBarometerZeroed() ? "true" : "false",
        _rocketModel ? _rocketModel->getBarometerSampleCount() : 0,
        static_cast<double>(_rocketModel ? _rocketModel->getCurrentHeight() : 0.0f),
        static_cast<double>(_rocketModel ? _rocketModel->getHeightGainSpeed() : 0.0f),
        _rocketModel && _rocketModel->getIsRising() ? "true" : "false",
        sensorStatusToString(imuStatus),
        imu.calibration_sys, imu.calibration_gyro, imu.calibration_accel, imu.calibration_mag,
        static_cast<double>(imu.orientation_x), static_cast<double>(imu.orientation_y), static_cast<double>(imu.orientation_z),
        imu.quaternion_w, imu.quaternion_x, imu.quaternion_y, imu.quaternion_z,
        static_cast<double>(imu.angular_velocity_x), static_cast<double>(imu.angular_velocity_y), static_cast<double>(imu.angular_velocity_z),
        static_cast<double>(imu.linear_acceleration_x), static_cast<double>(imu.linear_acceleration_y), static_cast<double>(imu.linear_acceleration_z),
        static_cast<double>(imu.acceleration_x), static_cast<double>(imu.acceleration_y), static_cast<double>(imu.acceleration_z),
        static_cast<double>(imu.gravity_x), static_cast<double>(imu.gravity_y), static_cast<double>(imu.gravity_z),
        static_cast<double>(imu.magnetometer_x), static_cast<double>(imu.magnetometer_y), static_cast<double>(imu.magnetometer_z),
        static_cast<double>(imu.temperature), static_cast<unsigned long>(imu.timestamp),
        sensorStatusToString(accStatus),
        static_cast<double>(acc.acceleration_x), static_cast<double>(acc.acceleration_y), static_cast<double>(acc.acceleration_z),
        static_cast<unsigned long>(acc.timestamp),
        sensorStatusToString(baroStatus),
        static_cast<double>(baro.pressure), static_cast<double>(baro.temperature), static_cast<unsigned long>(baro.timestamp),
        sensorStatusToString(gpsStatus),
        static_cast<double>(gps.latitude), static_cast<double>(gps.longitude), static_cast<double>(gps.altitude),
        gps.fixType >= 2 ? "true" : "false", gps.fixType, gps.satellites,
        static_cast<double>(gps.ground_speed), static_cast<double>(gps.hdop), static_cast<unsigned long>(gps.timestamp));
    return written > 0 && static_cast<size_t>(written) < body_size ? static_cast<size_t>(written) : 0;
}

esp_err_t GroundServicesTask::liveDataGetHandler(httpd_req_t *req)
{
    auto *self = fromReq(req);
    if (self == nullptr || self->buildLiveDataJson(s_responseBuffer, RESPONSE_BUFFER_SIZE) == 0) {
        return sendErrorJson(req, "500 Internal Server Error", "live data unavailable");
    }
    return sendJson(req, "200 OK", s_responseBuffer);
}

static esp_err_t acceptReadOnlyWebSocket(httpd_req_t *req, WsStream stream)
{
    size_t count = MAX_HTTP_CLIENTS;
    int clients[MAX_HTTP_CLIENTS] = {};
    if (httpd_get_client_list(req->handle, &count, clients) != ESP_OK) return ESP_FAIL;

    size_t websocket_count = 0;
    for (size_t i = 0; i < count; ++i) {
        if (httpd_ws_get_fd_info(req->handle, clients[i]) == HTTPD_WS_CLIENT_WEBSOCKET) websocket_count++;
    }
    if (websocket_count >= MAX_WS_CLIENTS) {
        s_wsLimitRejects.fetch_add(1, std::memory_order_relaxed);
        LOG_WARNING(TAG, "Rejecting WebSocket: client limit reached (%u/%u)",
                    static_cast<unsigned>(websocket_count), static_cast<unsigned>(MAX_WS_CLIENTS));
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "WebSocket client limit reached");
        return ESP_FAIL;
    }

    WsSession *session = nullptr;
    for (auto &candidate : s_wsSessions) {
        if (!candidate.active) {
            session = &candidate;
            break;
        }
    }
    if (session == nullptr) {
        s_wsLimitRejects.fetch_add(1, std::memory_order_relaxed);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "WebSocket sessions are still closing");
        return ESP_FAIL;
    }

    const int socket = httpd_req_to_sockfd(req);
    timeval timeout = {};
    timeout.tv_usec = WS_SEND_TIMEOUT_US;
    if (setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) != 0) {
        LOG_ERROR(TAG, "Failed to set WebSocket send timeout: errno=%d", errno);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "Failed to configure WebSocket");
        return ESP_FAIL;
    }

    session->stream = stream;
    session->socket = socket;
    session->last_log_seq = stream == WsStream::LOGS ? queryGetUint32(req, "since", 0) : 0;
    session->active = true;
    req->sess_ctx = session;
    req->free_ctx = releaseWsSession;
    return ESP_OK;
}

esp_err_t GroundServicesTask::liveDataWsPreHandshake(httpd_req_t *req)
{
    return acceptReadOnlyWebSocket(req, WsStream::LIVE_DATA);
}

esp_err_t GroundServicesTask::logsWsPreHandshake(httpd_req_t *req)
{
    return acceptReadOnlyWebSocket(req, WsStream::LOGS);
}

esp_err_t GroundServicesTask::readOnlyWsHandler(httpd_req_t *req)
{
    // This handler is used to notify both a connection being opened
    // and a frame being received on the WebSocket, we know
    // which one by the method. If we call recv_frame on the "connection opened"
    // request it will block the task until timeout, so being it an empty frame
    // return OK.
    if (req->method == HTTP_GET) return ESP_OK;

    httpd_ws_frame_t frame = {};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    shutdown(httpd_req_to_sockfd(req), SHUT_RDWR);
    return err;
}

void GroundServicesTask::broadcastLiveData()
{
    if (_server == nullptr) return;

    size_t count = MAX_HTTP_CLIENTS;
    int clients[MAX_HTTP_CLIENTS] = {};
    if (httpd_get_client_list(_server, &count, clients) != ESP_OK) return;

    int websocket_clients[MAX_WS_CLIENTS] = {};
    size_t websocket_count = 0;
    for (size_t i = 0; i < count && websocket_count < MAX_WS_CLIENTS; ++i) {
        auto *session = static_cast<WsSession *>(httpd_sess_get_ctx(_server, clients[i]));
        if (httpd_ws_get_fd_info(_server, clients[i]) == HTTPD_WS_CLIENT_WEBSOCKET &&
            session != nullptr && session->active && session->stream == WsStream::LIVE_DATA) {
            websocket_clients[websocket_count++] = clients[i];
        }
    }
    if (websocket_count == 0) return;

    size_t length = buildLiveDataJson(s_responseBuffer, RESPONSE_BUFFER_SIZE);
    if (length == 0) return;

    for (size_t i = 0; i < websocket_count; ++i) {
        sendWsText(_server, websocket_clients[i], s_responseBuffer, length);
    }
}

void GroundServicesTask::broadcastLogs()
{
    if (_server == nullptr) return;

    size_t count = MAX_HTTP_CLIENTS;
    int clients[MAX_HTTP_CLIENTS] = {};
    if (httpd_get_client_list(_server, &count, clients) != ESP_OK) return;

    for (size_t offset = 0; offset < count; ++offset) {
        const size_t i = (s_nextLogClient + offset) % count;
        if (httpd_ws_get_fd_info(_server, clients[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        auto *session = static_cast<WsSession *>(httpd_sess_get_ctx(_server, clients[i]));
        if (session == nullptr || !session->active || session->stream != WsStream::LOGS) continue;

        const uint32_t previous_seq = session->last_log_seq;
        uint32_t latest_seq = previous_seq;
        const size_t log_len = SerialLogger::getRecentLogsSince(
            previous_seq,
            s_responseBuffer + LOG_WS_PREFIX_SPACE,
            LOG_RESPONSE_BUFFER_SIZE - LOG_WS_PREFIX_SPACE,
            &latest_seq);
        if (log_len == 0) continue;

        const int prefix_len = snprintf(s_responseBuffer, LOG_WS_PREFIX_SPACE, "%lu\n", static_cast<unsigned long>(latest_seq));
        if (prefix_len <= 0 || static_cast<size_t>(prefix_len) >= LOG_WS_PREFIX_SPACE) continue;
        memmove(s_responseBuffer + prefix_len, s_responseBuffer + LOG_WS_PREFIX_SPACE, log_len);
        if (sendWsText(_server, clients[i], s_responseBuffer, static_cast<size_t>(prefix_len) + log_len)) {
            session->last_log_seq = latest_seq;
        }
        s_nextLogClient = (i + 1) % count;
        return;
    }
}

esp_err_t GroundServicesTask::logsGetHandler(httpd_req_t *req)
{
    const uint32_t since_seq = queryGetUint32(req, "since", 0);
    uint32_t latest_seq = since_seq;
    SerialLogger::getRecentLogsSince(since_seq, s_responseBuffer, LOG_RESPONSE_BUFFER_SIZE, &latest_seq);
    char latest_header[16];
    snprintf(latest_header, sizeof(latest_header), "%lu", static_cast<unsigned long>(latest_seq));
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "X-Log-Latest-Seq", latest_header);
    (void)httpd_resp_sendstr(req, s_responseBuffer);
    return ESP_OK;
}

esp_err_t GroundServicesTask::runtimeConfigGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    RuntimeConfig cfg = runtime_config_defaults();
    if (runtime_config_get(&cfg) != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_get failed");
    if (runtime_config_to_json(&cfg, s_responseBuffer, RESPONSE_BUFFER_SIZE) != ESP_OK) return sendErrorJson(req, "500 Internal Server Error", "runtime_config_to_json failed");
    return sendJson(req, "200 OK", s_responseBuffer);
}

esp_err_t GroundServicesTask::runtimeConfigPutHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    auto *self = fromReq(req);
    if (rejectMutationDuringOta(req, self)) return ESP_OK;
    if (runtime_config_is_locked()) {
        discardBody(req);
        return sendErrorJson(req, "423 Locked", "configuration is locked for flight");
    }
    esp_err_t err = readBody(req, s_responseBuffer, RESPONSE_BUFFER_SIZE);
    if (err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid config body", err);
    RuntimeConfig updated;
    err = runtime_config_update_from_json(s_responseBuffer, &updated);
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
    if (rejectMutationDuringOta(req, fromReq(req))) return ESP_OK;
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
    if (rejectMutationDuringOta(req, self)) return ESP_OK;
    if (!headerEquals(req, "X-Confirm", "UNLOCK_AFTER_RECOVERY")) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: UNLOCK_AFTER_RECOVERY header");
    }

    esp_err_t body_err = readBody(req, s_responseBuffer, RESPONSE_BUFFER_SIZE);
    if (body_err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid unlock body", body_err);
    if (strstr(s_responseBuffer, "UNLOCK_AFTER_RECOVERY") == nullptr) {
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
            return "Runtime recovery mode: one parachute";
        case RecoveryMode::TwoParachuteMode:
            return "Runtime recovery mode: two parachutes";
        default:
            return "Runtime recovery mode: unknown";
    }
}

esp_err_t GroundServicesTask::buildPrelaunchChecklistJson(GroundServicesTask *self, char *body, size_t body_size, bool *ok_out)
{
    if (self == nullptr || body == nullptr || ok_out == nullptr) return ESP_ERR_INVALID_ARG;

    RuntimeConfig cfg = runtime_config_defaults();
    char reason[128] = {};
    const bool config_loaded = runtime_config_get(&cfg) == ESP_OK;
    const bool config_valid = config_loaded && runtime_config_validate(&cfg, reason, sizeof(reason)) == ESP_OK;
    const bool config_unlocked = config_loaded && !cfg.config_locked;
    const bool barometer_ready = self->_rocketModel && self->_rocketModel->isBarometerZeroed();
    const bool imu_ready = self->_rocketModel && self->_rocketModel->isSensorSystemCalibrated();

    bool all_ok = true;
    size_t offset = 0;
    int written = snprintf(body, body_size, "{\"items\":[");
    if (written < 0 || static_cast<size_t>(written) >= body_size) return ESP_ERR_NO_MEM;
    offset = static_cast<size_t>(written);

    if (!appendChecklistItem(body, body_size, &offset, &all_ok, config_valid, "runtime_config_valid", "Runtime config valid", "error", config_valid ? "Runtime config validation passed" : reason)) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, config_unlocked, "config_unlocked", "Config not already locked", "error", config_unlocked ? "Configuration can be locked for flight" : "Configuration is already locked")) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, barometer_ready, "barometer_baseline", "Barometer baseline available", "error", barometer_ready ? "Barometer baseline is available" : "Barometer baseline is not ready")) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, imu_ready, "imu_calibrated", "IMU calibrated", "error", imu_ready ? "IMU calibration is ready" : "IMU calibration is not ready")) return ESP_ERR_NO_MEM;
    if (!appendChecklistItem(body, body_size, &offset, &all_ok, true, "recovery_mode", "Recovery mode", "info",
                             recoveryModeToChecklistMessage(static_cast<RecoveryMode>(cfg.recovery.mode)))) return ESP_ERR_NO_MEM;

    written = snprintf(body + offset, body_size - offset, "],\"ok\":%s}", all_ok ? "true" : "false");
    if (written < 0 || static_cast<size_t>(written) >= body_size - offset) return ESP_ERR_NO_MEM;
    *ok_out = all_ok;
    return ESP_OK;
}

esp_err_t GroundServicesTask::prelaunchChecklistGetHandler(httpd_req_t *req)
{
    bool checklist_ok = false;
    if (buildPrelaunchChecklistJson(fromReq(req), s_responseBuffer, RESPONSE_BUFFER_SIZE, &checklist_ok) != ESP_OK) {
        return sendErrorJson(req, "500 Internal Server Error", "prelaunch checklist response too large");
    }
    return sendJson(req, "200 OK", s_responseBuffer);
}


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

        esp_err_t err = httpd_resp_send_chunk(req, s_responseBuffer, out_len);
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
            const size_t room = STREAM_CHUNK_SIZE - out_len;
            if (room == 0) {
                esp_err_t err = flush();
                if (err != ESP_OK) return err;
                continue;
            }

            const size_t copy_len = std::min(room, data_len);
            memcpy(s_responseBuffer + out_len, data, copy_len);
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
    if (rejectMutationDuringOta(req, self)) return ESP_OK;
    if (!headerEquals(req, "X-Confirm", "READY_FOR_LAUNCH")) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: READY_FOR_LAUNCH header");
    }
    esp_err_t body_err = readBody(req, s_responseBuffer, RESPONSE_BUFFER_SIZE);
    if (body_err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid ready-for-launch body", body_err);
    if (self->_fsm == nullptr || self->_fsm->getCurrentState() != RocketState::GROUND_SERVICES) {
        return sendErrorJson(req, "409 Conflict", "ready-for-launch is only available in GROUND_SERVICES");
    }
    if (runtime_config_is_locked()) {
        return sendErrorJson(req, "409 Conflict", "configuration is already locked; use Unlock After Recovery first");
    }

    bool checklist_ok = false;
    if (buildPrelaunchChecklistJson(self, s_responseBuffer, RESPONSE_BUFFER_SIZE, &checklist_ok) != ESP_OK) {
        return sendErrorJson(req, "500 Internal Server Error", "prelaunch checklist response too large");
    }
    if (!checklist_ok) {
        return sendJson(req, "409 Conflict", s_responseBuffer);
    }

    if (!self->_fsm->sendEvent(FSMEvent::START_READY_FOR_LAUNCH)) {
        return sendErrorJson(req, "500 Internal Server Error", "failed to queue START_READY_FOR_LAUNCH");
    }
    return sendJson(req, "200 OK",
                    "{\"ok\":true,\"queued\":\"START_READY_FOR_LAUNCH\",\"config_lock_pending\":true}");
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
    if (rejectMutationDuringOta(req, self)) return ESP_OK;
    if (!self->_testRunner) {
        discardBody(req);
        return sendErrorJson(req, "503 Service Unavailable", "test runner unavailable");
    }
    if (!self->_fsm || self->_fsm->getCurrentState() != RocketState::GROUND_SERVICES) {
        discardBody(req);
        return sendErrorJson(req, "409 Conflict", "tests are only available in GROUND_SERVICES");
    }

    esp_err_t err = readBody(req, s_responseBuffer, RESPONSE_BUFFER_SIZE);
    if (err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid test start body", err);

    int id = 0;
    if (!bodyGetInt(s_responseBuffer, "id", &id)) {
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
    if (rejectMutationDuringOta(req, self)) return ESP_OK;
    if (!self->_testRunner) {
        discardBody(req);
        return sendErrorJson(req, "503 Service Unavailable", "test runner unavailable");
    }

    esp_err_t err = readBody(req, s_responseBuffer, RESPONSE_BUFFER_SIZE);
    if (err != ESP_OK) return sendErrorJson(req, "400 Bad Request", "invalid verdict body", err);

    char verdict_text[24] = {};
    if (!bodyGetString(s_responseBuffer, "verdict", verdict_text, sizeof(verdict_text))) {
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

esp_err_t GroundServicesTask::filesListGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    auto *self = fromReq(req);
    if (!self || !self->_rocketModel || !self->_rocketModel->isStorageInitialized()) {
        return sendErrorJson(req, "503 Service Unavailable", "storage unavailable");
    }

    auto *files = reinterpret_cast<StorageFileInfo *>(s_responseBuffer);
    constexpr size_t MAX_LISTED_FILES = 32;
    for (size_t i = 0; i < MAX_LISTED_FILES; ++i) new (&files[i]) StorageFileInfo{};
    const size_t count = self->_rocketModel->storageListFiles(files, MAX_LISTED_FILES);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    if (httpd_resp_send_chunk(req, "{\"ok\":true,\"files\":[", HTTPD_RESP_USE_STRLEN) != ESP_OK) return ESP_FAIL;

    bool first = true;
    char item[160];
    for (size_t i = 0; i < count; ++i) {
        const char *safeName = files[i].name;
        bool safe = safeName[0] != '\0' && strstr(safeName, "..") == nullptr;
        for (const unsigned char *p = reinterpret_cast<const unsigned char *>(safeName); safe && *p; ++p) {
            safe = std::isalnum(*p) || *p == '.' || *p == '_' || *p == '-';
        }
        if (!safe) continue;
        const int written = snprintf(item, sizeof(item), "%s{\"name\":\"%.63s\",\"size\":%lu}",
                                     first ? "" : ",", safeName,
                                     static_cast<unsigned long>(files[i].size));
        if (written < 0 || static_cast<size_t>(written) >= sizeof(item) ||
            httpd_resp_send_chunk(req, item, written) != ESP_OK) return ESP_FAIL;
        first = false;
    }
    if (httpd_resp_send_chunk(req, "]}", HTTPD_RESP_USE_STRLEN) != ESP_OK) return ESP_FAIL;
    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t GroundServicesTask::fileDownloadGetHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) return ESP_OK;
    auto *self = fromReq(req);
    char name[64] = {};
    if (!getSafeFileName(req, name, sizeof(name))) {
        return sendErrorJson(req, "400 Bad Request", "invalid file name");
    }
    if (!self || !self->_rocketModel) return sendErrorJson(req, "503 Service Unavailable", "storage unavailable");

    size_t bytesRead = 0;
    size_t fileSize = 0;
    if (!self->_rocketModel->storageReadFileChunk(name, 0, reinterpret_cast<uint8_t *>(s_responseBuffer),
                                                  STREAM_CHUNK_SIZE, bytesRead, fileSize)) {
        return sendErrorJson(req, "404 Not Found", "file not found");
    }
    const size_t downloadSize = fileSize;

    char disposition[96];
    snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", name);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");

    size_t offset = 0;
    while (offset < downloadSize) {
        if (offset != 0) {
            size_t currentFileSize = 0;
            const size_t remaining = downloadSize - offset;
            if (!self->_rocketModel->storageReadFileChunk(name, offset, reinterpret_cast<uint8_t *>(s_responseBuffer),
                                                           std::min(STREAM_CHUNK_SIZE, remaining), bytesRead, currentFileSize)) {
                return ESP_FAIL;
            }
        }
        if (bytesRead == 0 || httpd_resp_send_chunk(req, s_responseBuffer, bytesRead) != ESP_OK) return ESP_FAIL;
        offset += bytesRead;
        taskYIELD();
    }
    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t GroundServicesTask::fileDeleteHandler(httpd_req_t *req)
{
    if (!authOrSend(req)) { discardBody(req); return ESP_OK; }
    auto *self = fromReq(req);
    if (rejectMutationDuringOta(req, self)) return ESP_OK;
    if (!headerEquals(req, "X-Confirm", "DELETE_FILE")) {
        return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: DELETE_FILE header");
    }
    char name[64] = {};
    if (!getSafeFileName(req, name, sizeof(name))) {
        return sendErrorJson(req, "400 Bad Request", "invalid file name");
    }
    if (!self || !self->_rocketModel || !self->_rocketModel->storageDeleteFile(name)) {
        return sendErrorJson(req, "404 Not Found", "file not found");
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
        const size_t to_read = std::min(remaining, STREAM_CHUNK_SIZE);
        int received = httpd_req_recv(req, s_responseBuffer, static_cast<int>(to_read));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
            mbedtls_sha256_free(&sha);
            esp_ota_abort(ota_handle);
            self->setOtaStatus(OtaState::FAILED, written, req->content_len, ESP_FAIL, "http receive failed");
            return sendErrorJson(req, "500 Internal Server Error", "http receive failed");
        }

        mbedtls_sha256_update(&sha, reinterpret_cast<const uint8_t *>(s_responseBuffer), static_cast<size_t>(received));

        err = esp_ota_write(ota_handle, s_responseBuffer, static_cast<size_t>(received));
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
    if (rejectMutationDuringOta(req, self)) return ESP_OK;
    if (!headerEquals(req, "X-Confirm", "REBOOT_TO_NEW_FIRMWARE")) {
        discardBody(req);
        return sendErrorJson(req, "400 Bad Request", "missing X-Confirm: REBOOT_TO_NEW_FIRMWARE header");
    }
    esp_err_t err = readBody(req, s_responseBuffer, RESPONSE_BUFFER_SIZE);
    if (err != ESP_OK || strstr(s_responseBuffer, "REBOOT_TO_NEW_FIRMWARE") == nullptr) {
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
