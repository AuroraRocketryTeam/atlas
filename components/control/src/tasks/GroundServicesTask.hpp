#pragma once

#include "BaseTask.hpp"
#include "RocketModel.hpp"
#include "RocketLogger.hpp"
#include "RuntimeConfig.hpp"
#include "esp_http_server.h"
#include <IBoardHardware.hpp>
#include <IStateMachine.hpp>
#include <memory>

class IGroundTestRunner;

class GroundServicesTask : public BaseTask
{
public:
    enum class OtaState {
        IDLE,
        WRITING,
        READY_TO_REBOOT,
        FAILED,
    };

    GroundServicesTask(std::shared_ptr<RocketModel> rocketModel,
                       std::shared_ptr<RocketLogger> logger,
                       IBoardHardware* board,
                       IStateMachine* fsm,
                       std::shared_ptr<IGroundTestRunner> testRunner = nullptr);
    ~GroundServicesTask() override;

    void onTaskStart() override;
    void onTaskStop() override;

    // Public only so the file-local HTTP authentication helper can call it.
    // Do not use outside GroundServicesTask.cpp.
    bool authenticateRequest(httpd_req_t *req);

private:
    struct OtaStatus {
        OtaState state = OtaState::IDLE;
        size_t bytes_written = 0;
        size_t total_size = 0;
        esp_err_t last_error = ESP_OK;
        char message[128] = "idle";
        char sha256[65] = "";
    };

    struct AuthNonce {
        char value[33] = "";
        int64_t expires_at_ms = 0;
        bool used = true;
    };

    static constexpr size_t AUTH_NONCE_COUNT = 8;

    void taskFunction() override;
    esp_err_t startServer();
    void stopServer();
    void registerHandlers();
    void setOtaStatus(OtaState state, size_t written, size_t total, esp_err_t err, const char *message, const char *sha256 = nullptr);
    OtaStatus getOtaStatus() const;

    bool issueAuthNonce(char out[33], uint32_t ttl_ms);
    bool consumeAuthNonce(const char *nonce);

    static GroundServicesTask* fromReq(httpd_req_t *req);
    static esp_err_t rootGetHandler(httpd_req_t *req);
    static esp_err_t styleGetHandler(httpd_req_t *req);
    static esp_err_t appJsGetHandler(httpd_req_t *req);
    static esp_err_t faviconGetHandler(httpd_req_t *req);
    static esp_err_t authNonceGetHandler(httpd_req_t *req);
    static esp_err_t statusGetHandler(httpd_req_t *req);
    static esp_err_t healthGetHandler(httpd_req_t *req);
    static esp_err_t liveDataGetHandler(httpd_req_t *req);
    static esp_err_t logsGetHandler(httpd_req_t *req);
    static esp_err_t runtimeConfigGetHandler(httpd_req_t *req);
    static esp_err_t runtimeConfigPutHandler(httpd_req_t *req);
    static esp_err_t runtimeConfigSchemaGetHandler(httpd_req_t *req);
    static esp_err_t runtimeConfigValidationGetHandler(httpd_req_t *req);
    static esp_err_t runtimeConfigResetHandler(httpd_req_t *req);
    static esp_err_t runtimeConfigUnlockHandler(httpd_req_t *req);
    static esp_err_t prelaunchChecklistGetHandler(httpd_req_t *req);
    static esp_err_t sdkconfigGetHandler(httpd_req_t *req);
    static esp_err_t readyForLaunchPostHandler(httpd_req_t *req);
    static esp_err_t testsListGetHandler(httpd_req_t *req);
    static esp_err_t testsStatusGetHandler(httpd_req_t *req);
    static esp_err_t testsStartPostHandler(httpd_req_t *req);
    static esp_err_t testsVerdictPostHandler(httpd_req_t *req);
    static esp_err_t otaStatusGetHandler(httpd_req_t *req);
    static esp_err_t otaUploadPostHandler(httpd_req_t *req);
    static esp_err_t otaRebootPostHandler(httpd_req_t *req);
    static esp_err_t buildPrelaunchChecklistJson(GroundServicesTask *self, char *body, size_t body_size, bool *ok_out);

    std::shared_ptr<RocketModel> _rocketModel;
    std::shared_ptr<RocketLogger> _logger;
    std::shared_ptr<IGroundTestRunner> _testRunner;
    IBoardHardware* _board;
    IStateMachine* _fsm;
    httpd_handle_t _server;
    SemaphoreHandle_t _otaMutex;
    SemaphoreHandle_t _authMutex;
    OtaStatus _otaStatus;
    AuthNonce _authNonces[AUTH_NONCE_COUNT];
};
