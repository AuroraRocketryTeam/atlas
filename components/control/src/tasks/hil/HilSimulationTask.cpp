#include "HilSimulationTask.hpp"
#include "protocol.hpp"

#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_task_wdt.h"

static const char *TAG = "HilSimulationTask";

static constexpr int HIL_SERVER_PORT = CONFIG_AURORA_HIL_SERVER_PORT;

/* ===================== PACKETS ===================== */

typedef struct __attribute__((packed)) {
    uint32_t seq;
    float sim_time;
    uint32_t timestamp;

    float ax;
    float ay;
    float az;
    float p;
    float lat;
    float lon;
    float alt;
} sim_packet_t;


/* ===================== SOCKET HELPERS ===================== */

static bool recv_all(int _client_sock, uint8_t *buf, size_t len, const volatile bool& running)
{
    size_t received = 0;

    while (received < len) {

        if(!running) {
            LOG_WARNING(TAG, "early exit in recv_all: running=false");
            return false;
        }

        int ret = recv(_client_sock, buf + received, len - received, 0);

        if (ret == 0) {
            LOG_WARNING(TAG, "connection closed by peer");
            return false;
        }

        if (ret < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                esp_task_wdt_reset();
                vTaskDelay(1);
                continue;
            }

            LOG_ERROR(TAG, "recv error errno=%d", errno);
            return false;
        }

        received += ret;
    }

    return true;
}

static bool send_all(int _client_sock, const uint8_t *buf, size_t len, const volatile bool& running)
{
    size_t sent = 0;

    while (sent < len) {

        if(!running) {
            LOG_WARNING(TAG, "early exit in send_all: running=false");
            return false;
        }

        int ret = send(_client_sock, buf + sent, len - sent, 0);

        if (ret < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                esp_task_wdt_reset();
                vTaskDelay(1);
                continue;   // retry
            }

            LOG_ERROR(TAG, "send error errno=%d", errno);
            return false;
        }

        if (ret == 0) {
            LOG_WARNING(TAG, "send returned 0");
            return false;
        }

        sent += ret;
    }

    return true;
}

/* ===================== CTOR ===================== */

HilSimulationTask::HilSimulationTask(
    std::shared_ptr<RocketModel> rocketModel,
    SemaphoreHandle_t modelMutex,
    std::shared_ptr<RocketLogger> logger)
    : BaseTask("HilSimulationTask"),
      _rocketModel(rocketModel),
      _modelMutex(modelMutex),
      _logger(logger)
{
    // ctor
}


/* ===================== DTOR ===================== */

HilSimulationTask::~HilSimulationTask() {
    // dtor
}

void HilSimulationTask::onTaskStart() {
    // LOG_INFO(TAG, "onTaskStart");

    const int MAX_RETRY = 5;
    const TickType_t RETRY_DELAY = 200 / portTICK_PERIOD_MS;

    for (int attempt = 0; attempt < MAX_RETRY; ++attempt) {

        bool success = true;

        /* ===== socket ===== */
        _listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (_listen_sock < 0) {
            LOG_ERROR(TAG, "socket() failed: %s", strerror(errno));
            success = false;
        } else {
            // LOG_INFO(TAG, "socket() -> %d", _listen_sock);
        }

        /* ===== SO_REUSEADDR ===== */
        if (success) {
            int yes = 1;
            if (setsockopt(_listen_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
                LOG_ERROR(TAG, "setsockopt(SO_REUSEADDR) failed: %s", strerror(errno));
                success = false;
            }
        }

        /* ===== SO_RCVTIMEO ===== */
        if (success) {
            struct timeval timeout;
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            if (setsockopt(_listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
                LOG_ERROR(TAG, "setsockopt(SO_RCVTIMEO) failed: %s", strerror(errno));
                success = false;
            }
        }

        /* ===== bind ===== */
        if (success) {
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(HIL_SERVER_PORT);
            addr.sin_addr.s_addr = htonl(INADDR_ANY);

            if (bind(_listen_sock, (sockaddr *)&addr, sizeof(addr)) < 0) {
                LOG_ERROR(TAG, "bind failed: %s", strerror(errno));
                success = false;
            }
        }

        /* ===== listen ===== */
        if (success) {
            if (listen(_listen_sock, 1) < 0) {
                LOG_ERROR(TAG, "listen failed: %s", strerror(errno));
                success = false;
            }
        }

        /* ===== success path ===== */
        if (success) {
            LOG_INFO(TAG, "Listening on port %d", HIL_SERVER_PORT);
            return;
        }

        /* ===== cleanup ===== */
        if (_listen_sock >= 0) {
            close(_listen_sock);
            _listen_sock = -1;
        }

        LOG_WARNING(TAG, "Retrying socket init (%d/%d)...", attempt + 1, MAX_RETRY);
        vTaskDelay(RETRY_DELAY);
    }

    LOG_ERROR(TAG, "Failed to initialize TCP server after %d attempts", MAX_RETRY);
    _listen_sock = -1;
}

void HilSimulationTask::onTaskStop() {
    // LOG_INFO(TAG, "onTaskStop: client=%d listen=%d", _client_sock, _listen_sock);

    if (_client_sock >= 0) {
        if (shutdown(_client_sock, SHUT_RDWR) < 0) {
            LOG_ERROR(TAG, "Failed to shutdown _client_sock: %s", strerror(errno));
        }
        
        if (close(_client_sock) < 0) {
            LOG_ERROR(TAG, "Failed to close _client_sock: %s", strerror(errno));
        } else {
            // LOG_INFO(TAG, "onTaskStop: shutdown and close _client_sock");
        }

        _client_sock = -1;
    }

    if (_listen_sock >= 0) {
        if (close(_listen_sock) < 0) {
            LOG_ERROR(TAG, "Failed to close _listen_sock: %s", strerror(errno));
        } else {
            // LOG_INFO(TAG, "onTaskStop: shutdown and close _listen_sock");
        }

        _listen_sock = -1;
    }

}

void HilSimulationTask::reset() {
    _rocketModel->setResetSimulationFlag(true);
    // LOG_INFO(TAG, "reset: set flag in rocketmodel.");

}


/* ===================== MAIN TASK ===================== */

void HilSimulationTask::taskFunction() {

    if (_listen_sock < 0) {
        LOG_ERROR(TAG, "HilSimulationTask cannot start: listen socket is invalid");
        running = false;
        return;
    }

    proto_msg_t in_msg;
    proto_msg_t out_msg;

    while (running) {

        esp_task_wdt_reset();

        /* ================= ACCEPT ================= */

        sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);

        // LOG_INFO(TAG, "start accept");
        _client_sock = accept(_listen_sock, (sockaddr *)&client_addr, &len);

        if (_client_sock < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                esp_task_wdt_reset();
                vTaskDelay(1);
                // LOG_INFO(TAG, "no client yet");
                continue;   // no client yet
            }

            LOG_ERROR(TAG, "accept failed errno=%s", strerror(errno));
            continue;
        }

        struct timeval timeout;
        timeout.tv_sec = 1;   // 1 second
        timeout.tv_usec = 0;

        int ret = setsockopt(_client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        if (ret < 0) {
            LOG_ERROR(TAG, "_client_sock.setsockopt(SO_RCVTIMEO) failed: %s", strerror(errno));
            close(_client_sock);
            _client_sock = -1;
            return;
        }
        

        LOG_INFO(TAG, "Client connected");

        /* ================= CONNECTION LOOP ================= */

        while (running) {

            esp_task_wdt_reset();

            /* ===== RECEIVE HEADER ===== */

            uint8_t header[PROTO_HEADER_SIZE];

            // LOG_INFO(TAG, "start recv_all header");
            if (!recv_all(_client_sock, header, PROTO_HEADER_SIZE, running)) {
                LOG_WARNING(TAG, "closing: recv header failed");
                break;
            }

            uint32_t magic;
            uint16_t plen;
            uint16_t type;

            memcpy(&magic, header + 0, 4);
            memcpy(&plen,  header + 4, 2);
            memcpy(&type,  header + 6, 2);

            magic = ntohl(magic);
            plen  = ntohs(plen);
            type  = ntohs(type);

            if (magic != PROTO_MAGIC) {
                LOG_WARNING(TAG, "closing: invalid magic 0x%08" PRIx32, magic);
                break;
            }

            if (plen > PROTO_MAX_PAYLOAD_SIZE) {
                LOG_WARNING(TAG, "closing: payload too large len=%u max=%u",
                            plen, PROTO_MAX_PAYLOAD_SIZE);
                break;
            }

            uint8_t frame[PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD_SIZE];
            memcpy(frame, header, PROTO_HEADER_SIZE);

            /* ===== RECEIVE PAYLOAD ===== */

            if (plen > 0) {
                // LOG_INFO(TAG, "start recv_all payload");
                if (!recv_all(_client_sock, frame + PROTO_HEADER_SIZE, plen, running)) {
                    LOG_ERROR(TAG, "closing: recv payload failed len=%u: %s", plen, strerror(errno));
                    break;
                }
            }

            if (!protocol_decode_frame(frame, PROTO_HEADER_SIZE + plen, &in_msg)) {
                LOG_WARNING(TAG, "skip: protocol_decode_frame failed len=%u type=%u", plen, type);
                continue;
            }

            if (in_msg.type == MSG_TYPE_SIM_RESET) {
                if (in_msg.len != 0) {
                    LOG_WARNING(TAG, "skip: reset payload must be empty, got len=%u", in_msg.len);
                    continue;
                }

                LOG_INFO(TAG, "received RESET_SIM");
                this->reset();
                break;
            }

            if (in_msg.len != sizeof(sim_packet_t)) {
                LOG_WARNING(TAG, "skip: sim payload size mismatch got=%u expected=%u",
                            in_msg.len, (unsigned)sizeof(sim_packet_t));
                continue;
            }

            if (in_msg.type != MSG_TYPE_SIM_INPUT) {
                LOG_WARNING(TAG, "skip: in_msg.type != MSG_TYPE_SIM_INPUT");
                continue;
            }

            /* ================= UNPACK ================= */

            sim_packet_t pkt;
            memcpy(&pkt, in_msg.payload, sizeof(pkt));

            /* ================= FILL SENSOR DATA ================= */

            auto bnoData = std::make_shared<IMUData>("Sim_IMU");
            auto lis3dhData = std::make_shared<AccelerometerSensorData>("Sim_LIS3DH");
            auto ms1 = std::make_shared<PressureSensorData>("Sim_MS5611_1");
            auto ms2 = std::make_shared<PressureSensorData>("Sim_MS5611_2");
            auto gps = std::make_shared<GPSData>("Sim_GPS");

            bnoData->timestamp = pkt.timestamp;
            bnoData->acceleration_x = pkt.ax;
            bnoData->acceleration_y = pkt.ay;
            bnoData->acceleration_z = pkt.az;

            lis3dhData->timestamp = pkt.timestamp;
            lis3dhData->acceleration_x = pkt.ax;
            lis3dhData->acceleration_y = pkt.ay;
            lis3dhData->acceleration_z = pkt.az;

            ms1->timestamp = pkt.timestamp;
            ms1->pressure = pkt.p;
            ms2->timestamp = pkt.timestamp;
            ms2->pressure = pkt.p;

            gps->timestamp = pkt.timestamp;
            gps->latitude  = pkt.lat;
            gps->longitude = pkt.lon;
            gps->altitude  = pkt.alt;

            
            /* ================= UPDATE MODEL ================= */

            if (_rocketModel && xSemaphoreTake(_modelMutex, pdMS_TO_TICKS(200))) {
                _rocketModel->setSimulatedBNO055Data(bnoData);
                _rocketModel->setSimulatedLIS3DHTRData(lis3dhData);
                _rocketModel->setSimulatedMS561101BA03Data_1(ms1);
                _rocketModel->setSimulatedMS561101BA03Data_2(ms2);
                _rocketModel->setSimulatedGPSData(gps);
    
                if (_logger) {
                    _logger->logSensorData(bnoData);
                    _logger->logSensorData(lis3dhData);
                    _logger->logSensorData(ms1);
                    _logger->logSensorData(ms2);
                    _logger->logSensorData(gps);
                }

                xSemaphoreGive(_modelMutex);
            }

            Utils::setSimMillis(pkt.sim_time * 1000);

            vTaskDelay(1); // yield in order to let the other task to set the command
            if(!running) break;

            /* ================= READ COMMAND FROM MODEL ================= */
            esp_task_wdt_reset();
            Command cmd;

            if (_rocketModel && xSemaphoreTake(_modelMutex, pdMS_TO_TICKS(200))) {
                cmd = _rocketModel->getCommand();
                xSemaphoreGive(_modelMutex);
            }

            /* ================= PACK + SEND ================= */

            // Convert to wire format (adds sim_time here)
            FcCommandWire wire = cmd.serialize(pkt.sim_time);

            memset(&out_msg, 0, sizeof(out_msg));
            out_msg.type = MSG_TYPE_FC_COMMAND;
            out_msg.len  = sizeof(FcCommandWire);

            // Copy packed struct into payload
            memcpy(out_msg.payload, &wire, sizeof(wire));

            uint8_t out_buf[512];
            size_t out_len = 0;

            if (!protocol_encode_frame(&out_msg, out_buf, sizeof(out_buf), &out_len)) {
                LOG_WARNING(TAG, "closing: protocol_encode_frame failed payload_len=%zu", out_msg.len);
                break;
            }

            // LOG_INFO(TAG, "start send_all cmd");
            if (!send_all(_client_sock, out_buf, out_len, running)) {
                LOG_WARNING(TAG, "closing: send_all failed");
                break;
            }

            /* ================= LOG ================= */

            // LOG_INFO(TAG,
            //     "t=%.2f | acc=[%.2f %.2f %.2f] | alt=%.2f",
            //     pkt.sim_time,
            //     pkt.ax, pkt.ay, pkt.az,
            //     pkt.alt
            // );

            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms beacause Python runs at sampling_rate=20Hz (WARNING: mixing real and simulated time)
        }

        if (_client_sock >= 0) {
            close(_client_sock);
            _client_sock = -1;
            LOG_INFO(TAG, "Client disconnected");
        }

    }
}