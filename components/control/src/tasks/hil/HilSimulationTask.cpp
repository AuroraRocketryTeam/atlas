#include "HilSimulationTask.hpp"
#include "protocol.hpp"
#include "SensorTask.hpp"

#include <inttypes.h>
#include <cmath>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_task_wdt.h"

static const char *TAG = "HilSimulationTask";

static constexpr int HIL_SERVER_PORT = 5000;
static constexpr uint32_t HIL_PACKET_LOG_PERIOD_MS = 5000;
static constexpr uint32_t HIL_PACKET_PERIOD_MS = 20;

// The simulator drives one packet per acquisition cycle at the same 50 Hz the
// real SensorTask loop runs at, so recording every SENSOR_LOG_INTERVAL_LOOPS-th
// packet reproduces the flight recorder cadence (25 Hz) in simulation.
static_assert(HIL_PACKET_PERIOD_MS == SENSOR_LOOP_PERIOD_MS,
              "HIL packet cadence must match the sensor loop period");
/* ===================== PACKETS ===================== */

typedef struct __attribute__((packed)) {
    uint32_t seq;           // sequence number
    float sim_time;         // simulation time
    uint32_t timestamp;     // rocketpy's host computer timestamp
    float ax;               // acceleration x
    float ay;               // acceleration y
    float az;               // acceleration z
    float qw;               // body-to-inertial attitude quaternion
    float qx;
    float qy;
    float qz;
    float p;                // pressure
    float t;                // temperature
    float lat;              // latitude
    float lon;              // longitude
    float alt;              // altitude
} sim_packet_t;

static_assert(sizeof(sim_packet_t) == 60, "HIL simulator packet layout mismatch");

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
    std::shared_ptr<RocketLogger> logger,
    IBoardHardware* board,
    IStateMachine* fsm)
    : BaseTask("HilSimulationTask"),
      _rocketModel(rocketModel),
      _logger(logger),
      _board(board),
      _fsm(fsm)
{
    // ctor
}


/* ===================== DTOR ===================== */

HilSimulationTask::~HilSimulationTask() {
    // dtor
}

void HilSimulationTask::onTaskStart() {
    // LOG_INFO(TAG, "onTaskStart");

    if (_board != nullptr) {
        if (!_board->startWifiSoftAp()) {
            LOG_ERROR(TAG, "Failed to acquire HIL SoftAP");
            running = false;
            return;
        }
        _softApAcquired = true;
        LOG_INFO(TAG, "HIL SoftAP ready at %s", _board->getWifiIpAddress());
    }

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
            timeout.tv_sec = 0;
            timeout.tv_usec = 500000;

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
    running = false;
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

    if (_softApAcquired) {
        if (_board != nullptr && !_board->stopWifi()) {
            LOG_ERROR(TAG, "Failed to release HIL SoftAP");
        }
        _softApAcquired = false;
    }

}

void HilSimulationTask::reset() {
    if (_rocketModel) {
        _rocketModel->setResetSimulationFlag(true);
    }

    // Force the current client out of lockstep. main_hil.cpp owns the actual
    // runtime reset and will stop/destroy/recreate this task cleanly.
    if (_client_sock >= 0) {
        shutdown(_client_sock, SHUT_RDWR);
        close(_client_sock);
        _client_sock = -1;
    }

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
            
            if (!running) {
                break;
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
        uint32_t sensorLogCounter = 0;

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

            IMUData bnoData;
            AccelerometerSensorData lis3dhData;
            PressureSensorData ms1;
            PressureSensorData ms2;
            GPSData gps;
            
            const uint32_t sim_time_ms = static_cast<uint32_t>(pkt.sim_time * 1000.0f);

            bnoData.timestamp = sim_time_ms;
            bnoData.acceleration_x = pkt.ax;
            bnoData.acceleration_y = pkt.ay;
            bnoData.acceleration_z = pkt.az;
            bnoData.quaternion_w = pkt.qw;
            bnoData.quaternion_x = pkt.qx;
            bnoData.quaternion_y = pkt.qy;
            bnoData.quaternion_z = pkt.qz;

            constexpr float RADIANS_TO_DEGREES = 180.0f / 3.14159265358979323846f;
            const float roll = std::atan2(
                2.0f * (pkt.qw * pkt.qx + pkt.qy * pkt.qz),
                1.0f - 2.0f * (pkt.qx * pkt.qx + pkt.qy * pkt.qy));
            const float pitchInput = 2.0f * (pkt.qw * pkt.qy - pkt.qz * pkt.qx);
            const float pitch = std::asin(std::fmax(-1.0f, std::fmin(1.0f, pitchInput)));
            float heading = std::atan2(
                2.0f * (pkt.qw * pkt.qz + pkt.qx * pkt.qy),
                1.0f - 2.0f * (pkt.qy * pkt.qy + pkt.qz * pkt.qz)) * RADIANS_TO_DEGREES;
            if (heading < 0.0f) heading += 360.0f;
            bnoData.orientation_x = heading;
            bnoData.orientation_y = roll * RADIANS_TO_DEGREES;
            bnoData.orientation_z = pitch * RADIANS_TO_DEGREES;
            bnoData.setSensorName("BNO055_SIM");

            lis3dhData.timestamp = sim_time_ms;
            lis3dhData.acceleration_x = pkt.ax;
            lis3dhData.acceleration_y = pkt.ay;
            lis3dhData.acceleration_z = pkt.az;
            lis3dhData.setSensorName("LIS3DH_SIM");

            ms1.timestamp = sim_time_ms;
            ms1.pressure = pkt.p;
            ms1.temperature = pkt.t - 273.15f;
            ms1.setSensorName("MS56_1_SIM");
            ms2.timestamp = sim_time_ms;
            ms2.pressure = pkt.p;
            ms2.temperature = pkt.t - 273.15f;
            ms2.setSensorName("MS56_2_SIM");

            gps.timestamp = sim_time_ms;
            gps.latitude  = pkt.lat;
            gps.longitude = pkt.lon;
            gps.altitude  = pkt.alt;
            gps.setSensorName("GPS_SIM");

            LOG_EVERY_MS(HIL_PACKET_LOG_PERIOD_MS, INFO, TAG,
                         "Received sim packet: time=%" PRIu32 " ax=%.2f ay=%.2f az=%.2f p=%.2f t=%.2f lat=%.6f lon=%.6f alt=%.2f",
                         sim_time_ms, pkt.ax, pkt.ay, pkt.az, pkt.p, pkt.t, pkt.lat, pkt.lon, pkt.alt);
            
            /* ================= UPDATE MODEL ================= */

            _rocketModel->setSimulatedBNO055Data(bnoData);
            _rocketModel->setSimulatedLIS3DHTRData(lis3dhData);
            _rocketModel->setSimulatedMS561101BA03Data_1(ms1);
            _rocketModel->setSimulatedMS561101BA03Data_2(ms2);
            _rocketModel->setSimulatedGPSData(gps);

            // Record every second acquisition cycle, exactly as SensorTask does.
            if (++sensorLogCounter >= SENSOR_LOG_INTERVAL_LOOPS) {
                sensorLogCounter = 0;
                if (_logger) {
                    _logger->logSensorData(bnoData);
                    _logger->logSensorData(lis3dhData);
                    _logger->logSensorData(ms1);
                    _logger->logSensorData(ms2);
                    _logger->logSensorData(gps);
                }
            }

            Utils::setSimMillis(sim_time_ms);

            // yield in order to let the other task to set the command
            // HIL_PACKET_PERIOD_MS -> 50Hz, because Python runs at default --sampling-rate=50Hz (WARNING: mixing real and simulated time)
            vTaskDelay(pdMS_TO_TICKS(HIL_PACKET_PERIOD_MS));
            if(!running) break;

            /* ================= READ COMMAND FROM MODEL ================= */
            esp_task_wdt_reset();
            Command cmd =_rocketModel->getCommand();

            /* ================= PACK + SEND ================= */

            // Convert to wire format (adds sim_time and fsm state)
            fc_command_packet_t wire = cmd.serialize(pkt.sim_time, _fsm->getCurrentState());
            
            memset(&out_msg, 0, sizeof(out_msg));
            out_msg.type = MSG_TYPE_FC_COMMAND;
            out_msg.len  = sizeof(fc_command_packet_t);

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
        }

        if (_client_sock >= 0) {
            close(_client_sock);
            _client_sock = -1;
            LOG_INFO(TAG, "Client disconnected");
        }

    }
}
