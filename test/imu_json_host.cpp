// Run with: g++ -std=c++17 -Icomponents/common/data/src test/imu_json_host.cpp -o /tmp/imu_json_host && /tmp/imu_json_host
#include <AccelerometerSensorData.hpp>
#include <AltitudeData.hpp>
#include <GPSData.hpp>
#include <IMUData.hpp>
#include <PressureSensorData.hpp>
#include <cassert>
#include <cstring>

template <typename Packet>
void checkRequiredLength(const Packet& packet) {
    char full[1024];
    char small[8];
    const size_t required = packet.serializeJson(full, sizeof(full));
    assert(required == std::strlen(full));
    assert(packet.serializeJson(small, sizeof(small)) == required);
    assert(small[sizeof(small) - 1] == '\0');
    assert(packet.serializeJson(full, required) == required);
    assert(packet.serializeJson(full, required + 1) == required);
    assert(packet.serializeJson(nullptr, 0) == required);
}

int main() {
    IMUData imu;
    imu.setSensorName("BNO055");
    imu.timestamp = 42;
    imu.angular_velocity_x = -0.0625f;
    imu.validGroups = IMUData::VALIDITY_SENSOR_CALIBRATION | IMUData::VALIDITY_ANGULAR_VELOCITY |
                      IMUData::VALIDITY_ACCELERATION | IMUData::VALIDITY_MAGNETOMETER |
                      IMUData::VALIDITY_TEMPERATURE;

    char json[1024];
    const size_t amgLength = imu.serializeJson(json, sizeof(json));
    assert(amgLength == std::strlen(json));
    assert(std::strstr(json, "{\"src\":\"BNO055\",\"t\":42"));
    assert(std::strstr(json, "\"ax\":0"));
    assert(std::strstr(json, "\"avx\":-0.0625"));
    assert(std::strstr(json, "\"cgyro\":0"));
    assert(!std::strstr(json, "sensorData"));
    assert(!std::strstr(json, "\"csys\""));
    assert(!std::strstr(json, "\"ox\""));
    assert(!std::strstr(json, "\"lax\""));
    assert(!std::strstr(json, "\"gx\""));
    assert(!std::strstr(json, "\"qw\""));
    checkRequiredLength(imu);

    imu.validGroups = IMUData::VALIDITY_ATTITUDE | IMUData::VALIDITY_ACCELERATION;
    const size_t hilLength = imu.serializeJson(json, sizeof(json));
    assert(hilLength == std::strlen(json));
    assert(std::strstr(json, "\"qw\":1"));
    assert(!std::strstr(json, "\"avx\""));

    checkRequiredLength(imu);

    AltitudeData altitude;
    altitude.timestamp = 43;
    altitude.filtered_pressure_pa = 102032.0f;
    altitude.altitude_m = -0.164139f;
    altitude.ols_ready = true;
    assert(altitude.serializeJson(json, sizeof(json)) == std::strlen(json));
    assert(std::strstr(json, "\"pf_pa\":102032,\"alt_m\":-0.164139,\"vz_mps\":0"));
    assert(std::strstr(json, "\"ready\":true,\"rising\":true"));
    checkRequiredLength(altitude);

    PressureSensorData pressure;
    pressure.setSensorName("MS561101BA03_1");
    pressure.timestamp = 44;
    pressure.pressure = 102031.0f;
    pressure.temperature = 25.309999f;
    assert(pressure.serializeJson(json, sizeof(json)) == std::strlen(json));
    assert(std::strcmp(json, "{\"src\":\"MS561101BA03_1\",\"t\":44,\"p_pa\":102031,\"temp_c\":25.31}\n") == 0);
    checkRequiredLength(pressure);

    AccelerometerSensorData accel;
    accel.setSensorName("LIS3DHTR");
    accel.timestamp = 45;
    accel.acceleration_x = -0.0625f;
    assert(accel.serializeJson(json, sizeof(json)) == std::strlen(json));
    assert(std::strcmp(json, "{\"src\":\"LIS3DHTR\",\"t\":45,\"accx\":-0.0625,\"accy\":0,\"accz\":0}\n") == 0);
    checkRequiredLength(accel);

    GPSData gps;
    gps.setSensorName("GPS");
    gps.timestamp = 46;
    gps.satellites = 7;
    gps.fixType = 3;
    gps.latitude = 45.5f;
    assert(gps.serializeJson(json, sizeof(json)) == std::strlen(json));
    assert(std::strstr(json, "\"sats\":7,\"fix\":3,\"lat\":45.5,\"lon\":0,\"alt_m\":0,\"speed\":0,\"hdop\":0"));
    checkRequiredLength(gps);
}
