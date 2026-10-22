// Run with: g++ -std=c++17 -Icomponents/common/data/src test/imu_json_host.cpp -o /tmp/imu_json_host && /tmp/imu_json_host
#include <IMUData.hpp>
#include <cassert>
#include <cstring>

int main() {
    IMUData imu;
    imu.setSensorName("BNO055_AMG");
    imu.timestamp = 42;
    imu.validGroups = IMUData::VALIDITY_SENSOR_CALIBRATION | IMUData::VALIDITY_ANGULAR_VELOCITY |
                      IMUData::VALIDITY_ACCELERATION | IMUData::VALIDITY_MAGNETOMETER |
                      IMUData::VALIDITY_TEMPERATURE;

    char json[1024];
    const size_t amgLength = imu.serializeJson(json, sizeof(json));
    assert(amgLength == std::strlen(json));
    assert(std::strstr(json, "\"t\":42"));
    assert(std::strstr(json, "\"ax\":0.000000"));
    assert(std::strstr(json, "\"cgyro\":0"));
    assert(!std::strstr(json, "\"csys\""));
    assert(!std::strstr(json, "\"ox\""));
    assert(!std::strstr(json, "\"lax\""));
    assert(!std::strstr(json, "\"gx\""));
    assert(!std::strstr(json, "\"qw\""));

    imu.validGroups = IMUData::VALIDITY_ATTITUDE | IMUData::VALIDITY_ACCELERATION;
    const size_t hilLength = imu.serializeJson(json, sizeof(json));
    assert(std::strstr(json, "\"qw\":1.000000"));
    assert(!std::strstr(json, "\"avx\""));

    char small[8];
    assert(imu.serializeJson(small, sizeof(small)) == hilLength);
    assert(small[sizeof(small) - 1] == '\0');
}
