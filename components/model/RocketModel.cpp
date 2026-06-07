#include "RocketModel.hpp"
#include "MirrorStorage.hpp"
#include <cstdlib>
#include <cstring>
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "pins.h"

RocketModel::RocketModel(std::shared_ptr<BNO055Sensor> bno,
            std::shared_ptr<LIS3DHTRSensor> lis3dh,
            std::shared_ptr<MS561101BA03> ms56_1,
            std::shared_ptr<MS561101BA03> ms56_2,
        std::shared_ptr<GPS> gps,
        std::shared_ptr<SD> sd,
        std::shared_ptr<Flash> flash) :
    _bno(bno),
    _lis3dh(lis3dh),
    _ms56_1(ms56_1),
    _ms56_2(ms56_2),
    _gps(gps),
    _isRising(false),
    _heightGainSpeed(0.0f),
    _currentHeight(0.0f)
#if CONFIG_AURORA_HIL_SIMULATION
    , _reset_simulation(false)
#endif
    , _storageMutex(xSemaphoreCreateMutex())
{
    // Configure and Initialize ADC unit
    adc_oneshot_unit_init_cfg_t adc1_config = {};
    adc1_config.unit_id  = ADC_UNIT_1;
    adc1_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config, &_adc1_handle));

    // ADC channel Configuration and Initialization (GPIO1 = ADC_CHANNEL_0)
    adc_oneshot_chan_cfg_t channel_config = {};
    channel_config.atten    = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc1_handle, ADC_PIN, &channel_config));

    auto mirrorStorage = std::make_shared<MirrorStorage>();

    // Priority order: flash first, then SD.
    if (flash) {
        mirrorStorage->addStorage(flash);
    }
    if (sd) {
        mirrorStorage->addStorage(sd);
    }
    _storage = mirrorStorage;

    _cmd = Command();
    _imuMutex = xSemaphoreCreateMutex();
    _baro1Mutex = xSemaphoreCreateMutex();
    _baro2Mutex = xSemaphoreCreateMutex();
    _gpsMutex = xSemaphoreCreateMutex();
    _stateMutex = xSemaphoreCreateMutex();
}

RocketModel::~RocketModel() {
    if (_storageMutex) {
        vSemaphoreDelete(_storageMutex);
        _storageMutex = nullptr;
    }
}

void RocketModel::reset() {
    _cmd.reset();

    if (_isRising) {
        _isRising = false;
    }

    if (_heightGainSpeed) {
        _heightGainSpeed = 0.0f;
    }

    if (_currentHeight) {
        _currentHeight = 0.0f;
    }

#if CONFIG_AURORA_HIL_SIMULATION
    _reset_simulation = false;

    IMUData bnoData;
    AccelerometerSensorData lis3dhData;
    PressureSensorData ms561101ba03Data_1;
    PressureSensorData ms561101ba03Data_2;
    GPSData gpsData;
    setSimulatedBNO055Data(bnoData);
    setSimulatedLIS3DHTRData(lis3dhData);
    setSimulatedMS561101BA03Data_1(ms561101ba03Data_1);
    setSimulatedMS561101BA03Data_2(ms561101ba03Data_2);
    setSimulatedGPSData(gpsData);

    LOG_INFO("Main", "Set simulated data to nullptr");
#endif
}   


void RocketModel::readBattery() {
    esp_err_t res = adc_oneshot_read(_adc1_handle, ADC_PIN, &_batteryAdc);
    if (res == ESP_OK) {
        _batteryVoltage = ((_batteryAdc / 4095.0f) * 3.3f) * 2.0f;
        _batteryPercentage = (_batteryVoltage - 5.6f) / (7.2f - 5.6f) * 100.0f;
        std::string voltageStr = (String(_batteryPercentage, 1) + "%").c_str();
    } else {
        LOG_ERROR("ADC", "Failed to read battery voltage: %s", esp_err_to_name(res));
    }
    
    //auto voltageData = SensorData("Voltage");
    //voltageData.setData("ADC_Value", _batteryAdc);
    //voltageData.setData("Voltage", _batteryVoltage);
    //voltageData.setData("Percentage", _batteryPercentage);
    
    //logger->logSensorData(voltageData);
}

bool RocketModel::setOpenMainCommand(){
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _cmd.setMain(true);
        xSemaphoreGive(_stateMutex);
    }
    
    // bitmask |= FSM_DONE;
    // if bitmask == DONE:
    //     bitmask = 0;
    //     xTaskNotifyGive(); // unlock Simulation task to simulatorTask
    // release lock;
    return true;
}

bool RocketModel::setOpenDrogueCommand(){
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _cmd.setDrogue(true);
        xSemaphoreGive(_stateMutex);
    }
    // bitmask |= FSM_DONE;
    // if bitmask == DONE:
    //     bitmask = 0;
    //     xTaskNotifyGive(); // unlock Simulation task to simulatorTask
    // release lock;
    return true;
}

bool RocketModel::setAirbrakesCommand(float lvl){
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _cmd.setAirbrakes(lvl);
        xSemaphoreGive(_stateMutex);
    }
    // bitmask |= AIRBRAKE_DONE;
    // if bitmask == DONE:
    //     bitmask = 0;    
    //     xTaskNotifyGive(); // unlock Simulation task to simulatorTask
    // release lock;
    return true;
}

Command RocketModel::getCommand() {
    return _cmd;
}

void RocketModel::resetCommand()
{
    _cmd.reset();
}

#if CONFIG_AURORA_HIL_SIMULATION
void RocketModel::setResetSimulationFlag(bool value)
{
    _reset_simulation = value;
}

bool RocketModel::getResetSimulationFlag()
{
    return _reset_simulation;
}
#endif

bool RocketModel::updateBNO055() {
    // In simulation mode, the data is set directly by the HilSimulationTask, so we can just return true here, to avoid trying to read from the sensor
    if (!_bno) return true;
    bool result = _bno->updateData();

    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _bnoData = _bno->getData();
        xSemaphoreGive(_imuMutex);
    }    

    return result;
}

bool RocketModel::updateLIS3DHTR() {
    // In simulation mode, the data is set directly by the HilSimulationTask, so we can just return true here, to avoid trying to read from the sensor
    if (!_lis3dh) return true;
    bool result = _lis3dh->updateData();

    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _lis3dhData = _lis3dh->getData();
        xSemaphoreGive(_imuMutex);
    }

    return result;
}

bool RocketModel::updateMS561101BA03_1() {
    // In simulation mode, the data is set directly by the HilSimulationTask, so we can just return true here, to avoid trying to read from the sensor
    if (!_ms56_1) return true;
    bool result = _ms56_1->updateData();

    if (xSemaphoreTake(_baro1Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _ms561101ba03Data_1 = _ms56_1->getData();
        xSemaphoreGive(_baro1Mutex);
    }

    return result;
}

bool RocketModel::updateMS561101BA03_2() {
    // In simulation mode, the data is set directly by the HilSimulationTask, so we can just return true here, to avoid trying to read from the sensor
    if (!_ms56_2) return true;
    bool result = _ms56_2->updateData();

    if (xSemaphoreTake(_baro2Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _ms561101ba03Data_2 = _ms56_2->getData();
        xSemaphoreGive(_baro2Mutex);
    }

    return result;
}

bool RocketModel::updateGPS() {
    // In simulation mode, the data is set directly by the HilSimulationTask, so we can just return true here, to avoid trying to read from the sensor
    if (!_gps) return true;
    bool result = _gps->updateData();

    if (xSemaphoreTake(_gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _gpsData = _gps->getData();
        xSemaphoreGive(_gpsMutex);
    }

    return result;
}

bool RocketModel::getBNO055Data(IMUData& data) {
    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = _bnoData;
        xSemaphoreGive(_imuMutex);
        return true;
    }
    return false;
}

bool RocketModel::getLIS3DHTRData(AccelerometerSensorData& data) {
    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = _lis3dhData;
        xSemaphoreGive(_imuMutex);
        return true;
    }
    return false;
}

bool RocketModel::getMS561101BA03Data_1(PressureSensorData& data) {
    if (xSemaphoreTake(_baro1Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = _ms561101ba03Data_1;
        xSemaphoreGive(_baro1Mutex);
        return true;
    }
    return false;
}

bool RocketModel::getMS561101BA03Data_2(PressureSensorData& data) {
    if (xSemaphoreTake(_baro2Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = _ms561101ba03Data_2;
        xSemaphoreGive(_baro2Mutex);
        return true;
    }
    return false;
}

bool RocketModel::getGPSData(GPSData& data) {
    if (!_gps) {
        return false;
    }
    
    if (xSemaphoreTake(_gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = _gpsData;
        xSemaphoreGive(_gpsMutex);
        return true;
    }
    return false;
}

bool RocketModel::setSimulatedBNO055Data(IMUData data) {
    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _bnoData = data;
        xSemaphoreGive(_imuMutex);
        return true;
    }
    return false;
}

bool RocketModel::setSimulatedLIS3DHTRData(AccelerometerSensorData data) {
    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _lis3dhData = data;
        xSemaphoreGive(_imuMutex);
        return true;
    }

    return false;
}

bool RocketModel::setSimulatedMS561101BA03Data_1(PressureSensorData data) {
    if (xSemaphoreTake(_baro1Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _ms561101ba03Data_1 = data;
        xSemaphoreGive(_baro1Mutex);
        return true;
    }
    return false;
}

bool RocketModel::setSimulatedMS561101BA03Data_2(PressureSensorData data) {
    if (xSemaphoreTake(_baro2Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _ms561101ba03Data_2 = data;
        xSemaphoreGive(_baro2Mutex);
        return true;
    }
    return false;
}

bool RocketModel::setSimulatedGPSData(GPSData data) {
    if (xSemaphoreTake(_gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _gpsData = data;
        xSemaphoreGive(_gpsMutex);
        return true;
    }
    return false;
}

bool RocketModel::getIsRising() {
    return _isRising;
}

void RocketModel::setIsRising(bool isRising) {
    _isRising = isRising;
}

float RocketModel::getHeightGainSpeed() {
    return _heightGainSpeed;
}

float RocketModel::getCurrentHeight() {
    return _currentHeight;
}

void RocketModel::setCurrentHeight(float height) {
    _currentHeight = height;
}

bool RocketModel::isStorageInitialized(uint32_t timeoutMs) const {
    if (_storageMutex && xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        bool initialized = _storage && _storage->isInitialized();
        xSemaphoreGive(_storageMutex);
        return initialized;
    }
    return false;
}

bool RocketModel::storageFileExists(const char* filename, uint32_t timeoutMs) {
    if (filename == nullptr) {
        return false;
    }

    if (_storageMutex && xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        bool exists = _storage && _storage->isInitialized() && _storage->fileExists(filename);
        xSemaphoreGive(_storageMutex);
        return exists;
    }
    return false;
}

bool RocketModel::storageResetReadCursor(const char* filename, uint32_t timeoutMs) {
    if (filename == nullptr) {
        return false;
    }

    if (_storageMutex && xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        const bool ok = _storage && _storage->isInitialized() && _storage->openFile(filename);
        xSemaphoreGive(_storageMutex);
        return ok;
    }
    return false;
}

std::string RocketModel::storageReadFile(const char* filename, uint32_t timeoutMs) {
    if (filename == nullptr) {
        return "";
    }

    if (_storageMutex && xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        std::string content = (_storage && _storage->isInitialized()) ? _storage->readFile(filename) : "";
        xSemaphoreGive(_storageMutex);
        return content;
    }
    return "";
}

std::string RocketModel::storageReadLine(uint32_t timeoutMs) {
    if (_storageMutex && xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        std::string line = (_storage && _storage->isInitialized()) ? _storage->readLine() : "";
        xSemaphoreGive(_storageMutex);
        return line;
    }
    return "";
}

bool RocketModel::storageWriteFile(const char* filename, const uint8_t* data, size_t length, uint32_t timeoutMs) {
    if (filename == nullptr || data == nullptr || length == 0) return false;

    if (_storageMutex && xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        const bool ready = _storage && _storage->isInitialized();
        const bool ok = ready && _storage->openFile(filename) && 
                        _storage->writeFile(filename, data, length) && 
                        _storage->closeFile();
        xSemaphoreGive(_storageMutex);
        return ok;
    }
    return false;
}

bool RocketModel::storageAppendFile(const char* filename, const uint8_t* data, size_t length, uint32_t timeoutMs) {
    if (filename == nullptr || data == nullptr || length == 0) return false;

    if (_storageMutex && xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        const bool ready = _storage && _storage->isInitialized();
        const bool ok = ready && _storage->appendFile(filename, data, length);
        xSemaphoreGive(_storageMutex);
        return ok;
    }
    return false;
}

std::shared_ptr<BNO055Sensor> RocketModel::getBNO055Sensor() {
    return _bno;
}

bool RocketModel::isSensorSystemCalibrated() {
    // Assuming your BNO055Sensor class has a method to check the system calibration status.
    // Since NVS auto-loads on boot, this should return true almost immediately.
    if (_bno) {
        return _bno->isFullyCalibrated(); // You may need to ensure this method exists in BNO055Sensor.hpp
    }
    return true; // Fallback so the FSM doesn't lock up if the BNO is missing/disabled
}

void RocketModel::addBarometerSample(float pressure) {
    if (_barometerZeroed) return;

    if (_barometerSamples.size() < REQUIRED_BARO_SAMPLES) {
        _barometerSamples.push_back(pressure);
    }

    if (_barometerSamples.size() >= REQUIRED_BARO_SAMPLES) {
        float sum = 0.0f;
        for (float p : _barometerSamples) {
            sum += p;
        }
        _launchpadBasePressure = sum / REQUIRED_BARO_SAMPLES;
        _barometerZeroed = true;
        
        LOG_INFO("RocketModel", "Barometer zeroed. Base pressure set to: %.2f", _launchpadBasePressure);
    }
}

void RocketModel::addTemperatureSample(float temperature) {
    if (_temperatureZeroed) return;

    if (_temperatureSamples.size() < REQUIRED_TEMPERATURE_SAMPLES) {
        _temperatureSamples.push_back(temperature);
    }

    if (_temperatureSamples.size() >= REQUIRED_TEMPERATURE_SAMPLES) {
        float sum = 0.0f;
        for (float p : _temperatureSamples) {
            sum += p;
        }
        _launchpadBaseTemperature = sum / REQUIRED_TEMPERATURE_SAMPLES;
        _temperatureZeroed = true;
        
        LOG_INFO("RocketModel", "Temperature zeroed. Base temperature set to: %.2f", _launchpadBaseTemperature);
    }
}

int RocketModel::getBarometerSampleCount() {
    return _barometerSamples.size();
}

bool RocketModel::isBarometerZeroed() const {
    return _barometerZeroed;
}

bool RocketModel::isTemperatureZeroed() const {
    return _temperatureZeroed;
}

float RocketModel::getLaunchpadBasePressure() const {
    return _launchpadBasePressure;
}

float RocketModel::getLaunchpadBaseTemperature() const {
    return _launchpadBaseTemperature;
}
