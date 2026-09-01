#include "RocketModel.hpp"
#include "MirrorStorage.hpp"
#include <cstdlib>
#include <cstring>
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "pins.h"

namespace {
void reportMissingMutex(const char* operation)
{
    LOG_ERROR("RocketModel", "%s skipped: required mutex was not created", operation);
}
} // namespace

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
    _sd(sd),
    _flash(flash),
    _isRising(false),
    _heightGainSpeed(0.0f),
    _currentHeight(0.0f)
#if CONFIG_AURORA_HIL_SUPPORT
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

    if (!_storageMutex || !_imuMutex || !_baro1Mutex || !_baro2Mutex || !_gpsMutex || !_stateMutex) {
        LOG_ERROR("RocketModel", "Failed to create one or more model mutexes; affected operations will be skipped");
    }
}

RocketModel::~RocketModel() {
    if (_imuMutex) { vSemaphoreDelete(_imuMutex); _imuMutex = nullptr; }
    if (_baro1Mutex) { vSemaphoreDelete(_baro1Mutex); _baro1Mutex = nullptr; }
    if (_baro2Mutex) { vSemaphoreDelete(_baro2Mutex); _baro2Mutex = nullptr; }
    if (_gpsMutex) { vSemaphoreDelete(_gpsMutex); _gpsMutex = nullptr; }
    if (_stateMutex) { vSemaphoreDelete(_stateMutex); _stateMutex = nullptr; }
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

#if CONFIG_AURORA_HIL_SUPPORT
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
    if (!_stateMutex) { reportMissingMutex("setOpenMainCommand"); return false; }
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _cmd.setMain(true);
        xSemaphoreGive(_stateMutex);
    }
    return true;
}

bool RocketModel::setOpenDrogueCommand(){
    if (!_stateMutex) { reportMissingMutex("setOpenDrogueCommand"); return false; }
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _cmd.setDrogue(true);
        xSemaphoreGive(_stateMutex);
    }
    return true;
}

bool RocketModel::setAirbrakesCommand(float lvl){
    if (!_stateMutex) { reportMissingMutex("setAirbrakesCommand"); return false; }
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _cmd.setAirbrakes(lvl);
        xSemaphoreGive(_stateMutex);
    }
    return true;
}

Command RocketModel::getCommand() {
    return _cmd;
}

void RocketModel::resetCommand()
{
    _cmd.reset();
}

#if CONFIG_AURORA_HIL_SUPPORT
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
    if (!_bno) return false;
    if (!_imuMutex) { reportMissingMutex("updateBNO055"); return false; }
    bool result = _bno->updateData();

    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (result) {
            _bnoData = _bno->getData();    
        }
        _bnoDataValid = result;
        xSemaphoreGive(_imuMutex);
    }    
    return result;
}

bool RocketModel::updateLIS3DHTR() {
    if (!_lis3dh) return false;
    if (!_imuMutex) { reportMissingMutex("updateLIS3DHTR"); return false; }
    bool result = _lis3dh->updateData();

    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (result) {
            _lis3dhData = _lis3dh->getData();
        }
        _lis3dhDataValid = result;
        xSemaphoreGive(_imuMutex);
    }
    return result;
}

bool RocketModel::updateMS561101BA03_1() {
    if (!_ms56_1) return false;
    if (!_baro1Mutex) { reportMissingMutex("updateMS561101BA03_1"); return false; }
    bool result = _ms56_1->updateData();

    if (result && xSemaphoreTake(_baro1Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _ms561101ba03Data_1 = _ms56_1->getData();
        _ms561101ba03Data_1_Valid = true;
        xSemaphoreGive(_baro1Mutex);
    }
    return result;
}

bool RocketModel::updateMS561101BA03_2() {
    if (!_ms56_2) return false;
    if (!_baro2Mutex) { reportMissingMutex("updateMS561101BA03_2"); return false; }

    bool result = _ms56_2->updateData();

    if (result && xSemaphoreTake(_baro2Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _ms561101ba03Data_2 = _ms56_2->getData();
        _ms561101ba03Data_2_Valid = true;
        xSemaphoreGive(_baro2Mutex);
    }
    return result;
}

bool RocketModel::updateGPS() {
    if (!_gps) return false;
    if (!_gpsMutex) { reportMissingMutex("updateGPS"); return false; }
    bool result = _gps->updateData();

    if (xSemaphoreTake(_gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (result) {
            _gpsData = _gps->getData();
        }
        _gpsDataValid = result;
        xSemaphoreGive(_gpsMutex);
    }
    return result;
}

SensorReadStatus RocketModel::getBNO055Data(IMUData& data) {
#if !CONFIG_AURORA_HIL_SIMULATION
    if (!_bno) return SensorReadStatus::NOT_PRESENT;
#endif

    if (!_imuMutex) { reportMissingMutex("getBNO055Data"); return SensorReadStatus::MUTEX_TIMEOUT; }
    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!_bnoDataValid) {
            xSemaphoreGive(_imuMutex);
            return SensorReadStatus::SENSOR_ERROR;
        }
        data = _bnoData;
        xSemaphoreGive(_imuMutex);
        return SensorReadStatus::OK;
    }
    return SensorReadStatus::MUTEX_TIMEOUT;
}

SensorReadStatus RocketModel::getLIS3DHTRData(AccelerometerSensorData& data) {
#if !CONFIG_AURORA_HIL_SIMULATION
    if (!_lis3dh) return SensorReadStatus::NOT_PRESENT;
#endif

    if (!_imuMutex) { reportMissingMutex("getLIS3DHTRData"); return SensorReadStatus::MUTEX_TIMEOUT; }
    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!_lis3dhDataValid) {
            xSemaphoreGive(_imuMutex);
            return SensorReadStatus::SENSOR_ERROR;
        }
        data = _lis3dhData;
        xSemaphoreGive(_imuMutex);
        return SensorReadStatus::OK;
    }
    return SensorReadStatus::MUTEX_TIMEOUT;
}

SensorReadStatus RocketModel::getMS561101BA03Data_1(PressureSensorData& data) {
#if !CONFIG_AURORA_HIL_SIMULATION
    if (!_ms56_1) return SensorReadStatus::NOT_PRESENT;
#endif
    
    if (!_baro1Mutex) { reportMissingMutex("getMS561101BA03Data_1"); return SensorReadStatus::MUTEX_TIMEOUT; }
    if (xSemaphoreTake(_baro1Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!_ms561101ba03Data_1_Valid) {
            xSemaphoreGive(_baro1Mutex);
            return SensorReadStatus::SENSOR_ERROR;
        }
        data = _ms561101ba03Data_1;
        xSemaphoreGive(_baro1Mutex);
        return SensorReadStatus::OK;
    }
    return SensorReadStatus::MUTEX_TIMEOUT;
}

SensorReadStatus RocketModel::getMS561101BA03Data_2(PressureSensorData& data) {
#if !CONFIG_AURORA_HIL_SIMULATION
    if (!_ms56_2) return SensorReadStatus::NOT_PRESENT;
#endif
    
    if (!_baro2Mutex) { reportMissingMutex("getMS561101BA03Data_2"); return SensorReadStatus::MUTEX_TIMEOUT; }
    if (xSemaphoreTake(_baro2Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!_ms561101ba03Data_2_Valid) {
            xSemaphoreGive(_baro2Mutex);
            return SensorReadStatus::SENSOR_ERROR;
        }
        data = _ms561101ba03Data_2;
        xSemaphoreGive(_baro2Mutex);
        return SensorReadStatus::OK;
    }
    return SensorReadStatus::MUTEX_TIMEOUT;
}

SensorReadStatus RocketModel::getGPSData(GPSData& data) {
#if !CONFIG_AURORA_HIL_SIMULATION
    if (!_gps) return SensorReadStatus::NOT_PRESENT;
#endif
    
    if (!_gpsMutex) { reportMissingMutex("getGPSData"); return SensorReadStatus::MUTEX_TIMEOUT; }
    if (xSemaphoreTake(_gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!_gpsDataValid) {
            xSemaphoreGive(_gpsMutex);
            return SensorReadStatus::SENSOR_ERROR;
        }
        data = _gpsData;
        xSemaphoreGive(_gpsMutex);
        return SensorReadStatus::OK;
    }
    return SensorReadStatus::MUTEX_TIMEOUT;
}

bool RocketModel::setSimulatedBNO055Data(IMUData data) {
    if (!_imuMutex) { reportMissingMutex("setSimulatedBNO055Data"); return false; }
    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _bnoData = data;
        _bnoDataValid = true;
        xSemaphoreGive(_imuMutex);
        return true;
    }
    return false;
}

bool RocketModel::setSimulatedLIS3DHTRData(AccelerometerSensorData data) {
    if (!_imuMutex) { reportMissingMutex("setSimulatedLIS3DHTRData"); return false; }
    if (xSemaphoreTake(_imuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _lis3dhData = data;
        _lis3dhDataValid = true;
        xSemaphoreGive(_imuMutex);
        return true;
    }
    return false;
}

bool RocketModel::setSimulatedMS561101BA03Data_1(PressureSensorData data) {
    if (!_baro1Mutex) { reportMissingMutex("setSimulatedMS561101BA03Data_1"); return false; }
    if (xSemaphoreTake(_baro1Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _ms561101ba03Data_1 = data;
        _ms561101ba03Data_1_Valid = true;
        xSemaphoreGive(_baro1Mutex);
        return true;
    }
    return false;
}

bool RocketModel::setSimulatedMS561101BA03Data_2(PressureSensorData data) {
    if (!_baro2Mutex) { reportMissingMutex("setSimulatedMS561101BA03Data_2"); return false; }
    if (xSemaphoreTake(_baro2Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _ms561101ba03Data_2 = data;
        _ms561101ba03Data_2_Valid = true;
        xSemaphoreGive(_baro2Mutex);
        return true;
    }
    return false;
}

bool RocketModel::setSimulatedGPSData(GPSData data) {
    if (!_gpsMutex) { reportMissingMutex("setSimulatedGPSData"); return false; }
    if (xSemaphoreTake(_gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _gpsData = data;
        _gpsDataValid = true;
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

void RocketModel::setHeightGainSpeed(float heightGainSpeed) {
    _heightGainSpeed = heightGainSpeed;
}

float RocketModel::getCurrentHeight() {
    return _currentHeight;
}

void RocketModel::setCurrentHeight(float height) {
    _currentHeight = height;
}

bool RocketModel::isStorageInitialized(uint32_t timeoutMs) const {
    if (!_storageMutex) { reportMissingMutex("isStorageInitialized"); return false; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        bool initialized = _storage && _storage->isInitialized();
        xSemaphoreGive(_storageMutex);
        return initialized;
    }
    return false;
}

bool RocketModel::isExternalFlashInitialized() const {
    return _flash && _flash->isInitialized();
}

bool RocketModel::isSdInitialized() const {
    return _sd && _sd->isInitialized();
}

bool RocketModel::storageFileExists(const char* filename, uint32_t timeoutMs) {
    if (filename == nullptr) {
        return false;
    }

    if (!_storageMutex) { reportMissingMutex("storageFileExists"); return false; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
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

    if (!_storageMutex) { reportMissingMutex("storageResetReadCursor"); return false; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
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

    if (!_storageMutex) { reportMissingMutex("storageReadFile"); return ""; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        std::string content = (_storage && _storage->isInitialized()) ? _storage->readFile(filename) : "";
        xSemaphoreGive(_storageMutex);
        return content;
    }
    return "";
}

std::string RocketModel::storageReadLine(uint32_t timeoutMs) {
    if (!_storageMutex) { reportMissingMutex("storageReadLine"); return ""; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        std::string line = (_storage && _storage->isInitialized()) ? _storage->readLine() : "";
        xSemaphoreGive(_storageMutex);
        return line;
    }
    return "";
}

bool RocketModel::storageWriteFile(const char* filename, const uint8_t* data, size_t length, uint32_t timeoutMs) {
    if (filename == nullptr || data == nullptr || length == 0) return false;

    if (!_storageMutex) { reportMissingMutex("storageWriteFile"); return false; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
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

    if (!_storageMutex) { reportMissingMutex("storageAppendFile"); return false; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        const bool ready = _storage && _storage->isInitialized();
        const bool ok = ready && _storage->appendFile(filename, data, length);
        xSemaphoreGive(_storageMutex);
        return ok;
    }
    return false;
}

size_t RocketModel::storageListFiles(StorageFileInfo* files, size_t capacity, uint32_t timeoutMs) {
    if (files == nullptr || capacity == 0) return 0;
    if (!_storageMutex) { reportMissingMutex("storageListFiles"); return 0; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        const size_t count = (_storage && _storage->isInitialized()) ? _storage->listFiles(files, capacity) : 0;
        xSemaphoreGive(_storageMutex);
        return count;
    }
    return 0;
}

bool RocketModel::storageReadFileChunk(const char* filename, size_t offset, uint8_t* buffer, size_t capacity,
                                       size_t& bytesRead, size_t& fileSize, uint32_t timeoutMs) {
    if (filename == nullptr || buffer == nullptr || capacity == 0) return false;
    if (!_storageMutex) { reportMissingMutex("storageReadFileChunk"); return false; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        const bool ok = _storage && _storage->isInitialized() &&
                        _storage->readFileChunk(filename, offset, buffer, capacity, bytesRead, fileSize);
        xSemaphoreGive(_storageMutex);
        return ok;
    }
    return false;
}

bool RocketModel::storageDeleteFile(const char* filename, uint32_t timeoutMs) {
    if (filename == nullptr) return false;
    if (!_storageMutex) { reportMissingMutex("storageDeleteFile"); return false; }
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        const bool ok = _storage && _storage->isInitialized() && _storage->deleteFile(filename);
        xSemaphoreGive(_storageMutex);
        return ok;
    }
    return false;
}

std::shared_ptr<BNO055Sensor> RocketModel::getBNO055Sensor() {
    return _bno;
}

bool RocketModel::isSensorSystemCalibrated() {
    if (_bno) {
        return _bno->isFullyCalibrated(); 
    }
    return true; 
}

void RocketModel::addBarometerSample(float pressure, size_t requiredSamples) {
    if (_barometerZeroed) return;

    if (_barometerSamples.size() < requiredSamples) {
        _barometerSamples.push_back(pressure);
    }

    if (_barometerSamples.size() >= requiredSamples) {
        float sum = 0.0f;
        for (float p : _barometerSamples) {
            sum += p;
        }
        _launchpadBasePressure = sum / requiredSamples;
        _barometerZeroed = true;
        
        LOG_INFO("RocketModel", "Barometer zeroed. Base pressure set to: %.2f", _launchpadBasePressure);
    }
}

void RocketModel::addTemperatureSample(float temperature, size_t requiredSamples) {
    if (_temperatureZeroed) return;

    if (_temperatureSamples.size() < requiredSamples) {
        _temperatureSamples.push_back(temperature);
    }

    if (_temperatureSamples.size() >= requiredSamples) {
        float sum = 0.0f;
        for (float p : _temperatureSamples) {
            sum += p;
        }
        _launchpadBaseTemperature = sum / requiredSamples;
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
