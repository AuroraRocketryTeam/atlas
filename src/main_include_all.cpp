#include <Arduino.h>

//
//  LDF smoke test
//

// ---- Project include/ ----
// #include "ResponseStatusContainer.hpp"

// ---- lib/control ----
#include "FlightState.hpp"
#include "IStateMachine.hpp"
#include "Logger.hpp"
#include "RocketFSM.hpp"
#include "SharedData.hpp"

// states
#include "states/IStateAction.hpp"
#include "states/StateAction.hpp"
#include "states/TransitionManager.hpp"

// tasks
#include "tasks/BarometerTask.hpp"
#include "tasks/BaseTask.hpp"
#include "tasks/EkfTask.hpp"
#include "tasks/GpsTask.hpp"
#include "tasks/ITask.hpp"
#include "tasks/SDLoggingTask.hpp"
#include "tasks/SensorTask.hpp"
#include "tasks/SimulationTask.hpp"
#include "tasks/TaskConfig.hpp"
#include "tasks/TaskManager.hpp"
#include "tasks/TelemetryTask.hpp"

// ---- lib/CSVlogger ----
#include "CSVLogger.hpp"

// ---- lib/data ----
#include "AccelerometerSensorData.hpp"
#include "GPSData.hpp"
#include "ILoggable.hpp"
#include "IMUData.hpp"
#include "ISensor.hpp"
#include "LogMessage.hpp"
#include "PressureSensorData.hpp"
#include "SensorData.hpp"

// ---- lib/global ----
#include "config.h"
#include "pins.h"
#include "TelemetryFields.h"

// ---- lib/kalman ----
// #include "KalmanFilter.hpp"
#include "KalmanFilter1D.hpp"

// ---- lib/logger ----
#include "ILogger.hpp"
#include "LogData.hpp"

// ---- lib/model ----
#include "RocketModel.hpp"

// ---- lib/rocket_logger ----
#include "RocketLogger.hpp"

// ---- lib/SD ----

#include "SD-master.hpp"

// ---- lib/StatusManager ----
#include "BuzzerController.hpp"
#include "LEDController.hpp"
#include "StatusManager.hpp"

// ---- lib/sensors (wrappers) ----
#include "BME680Sensor.hpp"

#include "BNO055Sensor.hpp"
#include "BNO055SensorInterface.hpp"

#include "GPS.hpp"

#include "LIS3DHTRSensor.hpp"

#include "MPRLSSensor.hpp"

#include "MS561101BA03.hpp"

#include "Termoresistenze.hpp"

// ---- lib/telemetry ----
// core
#include "ITransmitter.hpp"
#include "ResponseStatusContainer.hpp" // (telemetry/core)

// protocol
#include "Packet.hpp"
#include "PacketManager.hpp"
#include "PacketSerializer.hpp"

// radio espnow
#include "EspNowTransmitter.hpp"

// radio lora
// #include "E220LoRaTransmitter.hpp"
#include "LoRaConfigurationDeserializer.hpp"
#include "SX1261LoRaTransmitter.hpp"

// ---- third_party ----

// TinyEKF
// #include "tinyekf.h"
// #include "tinyekf_custom.h"

// BNO055 SensorAPI (C)
extern "C" {
#include "bno055.h"
}

// Seeed LIS3DHTR vendor
#include "LIS3DHTR.h"

// RadioLib (LoRa drivers, etc.)
#include <RadioLib.h>

// SparkFun u-blox GNSS
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

static void print_banner() {
  Serial.println();
  Serial.println("=== main_include_all: LDF smoke test ===");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  print_banner();

  // Touch a few types so the compiler can't discard everything as unused.
  SensorData sd = SensorData("fake");
  (void)sd;

  Packet pkt;
  (void)pkt;

  Serial.println("OK");
}

void loop() {
  delay(1000);
}
