#include <Arduino.h>
#include <Wire.h>
#include "const/pins.h"
#include "sensors/BME680/bme680_sensor.h"
#include "sensors/BN0055/bno055_sensor.h"

#include "sensors/BME680/sensor.h"
#include "sensors/BN0055/sensor.h"

BME680Sensor bme680;
BNO055Sensor bno055;

void setup() {
    // Serial port initialization.
    Serial.begin(115200);
    
    // Sensors initialization.
    bme680_sensor_create(&bme680);
    bno055_sensor_create(&bno055);

    if (bme680.base.init((Sensor*)&bme680)) {
        Serial.println("BME680 inizializzato correttamente.");
    } else {
        Serial.println("Errore nell'inizializzazione del BME680.");
    }

    if (bno055.base.init((Sensor*)&bno055)) {
        Serial.println("BNO055 inizializzato correttamente.");
    } else {
        Serial.println("Errore nell'inizializzazione del BNO055.");
    }

}

void loop() {
    bno_sensor_data_t bno055_data;
    bme_sensor_data_t bme680_data;

    // Sensor BME680 read. 
    if (bme680.base.read_data((Sensor*)&bme680, &bme680_data)) {
        // Stampa dei dati letti dal sensore
        Serial.printf("Temperatura: %d.%02d°C\n", bme680_data.temperature / 100, bme680_data.temperature % 100);
        Serial.printf("Pressione: %d hPa\n", bme680_data.pressure / 100);
        Serial.printf("Umidità: %d.%02d%%\n", bme680_data.humidity / 1000, (bme680_data.humidity % 1000) / 10);
        Serial.printf("Resistenza gas: %d Ohm\n", bme680_data.gas_resistance);
    } else {
        Serial.println("Errore nella lettura dei dati dal sensore.");
    }

    // Leggiamo i dati dal sensore BNO055
    if (bno055.base.read_data((Sensor*)&bno055, &bno055_data)) {
        // Stampa dei dati letti dal sensore
        Serial.printf("Heading: %.2f°, Roll: %.2f°, Pitch: %.2f°\n", bno055_data.euler_heading, bno055_data.euler_roll, bno055_data.euler_pitch);
        Serial.printf("Temperatura: %d°C\n", bno055_data.temperature);
    } else {
        Serial.println("Errore nella lettura dei dati dal sensore.");
    }

    // New read.
    delay(2000);
}
