// /** */
// #include "bno055_sensor.h"
// #include "driver/i2c.h"
// #include <stdio.h>



// /**
//  * @brief I2C master initialization.
//  * 
//  * @param i2c_port 
//  * @return esp_err_t 
//  */
// static esp_err_t i2c_master_init(i2c_port_t i2c_port) {
//     i2c_config_t conf;
//     conf.mode = I2C_MODE_MASTER;
//     conf.sda_io_num = SDA_PIN;
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.scl_io_num = SCL_PIN;
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
//     return i2c_param_config(i2c_port, &conf) || i2c_driver_install(i2c_port, conf.mode, 
//             I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
// }

// /**
//  * @brief Write on I2C channel.
//  * 
//  * @param i2c_port 
//  * @param device_addr 
//  * @param reg_addr 
//  * @param data 
//  * @return esp_err_t 
//  */
// static esp_err_t i2c_write_byte(i2c_port_t i2c_port, uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg_addr, true);
//     i2c_master_write_byte(cmd, data, true);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// /**
//  * @brief Read from I2C channel.
//  * 
//  * @param i2c_port 
//  * @param device_addr 
//  * @param reg_addr 
//  * @param data 
//  * @param len 
//  * @return esp_err_t 
//  */
// static esp_err_t i2c_read_bytes(i2c_port_t i2c_port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg_addr, true);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
//     i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// /**
//  * @brief BNO055 initialization.
//  * 
//  * @param self 
//  * @return true 
//  * @return false 
//  */
// bool bno055_init(Sensor* self) {

//     BNO055Sensor* sensor = (BNO055Sensor*) self;

//     if (i2c_master_init(sensor->i2c_port) != ESP_OK) {
//         printf("Errore inizializzazione I2C\n");
//         return false;
//     }

//     // BNO055 NDOF mode (Euler data).
//     if (i2c_write_byte(sensor->i2c_port, sensor->i2c_address, 0x3D, 0x0C) != ESP_OK) {
//         printf("Errore nell'impostazione della modalità del BNO055\n");
//         return false;
//     }

//     printf("BNO055 inizializzato correttamente.\n");
//     return true;
// }

// /**
//  * @brief Read from BNO055.
//  * 
//  * @param self 
//  * @param data 
//  * @return true 
//  * @return false 
//  */
// bool bno055_read_data(Sensor* self, bno_sensor_data_t* data) {
//     BNO055Sensor* sensor = (BNO055Sensor*) self;

//     uint8_t buffer[6];
//     if (i2c_read_bytes(sensor->i2c_port, sensor->i2c_address, 0x1A, buffer, 6) != ESP_OK) {
//         printf("Errore nella lettura dei dati dal BNO055\n");
//         return false;
//     }

//     // Conversion in Euler's values (heading, roll, pitch).
//     data->euler_heading = ((buffer[1] << 8) | buffer[0]) / 16.0;
//     data->euler_roll = ((buffer[3] << 8) | buffer[2]) / 16.0;
//     data->euler_pitch = ((buffer[5] << 8) | buffer[4]) / 16.0;

//     // Temp reading (registro 0x34)
//     uint8_t temp;
//     if (i2c_read_bytes(sensor->i2c_port, sensor->i2c_address, 0x34, &temp, 1) == ESP_OK) {
//         data->temperature = temp;
//     }

//     printf("Dati letti: Heading=%.2f, Roll=%.2f, Pitch=%.2f, Temp=%d°C\n",
//            data->euler_heading, data->euler_roll, data->euler_pitch, data->temperature);

//     return true;
// }

// void bno055_sensor_create(BNO055Sensor* sensor) {
//     sensor->base.init = bno055_init;
//     sensor->base.read_data = bno055_read_data;
//     sensor->i2c_port = I2C_MASTER_NUM;
//     sensor->i2c_address = BNO055_I2C_ADDR;
// }
// */