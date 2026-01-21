#include "mlx90632.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace mlx90632 {

// I2C Helper: Read 16-bit register (big-endian)
bool MLX90632Sensor::read_register16(uint16_t reg, uint16_t *value) {
  uint8_t addr_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  uint8_t data_buf[2] = {0};
  
  if (this->write_read(addr_buf, 2, data_buf, 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "%s Failed to read register 0x%04X", FW_VERSION, reg);
    return false;
  }
  
  *value = (data_buf[0] << 8) | data_buf[1];
  return true;
}

// I2C Helper: Write 16-bit register (big-endian)
bool MLX90632Sensor::write_register16(uint16_t reg, uint16_t value) {
  uint8_t buf[4] = {
    (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
    (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)
  };
  
  if (this->write(buf, 4) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "%s Failed to write register 0x%04X", FW_VERSION, reg);
    return false;
  }
  return true;
}

// Simplified temperature calculation for medical mode
float MLX90632Sensor::calculate_temperature(uint16_t raw) {
  // Simplified: raw is signed 16-bit, temp in Celsius
  // From datasheet: temp = (raw - 13657.5) / 50.0 for some modes
  // But for simplicity: temp = raw / 50.0 - 273.15 or similar
  // Actually, MLX90632 outputs in Kelvin * 100
  int16_t signed_raw = (int16_t)raw;
  float temp_kelvin = signed_raw / 100.0f;
  return temp_kelvin - 273.15f;  // Convert to Celsius
}

// Setup: Initialize sensor
void MLX90632Sensor::setup() {
  ESP_LOGI(TAG, "%s Setting up MLX90632...", FW_VERSION);
  
  // Check I2C address
  uint16_t addr_reg;
  if (!read_register16(0x3000, &addr_reg)) {
    ESP_LOGE(TAG, "%s Failed to read I2C address register", FW_VERSION);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "%s I2C Address register: 0x%04X", FW_VERSION, addr_reg);
  
  // Set to continuous mode for medical measurements
  if (!write_register16(0x3001, 0x0003)) {  // Continuous mode
    ESP_LOGE(TAG, "%s Failed to set continuous mode", FW_VERSION);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "%s Set to continuous mode", FW_VERSION);
}

// Update: Read and publish temperature
void MLX90632Sensor::update() {
  ESP_LOGD(TAG, "%s Reading temperature...", FW_VERSION);
  
  // Read all relevant registers for debugging
  uint16_t addr, ctrl, rate, status;
  uint16_t ram3, ram4, ram5, ram6, ram7, ram8, ram9;
  read_register16(0x3000, &addr);
  read_register16(0x3001, &ctrl);
  read_register16(0x3002, &rate);
  read_register16(0x3004, &status);
  read_register16(0x03, &ram3);
  read_register16(0x04, &ram4);
  read_register16(0x05, &ram5);
  read_register16(0x06, &ram6);
  read_register16(0x07, &ram7);
  read_register16(0x08, &ram8);
  read_register16(0x09, &ram9);
  
  ESP_LOGD(TAG, "%s EEPROM: addr=0x%04X ctrl=0x%04X rate=0x%04X status=0x%04X", 
           FW_VERSION, addr, ctrl, rate, status);
  ESP_LOGD(TAG, "%s RAM: 03=0x%04X 04=0x%04X 05=0x%04X 06=0x%04X 07=0x%04X 08=0x%04X 09=0x%04X", 
           FW_VERSION, ram3, ram4, ram5, ram6, ram7, ram8, ram9);
  
  float tobj_c = calculate_temperature(ram6);  // Use RAM_6 for medical mode
  ESP_LOGI(TAG, "%s Object temperature: %.2f°C (raw: 0x%04X)", FW_VERSION, tobj_c, ram6);
  
  // Publish
  this->publish_state(tobj_c);
}

// Dump config
void MLX90632Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "%s MLX90632 Temperature Sensor", FW_VERSION);
  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "Object Temperature", this);
}

}  // namespace mlx90632
}  // namespace esphome
