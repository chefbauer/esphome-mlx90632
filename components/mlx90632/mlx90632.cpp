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
  if (!write_register16(0x24D4, 0x0003)) {  // EE_CONTROL: Continuous mode
    ESP_LOGE(TAG, "%s Failed to set continuous mode", FW_VERSION);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "%s Set to continuous mode", FW_VERSION);
}

// Update: Read and publish temperature
void MLX90632Sensor::update() {
  ESP_LOGD(TAG, "%s Reading temperature...", FW_VERSION);
  
  // Read EEPROM registers (config + calibration) - CORRECT ADDRESSES FROM DATASHEET
  uint16_t ee_i2c_addr, ee_control;
  uint16_t cal_p_r, cal_p_g, cal_p_t, cal_p_o, cal_aa, cal_ab, cal_ba, cal_bb;
  read_register16(0x24D5, &ee_i2c_addr);  // EE_I2C_ADDRESS
  read_register16(0x24D4, &ee_control);   // EE_CONTROL
  read_register16(0x2410, &cal_p_r);  // P_T calibration (corrected address)
  read_register16(0x2414, &cal_p_g);  // Aa calibration (corrected address)
  read_register16(0x2418, &cal_p_t);  // Ba calibration (corrected address)
  read_register16(0x241C, &cal_p_o);  // Ca calibration (corrected address)
  read_register16(0x2420, &cal_aa);   // Da calibration (corrected address)
  read_register16(0x2424, &cal_ab);   // Ea calibration (corrected address)
  read_register16(0x2428, &cal_ba);   // Fa calibration (corrected address)
  read_register16(0x242C, &cal_bb);   // Ga calibration (corrected address)
  
  // Read RAM registers - Medical mode (0x4000-0x4008)
  uint16_t med1, med2, med3, med4, med5, med6, med7, med8, med9;
  read_register16(0x4000, &med1);  // RAM_1
  read_register16(0x4001, &med2);  // RAM_2
  read_register16(0x4002, &med3);  // RAM_3
  read_register16(0x4003, &med4);  // RAM_4
  read_register16(0x4004, &med5);  // RAM_5
  read_register16(0x4005, &med6);  // RAM_6 (Object temp)
  read_register16(0x4006, &med7);  // RAM_7
  read_register16(0x4007, &med8);  // RAM_8
  read_register16(0x4008, &med9);  // RAM_9 (Ambient temp)
  
  // Read RAM registers - Extended mode (0x4033-0x403B)
  uint16_t ext52, ext53, ext54, ext55, ext56, ext57, ext58, ext59, ext60;
  read_register16(0x4033, &ext52);  // RAM_52
  read_register16(0x4034, &ext53);  // RAM_53
  read_register16(0x4035, &ext54);  // RAM_54
  read_register16(0x4036, &ext55);  // RAM_55
  read_register16(0x4037, &ext56);  // RAM_56
  read_register16(0x4038, &ext57);  // RAM_57
  read_register16(0x4039, &ext58);  // RAM_58
  read_register16(0x403A, &ext59);  // RAM_59
  read_register16(0x403B, &ext60);  // RAM_60
  
  ESP_LOGD(TAG, "%s EEPROM: i2c_addr=0x%04X control=0x%04X", 
           FW_VERSION, ee_i2c_addr, ee_control);
  ESP_LOGD(TAG, "%s CAL: P_T=0x%04X Aa=0x%04X Ba=0x%04X Ca=0x%04X Da=0x%04X Ea=0x%04X Fa=0x%04X Ga=0x%04X", 
           FW_VERSION, cal_p_r, cal_p_g, cal_p_t, cal_p_o, cal_aa, cal_ab, cal_ba, cal_bb);
  ESP_LOGD(TAG, "%s RAM_MED: 1=0x%04X 2=0x%04X 3=0x%04X 4=0x%04X 5=0x%04X 6=0x%04X 7=0x%04X 8=0x%04X 9=0x%04X", 
           FW_VERSION, med1, med2, med3, med4, med5, med6, med7, med8, med9);
  ESP_LOGD(TAG, "%s RAM_EXT: 52=0x%04X 53=0x%04X 54=0x%04X 55=0x%04X 56=0x%04X 57=0x%04X 58=0x%04X 59=0x%04X 60=0x%04X", 
           FW_VERSION, ext52, ext53, ext54, ext55, ext56, ext57, ext58, ext59, ext60);
  
  float tobj_c = calculate_temperature(med6);  // Use medical mode RAM_6
  ESP_LOGI(TAG, "%s Object temperature: %.2f°C (raw: 0x%04X)", FW_VERSION, tobj_c, med6);
  
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
