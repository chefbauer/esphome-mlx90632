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

// Calculate temperatures using Melexis DSPv5 algorithm (simplified)
float MLX90632Sensor::calculate_object_temperature(uint16_t raw_obj, uint16_t raw_amb) {
  // MLX90632 DSPv5 algorithm based on Melexis library
  // Raw values are signed 16-bit in 0.01°C units
  
  int16_t obj = (int16_t)raw_obj;
  int16_t amb = (int16_t)raw_amb;
  
  // Convert to °C (raw values are in 0.01°C units)
  float T_obj_raw = obj / 100.0f;
  float T_amb_raw = amb / 100.0f;
  
  // For basic functionality, use simple emissivity correction
  // Medical mode: object temperature is close to ambient
  // Emissivity for human skin is ~0.97
  float emissivity = 0.97f;
  float T_obj = T_amb_raw + (T_obj_raw - T_amb_raw) / emissivity;
  
  return T_obj;
}

float MLX90632Sensor::calculate_ambient_temperature(uint16_t raw_amb) {
  int16_t amb = (int16_t)raw_amb;
  return amb / 100.0f;  // Raw is in 0.01°C units
}

// Setup: Initialize sensor
void MLX90632Sensor::setup() {
  ESP_LOGI(TAG, "%s Setting up MLX90632...", FW_VERSION);
  
  // Check I2C address
  uint16_t addr_reg;
  if (!read_register16(0x24D5, &addr_reg)) {
    ESP_LOGE(TAG, "%s Failed to read I2C address register", FW_VERSION);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "%s I2C Address register: 0x%04X", FW_VERSION, addr_reg);
  
  // Extended reset sequence for problematic sensors
  ESP_LOGI(TAG, "%s Performing extended reset sequence...", FW_VERSION);
  
  // 1. Addressed reset
  if (!write_register16(0x3005, 0x0006)) {
    ESP_LOGE(TAG, "%s Failed to send addressed reset", FW_VERSION);
    this->mark_failed();
    return;
  }
  delay(1000);  // Longer delay for problematic sensors
  
  // 2. Check if sensor responds
  uint16_t test_addr;
  if (!read_register16(0x24D5, &test_addr)) {
    ESP_LOGE(TAG, "%s Sensor not responding after reset", FW_VERSION);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "%s Sensor responding after reset: 0x%04X", FW_VERSION, test_addr);
  
  // 3. Set to HALT mode
  if (!write_register16(0x24D4, 0x0000)) {
    ESP_LOGE(TAG, "%s Failed to set HALT mode", FW_VERSION);
    this->mark_failed();
    return;
  }
  delay(100);
  
  // 4. Try continuous mode first - some sensors work better with it
  if (!write_register16(0x24D4, 0x0003)) {  // EE_CONTROL: Medical Continuous mode
    ESP_LOGE(TAG, "%s Failed to set continuous mode", FW_VERSION);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "%s Set to medical continuous mode (trying both step and continuous)", FW_VERSION);
  
  // 5. Wait for sensor to stabilize
  delay(2000);
  
  ESP_LOGI(TAG, "%s Setup complete - sensor should be ready", FW_VERSION);
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
  
  // Quick RAM scan - only check key registers that should change
  uint16_t ram_6, ram_9, ram_52, ram_54;
  read_register16(0x4005, &ram_6);   // Medical object
  read_register16(0x4008, &ram_9);   // Medical ambient  
  read_register16(0x4033, &ram_52);  // Extended object
  read_register16(0x4035, &ram_54);  // Extended ambient
  
  ESP_LOGD(TAG, "%s KEY RAM: MED_OBJ=0x%04X (%d) MED_AMB=0x%04X (%d) EXT_OBJ=0x%04X (%d) EXT_AMB=0x%04X (%d)", 
           FW_VERSION, ram_6, (int16_t)ram_6, ram_9, (int16_t)ram_9, ram_52, (int16_t)ram_52, ram_54, (int16_t)ram_54);
  
  // Full RAM dump for debugging
  ESP_LOGD(TAG, "%s FULL RAM DUMP:", FW_VERSION);
  for (uint16_t addr = 0x4000; addr <= 0x4040; addr += 2) {
    uint16_t value;
    if (read_register16(addr, &value) == ESP_OK) {
      ESP_LOGD(TAG, "  RAM_0x%04X: 0x%04X (%d)", addr, value, (int16_t)value);
    } else {
      ESP_LOGD(TAG, "  RAM_0x%04X: READ FAILED", addr);
    }
  }
  
  // For now, use the same registers as before
  uint16_t med6 = 0, med9 = 0;
  read_register16(0x4005, &med6);  // RAM_6
  read_register16(0x4008, &med9);  // RAM_9
  
  float tobj_c = calculate_object_temperature(med6, med9);  // Use medical mode RAM_6 (obj) and RAM_9 (amb)
  float tamb_c = calculate_ambient_temperature(med9);       // Use medical mode RAM_9 (amb)
  
  // Auto-reset if ambient temperature indicates sensor failure (like MLX90614)
  if (tamb_c < -250.0f) {
    ESP_LOGW(TAG, "%s Sensor failure detected (Ambient=%.2f°C < -250°C) - resetting...", FW_VERSION, tamb_c);
    
    // Perform addressed reset
    if (write_register16(0x3005, 0x0006)) {
      delay(1000);  // Wait for reset
      
      // Re-initialize continuous mode
      write_register16(0x24D4, 0x0000);  // HALT
      delay(100);
      write_register16(0x24D4, 0x0003);  // Continuous
      delay(500);
      
      ESP_LOGI(TAG, "%s Sensor reset complete", FW_VERSION);
    } else {
      ESP_LOGE(TAG, "%s Failed to reset sensor", FW_VERSION);
    }
    
    // Skip this measurement cycle
    return;
  }
  
  ESP_LOGI(TAG, "%s Temperatures: Object=%.2f°C (0x%04X), Ambient=%.2f°C (0x%04X)", 
           FW_VERSION, tobj_c, med6, tamb_c, med9);
  
  // In continuous mode: no need to re-trigger measurements
  
  // Publish object temperature
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
