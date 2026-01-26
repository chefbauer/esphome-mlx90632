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

// Byte swap helper for register addresses
uint16_t MLX90632Sensor::swapBytes(uint16_t value) {
  return (value << 8) | (value >> 8);
}

// Helper function to read 32-bit values from consecutive registers
uint32_t MLX90632Sensor::read32BitRegister(uint16_t lsw_addr) {
  uint16_t lsw, msw;
  read_register16(swapBytes(lsw_addr), &lsw);
  read_register16(swapBytes(lsw_addr + 1), &msw);
  return ((uint32_t)msw << 16) | lsw;
}

// Initialize sensor (Adafruit-style)
bool MLX90632Sensor::begin() {
  // Check if device responds
  uint16_t product_code;
  if (!read_register16(swapBytes(MLX90632_REG_EE_PRODUCT_CODE), &product_code)) {
    ESP_LOGE(TAG, "%s Failed to read product code", FW_VERSION);
    return false;
  }

  if (product_code == 0xFFFF || product_code == 0x0000) {
    ESP_LOGE(TAG, "%s Invalid product code: 0x%04X", FW_VERSION, product_code);
    return false;
  }

  ESP_LOGI(TAG, "%s Product code: 0x%04X", FW_VERSION, product_code);

  // Load calibration constants
  if (!getCalibrations()) {
    ESP_LOGE(TAG, "%s Failed to load calibrations", FW_VERSION);
    return false;
  }

  return true;
}

// Reset device using addressed reset command
bool MLX90632Sensor::reset() {
  uint8_t reset_cmd[] = {0x30, 0x05, 0x00, 0x06};
  if (this->write(reset_cmd, 4) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "%s Reset command failed", FW_VERSION);
    return false;
  }

  // Wait for reset to complete
  delay(1);
  return true;
}

// Read all calibration constants from EEPROM
bool MLX90632Sensor::getCalibrations() {
  // Read 32-bit calibration constants
  uint32_t ee_p_r = read32BitRegister(MLX90632_REG_EE_P_R_LSW);
  uint32_t ee_p_g = read32BitRegister(MLX90632_REG_EE_P_G_LSW);
  uint32_t ee_p_t = read32BitRegister(MLX90632_REG_EE_P_T_LSW);
  uint32_t ee_p_o = read32BitRegister(MLX90632_REG_EE_P_O_LSW);
  uint32_t ee_aa = read32BitRegister(MLX90632_REG_EE_AA_LSW);
  uint32_t ee_ab = read32BitRegister(MLX90632_REG_EE_AB_LSW);
  uint32_t ee_ba = read32BitRegister(MLX90632_REG_EE_BA_LSW);
  uint32_t ee_bb = read32BitRegister(MLX90632_REG_EE_BB_LSW);
  uint32_t ee_ca = read32BitRegister(MLX90632_REG_EE_CA_LSW);
  uint32_t ee_cb = read32BitRegister(MLX90632_REG_EE_CB_LSW);
  uint32_t ee_da = read32BitRegister(MLX90632_REG_EE_DA_LSW);
  uint32_t ee_db = read32BitRegister(MLX90632_REG_EE_DB_LSW);
  uint32_t ee_ea = read32BitRegister(MLX90632_REG_EE_EA_LSW);
  uint32_t ee_eb = read32BitRegister(MLX90632_REG_EE_EB_LSW);
  uint32_t ee_fa = read32BitRegister(MLX90632_REG_EE_FA_LSW);
  uint32_t ee_fb = read32BitRegister(MLX90632_REG_EE_FB_LSW);
  uint32_t ee_ga = read32BitRegister(MLX90632_REG_EE_GA_LSW);

  // Convert to proper double values with scaling factors from datasheet
  P_R = (double)(int32_t)ee_p_r * pow(2, -8);   // 2^-8
  P_G = (double)(int32_t)ee_p_g * pow(2, -20);  // 2^-20
  P_T = (double)(int32_t)ee_p_t * pow(2, -44);  // 2^-44
  P_O = (double)(int32_t)ee_p_o * pow(2, -8);   // 2^-8
  Aa = (double)(int32_t)ee_aa * pow(2, -16);    // 2^-16
  Ab = (double)(int32_t)ee_ab * pow(2, -16);    // 2^-16
  Ba = (double)(int32_t)ee_ba * pow(2, -16);    // 2^-16
  Bb = (double)(int32_t)ee_bb * pow(2, -16);    // 2^-16
  Ca = (double)(int32_t)ee_ca * pow(2, -16);    // 2^-16
  Cb = (double)(int32_t)ee_cb * pow(2, -16);    // 2^-16
  Da = (double)(int32_t)ee_da * pow(2, -16);    // 2^-16
  Db = (double)(int32_t)ee_db * pow(2, -16);    // 2^-16
  Ea = (double)(int32_t)ee_ea * pow(2, -16);    // 2^-16
  Eb = (double)(int32_t)ee_eb * pow(2, -16);    // 2^-16
  Fa = (double)(int32_t)ee_fa * pow(2, -16);    // 2^-16
  Fb = (double)(int32_t)ee_fb * pow(2, -16);    // 2^-16
  Ga = (double)(int32_t)ee_ga * pow(2, -16);    // 2^-16

  // Read 16-bit constants
  uint16_t kb_reg, ha_reg, hb_reg;
  read_register16(swapBytes(MLX90632_REG_EE_KB), &kb_reg);
  read_register16(swapBytes(MLX90632_REG_EE_HA), &ha_reg);
  read_register16(swapBytes(MLX90632_REG_EE_HB), &hb_reg);

  Kb = (int16_t)kb_reg;
  Ka = (double)Kb * pow(2, -10);  // 2^-10
  Gb = (double)(int16_t)kb_reg * pow(2, -10);  // Gb = Kb * 2^-10
  Ha = (double)(int16_t)ha_reg * pow(2, -14);  // 2^-14
  Hb = (double)(int16_t)hb_reg * pow(2, -14);  // 2^-14

  ESP_LOGI(TAG, "%s Calibration constants loaded", FW_VERSION);
  return true;
}

// Set measurement mode
bool MLX90632Sensor::setMode(mlx90632_mode_t mode) {
  uint16_t control_reg;
  if (!read_register16(swapBytes(MLX90632_REG_CONTROL), &control_reg)) {
    return false;
  }

  // Clear mode bits (bits 1-2) and set new mode
  control_reg &= ~(0x03 << 1);
  control_reg |= ((uint16_t)mode << 1);

  return write_register16(swapBytes(MLX90632_REG_CONTROL), control_reg);
}

// Get measurement mode
mlx90632_mode_t MLX90632Sensor::getMode() {
  uint16_t control_reg;
  if (!read_register16(swapBytes(MLX90632_REG_CONTROL), &control_reg)) {
    return MLX90632_MODE_HALT;
  }

  return (mlx90632_mode_t)((control_reg >> 1) & 0x03);
}

// Set measurement select type
bool MLX90632Sensor::setMeasurementSelect(mlx90632_meas_select_t meas_select) {
  uint16_t meas1_reg;
  if (!read_register16(swapBytes(MLX90632_REG_EE_MEAS_1), &meas1_reg)) {
    return false;
  }

  // Clear measurement select bits and set new value
  meas1_reg &= ~(0x1F << 4);  // Clear bits 4-8
  meas1_reg |= ((uint16_t)meas_select << 4);

  return write_register16(swapBytes(MLX90632_REG_EE_MEAS_1), meas1_reg);
}

// Get measurement select type
mlx90632_meas_select_t MLX90632Sensor::getMeasurementSelect() {
  uint16_t meas1_reg;
  if (!read_register16(swapBytes(MLX90632_REG_EE_MEAS_1), &meas1_reg)) {
    return MLX90632_MEAS_MEDICAL;
  }

  return (mlx90632_meas_select_t)((meas1_reg >> 4) & 0x1F);
}

// Set refresh rate
bool MLX90632Sensor::setRefreshRate(mlx90632_refresh_rate_t refresh_rate) {
  uint16_t meas1_reg;
  if (!read_register16(swapBytes(MLX90632_REG_EE_MEAS_1), &meas1_reg)) {
    return false;
  }

  // Clear refresh rate bits (bits 0-2) and set new rate
  meas1_reg &= ~0x07;
  meas1_reg |= (uint16_t)refresh_rate;

  return write_register16(swapBytes(MLX90632_REG_EE_MEAS_1), meas1_reg);
}

// Get refresh rate
mlx90632_refresh_rate_t MLX90632Sensor::getRefreshRate() {
  uint16_t meas1_reg;
  if (!read_register16(swapBytes(MLX90632_REG_EE_MEAS_1), &meas1_reg)) {
    return MLX90632_REFRESH_2HZ;
  }

  return (mlx90632_refresh_rate_t)(meas1_reg & 0x07);
}

// Check if new data is available
bool MLX90632Sensor::isNewData() {
  uint16_t status_reg;
  if (!read_register16(swapBytes(MLX90632_REG_STATUS), &status_reg)) {
    return false;
  }

  return (status_reg & 0x01) != 0;
}

// Reset new data flag
bool MLX90632Sensor::resetNewData() {
  uint16_t status_reg;
  if (!read_register16(swapBytes(MLX90632_REG_STATUS), &status_reg)) {
    return false;
  }

  // Clear new data bit
  status_reg &= ~0x01;

  return write_register16(swapBytes(MLX90632_REG_STATUS), status_reg);
}

// Read cycle position
uint8_t MLX90632Sensor::readCyclePosition() {
  uint16_t status_reg;
  if (!read_register16(swapBytes(MLX90632_REG_STATUS), &status_reg)) {
    return 0;
  }

  return (status_reg >> 2) & 0x1F;  // Bits 2-6
}

// Get product code
uint16_t MLX90632Sensor::getProductCode() {
  uint16_t product_code;
  read_register16(swapBytes(MLX90632_REG_EE_PRODUCT_CODE), &product_code);
  return product_code;
}

// Get EEPROM version
uint16_t MLX90632Sensor::getEEPROMVersion() {
  uint16_t version;
  read_register16(swapBytes(MLX90632_REG_EE_VERSION), &version);
  return version;
}

// Calculate ambient temperature
double MLX90632Sensor::getAmbientTemperature() {
  mlx90632_meas_select_t meas_mode = getMeasurementSelect();

  int16_t ram_ambient, ram_ref;

  if (meas_mode == MLX90632_MEAS_EXTENDED_RANGE) {
    // Extended range mode: use RAM_54 and RAM_57
    uint16_t ram54, ram57;
    read_register16(swapBytes(MLX90632_REG_RAM_54), &ram54);
    read_register16(swapBytes(MLX90632_REG_RAM_57), &ram57);
    ram_ambient = (int16_t)ram54;
    ram_ref = (int16_t)ram57;
  } else {
    // Medical mode: use RAM_6 and RAM_9
    uint16_t ram6, ram9;
    read_register16(swapBytes(MLX90632_REG_RAM_6), &ram6);
    read_register16(swapBytes(MLX90632_REG_RAM_9), &ram9);
    ram_ambient = (int16_t)ram6;
    ram_ref = (int16_t)ram9;
  }

  // Pre-calculations for ambient temperature
  double VRTA = (double)ram_ref + Gb * ((double)ram_ambient / 12.0);
  double AMB = ((double)ram_ambient / 12.0) / VRTA * pow(2, 19);

  // Calculate ambient temperature: P_O + (AMB - P_R)/P_G + P_T * (AMB - P_R)^2
  double amb_diff = AMB - P_R;
  double ambient_temp = P_O + (amb_diff / P_G) + P_T * (amb_diff * amb_diff);

  return ambient_temp;
}

// Calculate object temperature (DSPv5 algorithm)
double MLX90632Sensor::getObjectTemperature() {
  mlx90632_meas_select_t meas_mode = getMeasurementSelect();

  double S;
  int16_t ram_ambient, ram_ref;

  if (meas_mode == MLX90632_MEAS_EXTENDED_RANGE) {
    // Extended range mode: use RAM_52-59
    uint16_t ram52, ram53, ram54, ram55, ram56, ram57, ram58, ram59;
    read_register16(swapBytes(MLX90632_REG_RAM_52), &ram52);
    read_register16(swapBytes(MLX90632_REG_RAM_53), &ram53);
    read_register16(swapBytes(MLX90632_REG_RAM_54), &ram54);
    read_register16(swapBytes(MLX90632_REG_RAM_55), &ram55);
    read_register16(swapBytes(MLX90632_REG_RAM_56), &ram56);
    read_register16(swapBytes(MLX90632_REG_RAM_57), &ram57);
    read_register16(swapBytes(MLX90632_REG_RAM_58), &ram58);
    read_register16(swapBytes(MLX90632_REG_RAM_59), &ram59);

    int16_t r52 = (int16_t)ram52, r53 = (int16_t)ram53, r54 = (int16_t)ram54;
    int16_t r55 = (int16_t)ram55, r56 = (int16_t)ram56, r57 = (int16_t)ram57;
    int16_t r58 = (int16_t)ram58, r59 = (int16_t)ram59;

    // Extended range S calculation
    S = ((double)r52 - (double)r53 - (double)r55 + (double)r56) / 2.0 +
        (double)r58 + (double)r59;
    ram_ambient = r54;
    ram_ref = r57;

  } else {
    // Medical mode: use cycle position and RAM_4-9
    uint8_t cycle_pos = readCyclePosition();

    uint16_t ram4, ram5, ram6, ram7, ram8, ram9;
    read_register16(swapBytes(MLX90632_REG_RAM_4), &ram4);
    read_register16(swapBytes(MLX90632_REG_RAM_5), &ram5);
    read_register16(swapBytes(MLX90632_REG_RAM_6), &ram6);
    read_register16(swapBytes(MLX90632_REG_RAM_7), &ram7);
    read_register16(swapBytes(MLX90632_REG_RAM_8), &ram8);
    read_register16(swapBytes(MLX90632_REG_RAM_9), &ram9);

    int16_t r4 = (int16_t)ram4, r5 = (int16_t)ram5, r6 = (int16_t)ram6;
    int16_t r7 = (int16_t)ram7, r8 = (int16_t)ram8, r9 = (int16_t)ram9;

    // Medical mode S calculation based on cycle position
    if (cycle_pos == 2) {
      S = ((double)r4 + (double)r5) / 2.0;
    } else if (cycle_pos == 1) {
      S = ((double)r7 + (double)r8) / 2.0;
    } else {
      // Invalid cycle position
      return NAN;
    }

    ram_ambient = r6;
    ram_ref = r9;
  }

  // Pre-calculations for object temperature
  double VRTO = (double)ram_ref + Ka * ((double)ram_ambient / 12.0);
  double STO = ((S / 12.0) / VRTO) * pow(2, 19);

  // Calculate AMB for ambient temperature (needed for TADUT)
  double VRTA = (double)ram_ref + Gb * ((double)ram_ambient / 12.0);
  double AMB = ((double)ram_ambient / 12.0) / VRTA * pow(2, 19);

  // Additional temperature calculations
  double TADUT = (AMB - Eb) / Ea + 25.0;
  double TAK = TADUT + 273.15;
  double emissivity = 1.0;

  // For the first iteration, use current TADUT as TODUT approximation
  double TODUT = TADUT;

  // Calculate final object temperature using Stefan-Boltzmann law
  double denominator = emissivity * Fa * Ha * (1.0 + Ga * (TODUT - TO0) + Fb * (TADUT - TA0));
  double TO_K4 = (STO / denominator) + pow(TAK, 4);
  double TO = pow(TO_K4, 0.25) - 273.15 - Hb;

  // Update TO0 and TA0 with current measurements for next calculation
  TO0 = TO;
  TA0 = TADUT;

  return TO;
}

// ESPHome setup function
void MLX90632Sensor::setup() {
  ESP_LOGI(TAG, "%s Setting up MLX90632 sensor", FW_VERSION);

  // Initialize sensor
  if (!begin()) {
    ESP_LOGE(TAG, "%s Sensor initialization failed", FW_VERSION);
    this->mark_failed();
    return;
  }

  // Reset device
  if (!reset()) {
    ESP_LOGE(TAG, "%s Device reset failed", FW_VERSION);
    this->mark_failed();
    return;
  }

  // Set to continuous mode and medical measurement
  if (!setMode(MLX90632_MODE_CONTINUOUS)) {
    ESP_LOGE(TAG, "%s Failed to set continuous mode", FW_VERSION);
    this->mark_failed();
    return;
  }

  if (!setMeasurementSelect(MLX90632_MEAS_MEDICAL)) {
    ESP_LOGE(TAG, "%s Failed to set medical measurement mode", FW_VERSION);
    this->mark_failed();
    return;
  }

  if (!setRefreshRate(MLX90632_REFRESH_2HZ)) {
    ESP_LOGE(TAG, "%s Failed to set refresh rate", FW_VERSION);
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "%s Sensor setup complete - Continuous Medical Mode at 2Hz", FW_VERSION);
}

// ESPHome update function
void MLX90632Sensor::update() {
  // Check if new data is available
  if (!isNewData()) {
    ESP_LOGD(TAG, "%s No new data available", FW_VERSION);
    return;
  }

  // Read temperatures
  double object_temp = getObjectTemperature();
  double ambient_temp = getAmbientTemperature();

  if (isnan(object_temp)) {
    ESP_LOGW(TAG, "%s Invalid object temperature (NaN)", FW_VERSION);
    return;
  }

  // Publish temperatures
  this->publish_state(object_temp);

  ESP_LOGI(TAG, "%s Temperatures: Object=%.2f°C, Ambient=%.2f°C", 
           FW_VERSION, object_temp, ambient_temp);

  // Reset new data flag
  resetNewData();
}
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

// ESPHome dump config
void MLX90632Sensor::dump_config() {
  LOG_SENSOR("", "MLX90632", this);
  ESP_LOGI(TAG, "%s Product Code: 0x%04X", FW_VERSION, getProductCode());
  ESP_LOGI(TAG, "%s EEPROM Version: 0x%04X", FW_VERSION, getEEPROMVersion());
  ESP_LOGI(TAG, "%s Mode: %s", FW_VERSION, 
           getMode() == MLX90632_MODE_CONTINUOUS ? "Continuous" : "Other");
  ESP_LOGI(TAG, "%s Measurement: %s", FW_VERSION,
           getMeasurementSelect() == MLX90632_MEAS_MEDICAL ? "Medical" : "Extended");
}

}  // namespace mlx90632
}  // namespace esphome
