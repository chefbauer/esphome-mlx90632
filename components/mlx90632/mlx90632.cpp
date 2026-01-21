#include "mlx90632.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace mlx90632 {

using namespace mlx90632_registers;

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

// I2C Helper: Read 32-bit register (two 16-bit registers: LSW, MSW)
bool MLX90632Sensor::read_register32(uint16_t lsw_reg, uint32_t *value) {
  uint16_t lsw = 0, msw = 0;
  
  if (!read_register16(lsw_reg, &lsw)) return false;
  if (!read_register16(lsw_reg + 1, &msw)) return false;
  
  *value = ((uint32_t)msw << 16) | lsw;
  ESP_LOGD(TAG, "%s Read32 [0x%04X]: LSW=0x%04X MSW=0x%04X -> 0x%08X", 
           FW_VERSION, lsw_reg, lsw, msw, *value);
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

// Setup: Initialize sensor
void MLX90632Sensor::setup() {
  ESP_LOGE(TAG, "%s ========== SETUP() CALLED ==========", FW_VERSION);
  ESP_LOGI(TAG, "%s Setting up MLX90632...", FW_VERSION);
  
  // Read product ID
  uint16_t id0, id1, id2;
  if (!read_register16(EE_ID0, &id0) || 
      !read_register16(EE_ID1, &id1) || 
      !read_register16(EE_ID2, &id2)) {
    ESP_LOGE(TAG, "%s Failed to read product ID", FW_VERSION);
    this->mark_failed();
    return;
  }
  
  product_id_ = ((uint64_t)id0 << 32) | ((uint64_t)id1 << 16) | id2;
  ESP_LOGI(TAG, "%s Product ID: 0x%012llX", FW_VERSION, product_id_);
  
  // Read calibration from EEPROM
  if (!read_calibration()) {
    ESP_LOGE(TAG, "%s Failed to read calibration", FW_VERSION);
    this->mark_failed();
    return;
  }
  
  // Read current control register
  uint16_t ctrl_before;
  if (read_register16(REG_CONTROL, &ctrl_before)) {
    ESP_LOGI(TAG, "%s Control before: 0x%04X", FW_VERSION, ctrl_before);
  }
  
  // TEMPORARY: Test Medical Continuous mode (skip extended switch)
  // Set Continuous mode (11) with Medical range (0x00)
  // meas_select[4:0]=0x00 in bits 8:4 = 0x000, mode[1:0]=11 = 0x003
  uint16_t ctrl_value = 0x0003;  // Medical + Continuous
  if (!write_register16(REG_CONTROL, ctrl_value)) {
    ESP_LOGE(TAG, "%s Failed to set Medical Continuous", FW_VERSION);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "%s Control set to 0x%04X (Medical Continuous)", FW_VERSION, ctrl_value);
  
  // Read back to verify
  uint16_t ctrl_after_write;
  if (!read_register16(REG_CONTROL, &ctrl_after_write)) {
    ESP_LOGE(TAG, "%s Failed to read control after write", FW_VERSION);
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "%s Control after write: 0x%04X (expected 0x%04X)", FW_VERSION, ctrl_after_write, ctrl_value);
  
  // Verify control register was set (give sensor time to wake up)
  this->set_timeout(100, [this]() {
    uint16_t ctrl_after;
    if (read_register16(REG_CONTROL, &ctrl_after)) {
      ESP_LOGD(TAG, "%s Control register after timeout: 0x%04X (expected 0x%04X)", 
               FW_VERSION, ctrl_after, CTRL_MODE_CONTINUOUS);
      if ((ctrl_after & CTRL_MODE_CONTINUOUS) == 0) {
        ESP_LOGW(TAG, "%s Sensor not in continuous mode!", FW_VERSION);
      }
    }
    
    // Read initial status
    uint16_t status;
    if (read_register16(REG_STATUS, &status)) {
      ESP_LOGD(TAG, "%s Initial status: 0x%04X (NewData=%d, EEBusy=%d, DevBusy=%d)", 
               FW_VERSION, status, 
               (status & STATUS_NEW_DATA) ? 1 : 0,
               (status & STATUS_EEPROM_BUSY) ? 1 : 0,
               (status & STATUS_DEVICE_BUSY) ? 1 : 0);
    }
  });
  
  setup_complete_ = true;
  setup_time_ = millis();
  ESP_LOGI(TAG, "%s === Setup complete at %.3fs ===", FW_VERSION, setup_time_ / 1000.0);
}

// Read calibration constants from EEPROM
bool MLX90632Sensor::read_calibration() {
  ESP_LOGD(TAG, "%s Reading calibration from EEPROM...", FW_VERSION);
  
  // Wait for EEPROM to be ready
  uint16_t status;
  for (int i = 0; i < 10; i++) {
    if (!read_register16(REG_STATUS, &status)) return false;
    if ((status & STATUS_EEPROM_BUSY) == 0) break;
    ESP_LOGD(TAG, "%s EEPROM busy, waiting... (attempt %d/10)", FW_VERSION, i+1);
    delay(10);
  }
  
  if ((status & STATUS_EEPROM_BUSY) != 0) {
    ESP_LOGE(TAG, "%s EEPROM still busy after 100ms!", FW_VERSION);
    return false;
  }
  
  ESP_LOGD(TAG, "%s EEPROM ready, starting reads...", FW_VERSION);
  
  // Read 32-bit constants WITH DELAYS BETWEEN READS
  uint32_t ee_p_r, ee_p_g, ee_p_t, ee_p_o;
  uint32_t ee_aa, ee_ab, ee_ba, ee_bb, ee_ca, ee_cb, ee_da, ee_db;
  uint32_t ee_ea, ee_eb, ee_fa, ee_fb, ee_ga;
  
  if (!read_register32(EE_P_R_LSW, &ee_p_r)) return false;
  delay(5);  // EEPROM delay
  if (!read_register32(EE_P_G_LSW, &ee_p_g)) return false;
  delay(5);
  if (!read_register32(EE_P_T_LSW, &ee_p_t)) return false;
  delay(5);
  if (!read_register32(EE_P_O_LSW, &ee_p_o)) return false;
  delay(5);
  if (!read_register32(EE_AA_LSW, &ee_aa)) return false;
  delay(5);
  if (!read_register32(EE_AB_LSW, &ee_ab)) return false;
  delay(5);
  if (!read_register32(EE_BA_LSW, &ee_ba)) return false;
  delay(5);
  if (!read_register32(EE_BB_LSW, &ee_bb)) return false;
  delay(5);
  if (!read_register32(EE_CA_LSW, &ee_ca)) return false;
  delay(5);
  if (!read_register32(EE_CB_LSW, &ee_cb)) return false;
  delay(5);
  if (!read_register32(EE_DA_LSW, &ee_da)) return false;
  delay(5);
  if (!read_register32(EE_DB_LSW, &ee_db)) return false;
  delay(5);
  if (!read_register32(EE_EA_LSW, &ee_ea)) return false;
  delay(5);
  if (!read_register32(EE_EB_LSW, &ee_eb)) return false;
  delay(5);
  if (!read_register32(EE_FA_LSW, &ee_fa)) return false;
  delay(5);
  if (!read_register32(EE_FB_LSW, &ee_fb)) return false;
  delay(5);
  if (!read_register32(EE_GA_LSW, &ee_ga)) return false;
  delay(5);
  
  // Read 16-bit constants WITH DELAYS
  uint16_t ee_gb, ee_ka, ee_kb, ee_ha, ee_hb;
  if (!read_register16(EE_GB, &ee_gb)) return false;
  delay(5);
  if (!read_register16(EE_KA, &ee_ka)) return false;
  delay(5);
  if (!read_register16(EE_KB, &ee_kb)) return false;
  delay(5);
  if (!read_register16(EE_HA, &ee_ha)) return false;
  delay(5);
  if (!read_register16(EE_HB, &ee_hb)) return false;
  
  // Convert to calibration constants with scale factors
  P_R = (double)(int32_t)ee_p_r * pow(2, -8);
  P_G = (double)(int32_t)ee_p_g * pow(2, -20);
  P_T = (double)(int32_t)ee_p_t * pow(2, -44);
  P_O = (double)(int32_t)ee_p_o * pow(2, -8);
  Aa = (double)(int32_t)ee_aa * pow(2, -16);
  Ab = (double)(int32_t)ee_ab * pow(2, -16);
  Ba = (double)(int32_t)ee_ba * pow(2, -32);
  Bb = (double)(int32_t)ee_bb * pow(2, -32);
  Ca = (double)(int32_t)ee_ca * pow(2, -28);
  Cb = (double)(int32_t)ee_cb * pow(2, -32);
  Da = (double)(int32_t)ee_da * pow(2, -22);
  Db = (double)(int32_t)ee_db * pow(2, -22);
  Ea = (double)(int32_t)ee_ea * pow(2, -20);
  Eb = (double)(int32_t)ee_eb * pow(2, -20);
  Fa = (double)(int32_t)ee_fa * pow(2, -12);
  Fb = (double)(int32_t)ee_fb * pow(2, -10);
  Ga = (double)(int32_t)ee_ga * pow(2, -20);
  Gb = (double)(int16_t)ee_gb * pow(2, -10);
  Ka = (double)(int16_t)ee_ka * pow(2, -8);
  Kb = (int16_t)ee_kb;
  Ha = (double)(int16_t)ee_ha * pow(2, -10);
  Hb = (double)(int16_t)ee_hb * pow(2, -10);
  
  // Log calibration - raw hex values
  ESP_LOGD(TAG, "%s Cal RAW 32-bit: P_R=0x%08X P_G=0x%08X P_T=0x%08X P_O=0x%08X",
           FW_VERSION, ee_p_r, ee_p_g, ee_p_t, ee_p_o);
  ESP_LOGD(TAG, "%s Cal RAW 32-bit: Aa=0x%08X Ba=0x%08X Ga=0x%08X",
           FW_VERSION, ee_aa, ee_ba, ee_ga);
  ESP_LOGD(TAG, "%s Cal RAW 16-bit: Gb=0x%04X Ka=0x%04X Kb=0x%04X Ha=0x%04X Hb=0x%04X",
           FW_VERSION, ee_gb, ee_ka, ee_kb, ee_ha, ee_hb);
  
  // Log scaled calibration values
  ESP_LOGI(TAG, "%s Calibration scaled:", FW_VERSION);
  ESP_LOGI(TAG, "%s   P_R=%.6f P_G=%.9f P_T=%.12f P_O=%.6f",
           FW_VERSION, P_R, P_G, P_T, P_O);
  ESP_LOGI(TAG, "%s   Aa=%.6f Ab=%.6f Ba=%.9f Bb=%.9f",
           FW_VERSION, Aa, Ab, Ba, Bb);
  ESP_LOGI(TAG, "%s   Ca=%.9f Cb=%.9f Da=%.9f Db=%.9f",
           FW_VERSION, Ca, Cb, Da, Db);
  ESP_LOGI(TAG, "%s   Ea=%.9f Eb=%.9f Fa=%.9f Fb=%.9f",
           FW_VERSION, Ea, Eb, Fa, Fb);
  ESP_LOGI(TAG, "%s   Ga=%.9f Gb=%.6f Ka=%.6f Kb=%d",
           FW_VERSION, Ga, Gb, Ka, Kb);
  ESP_LOGI(TAG, "%s   Ha=%.6f Hb=%.6f",
           FW_VERSION, Ha, Hb);
  
  return true;
}

// Check if new measurement data is available
bool MLX90632Sensor::check_new_data() {
  uint16_t status;
  if (!read_register16(REG_STATUS, &status)) return false;
  
  ESP_LOGVV(TAG, "%s Status: 0x%04X (NewData=%d, Cycle=%d, EEBusy=%d, DevBusy=%d)", 
            FW_VERSION, status,
            (status & STATUS_NEW_DATA) ? 1 : 0,
            (status & STATUS_CYCLE_POS_MASK) >> 1,
            (status & STATUS_EEPROM_BUSY) ? 1 : 0,
            (status & STATUS_DEVICE_BUSY) ? 1 : 0);
  
  return (status & STATUS_NEW_DATA) != 0;
}

// Calculate ambient temperature (SIMPLIFIED for testing)
float MLX90632Sensor::calculate_ambient_temperature() {
  // Read RAM registers - Extended Range mode
  uint16_t ram_ambient, ram_ref;
  
  if (measurement_mode_ == MEASUREMENT_MODE_EXTENDED) {
    if (!read_register16(RAM_54, &ram_ambient)) return NAN;
    if (!read_register16(RAM_57, &ram_ref)) return NAN;
  } else {
    if (!read_register16(RAM_6, &ram_ambient)) return NAN;
    if (!read_register16(RAM_9, &ram_ref)) return NAN;
  }
  
  int16_t ambient_new_raw = (int16_t)ram_ambient;
  int16_t ambient_ref_raw = (int16_t)ram_ref;
  
  // SIMPLIFIED calculation - just to get something reasonable
  // Full Melexis DSPv5 comes later
  double ambient_temp = 25.0 + (ambient_new_raw / 100.0);
  
  ESP_LOGD(TAG, "%s [AMB] raw=%d ref=%d -> T=%.2f°C", 
           FW_VERSION, ambient_new_raw, ambient_ref_raw, ambient_temp);
  
  return (float)ambient_temp;
}

// Calculate object temperature (SIMPLIFIED for testing)
float MLX90632Sensor::calculate_object_temperature() {
  // Read RAM registers
  uint16_t ram_object, ram_ambient;
  
  if (measurement_mode_ == MEASUREMENT_MODE_EXTENDED) {
    if (!read_register16(RAM_52, &ram_object)) return NAN;
    if (!read_register16(RAM_54, &ram_ambient)) return NAN;
  } else {
    if (!read_register16(RAM_6, &ram_object)) return NAN;
    if (!read_register16(RAM_9, &ram_ambient)) return NAN;
  }
  
  int16_t object_new_raw = (int16_t)ram_object;
  int16_t ambient_new_raw = (int16_t)ram_ambient;
  
  // SIMPLIFIED calculation - just to get something reasonable
  // Full Melexis DSPv5 comes later
  double ambient_temp = 25.0 + (ambient_new_raw / 100.0);
  double object_temp = ambient_temp + (object_new_raw / 50.0);
  
  ESP_LOGD(TAG, "%s [OBJ] obj_raw=%d amb_raw=%d -> T=%.2f°C", 
           FW_VERSION, object_new_raw, ambient_new_raw, object_temp);
  
  return (float)object_temp;
}

// Update: Read and publish temperatures
void MLX90632Sensor::update() {
  uint32_t now = millis();
  float setup_seconds = setup_time_ / 1000.0;
  ESP_LOGI(TAG, "%s === UPDATE #%d === (Setup: %.3fs, P_R=%.2f)", 
           FW_VERSION, update_count_++, setup_seconds, P_R);
  
  // Don't run before 2 seconds after boot
  if (now < 2000) {
    ESP_LOGD(TAG, "%s Skipping - too early (millis=%lu)", FW_VERSION, now);
    return;
  }
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "%s Component marked as failed!", FW_VERSION);
    return;
  }
  
  // Check if setup ran and calibration is valid
  if (!setup_complete_ || P_R == 0.0) {
    ESP_LOGE(TAG, "%s Setup incomplete! ready=%d P_R=%.2f", FW_VERSION, setup_complete_, P_R);
    return;
  }
  
  // LOG SETUP INFO EVERY TIME (for debugging via web interface)
  ESP_LOGI(TAG, "%s === SETUP SUMMARY ===", FW_VERSION);
  ESP_LOGI(TAG, "%s Measurement Mode: %s", FW_VERSION, 
           measurement_mode_ == MEASUREMENT_MODE_MEDICAL ? "MEDICAL" : "EXTENDED");
  ESP_LOGI(TAG, "%s Product ID: 0x%012llX", FW_VERSION, product_id_);
  ESP_LOGI(TAG, "%s Calibration loaded: P_R=%.2f", FW_VERSION, P_R);
  
  // Read and log current control register
  uint16_t ctrl_current;
  if (read_register16(REG_CONTROL, &ctrl_current)) {
    ESP_LOGI(TAG, "%s Control Register: 0x%04X (Mode=%s, MeasSel=0x%X)", FW_VERSION, ctrl_current,
             (ctrl_current & 0x0003) == 0 ? "HALT" : 
             (ctrl_current & 0x0003) == 1 ? "STEP" : 
             (ctrl_current & 0x0003) == 3 ? "CONTINUOUS" : "UNKNOWN",
             (ctrl_current >> 4) & 0x1F);
  }
  
  ESP_LOGD(TAG, "%s Starting measurement cycle (continuous mode)...", FW_VERSION);
  
  // READ ALL RELEVANT REGISTERS FOR DEBUGGING
  uint16_t ctrl_reg, status_reg;
  if (read_register16(REG_CONTROL, &ctrl_reg)) {
    ESP_LOGI(TAG, "%s Control: 0x%04X", FW_VERSION, ctrl_reg);
  }
  if (read_register16(REG_STATUS, &status_reg)) {
    ESP_LOGI(TAG, "%s Status: 0x%04X (NewData=%d, CyclePos=%d, EEBusy=%d, DevBusy=%d)", 
             FW_VERSION, status_reg,
             (status_reg & STATUS_NEW_DATA) ? 1 : 0,
             (status_reg & STATUS_CYCLE_POS_MASK) >> 2,
             (status_reg & STATUS_EEPROM_BUSY) ? 1 : 0,
             (status_reg & STATUS_DEVICE_BUSY) ? 1 : 0);
  }
  
  // In continuous mode: Just check if new data is available
  // (Sensor runs continuously after setup, no need to trigger SOB every time!)
  if (!check_new_data()) {
    ESP_LOGW(TAG, "%s No new data available yet", FW_VERSION);
    
    // DEBUG: Try reading RAM registers anyway to see if they change
    uint16_t ram_obj, ram_amb;
    if (measurement_mode_ == MEASUREMENT_MODE_EXTENDED) {
      read_register16(RAM_52, &ram_obj);
      read_register16(RAM_54, &ram_amb);
    } else {
      read_register16(RAM_6, &ram_obj);
      read_register16(RAM_9, &ram_amb);
    }
    ESP_LOGD(TAG, "%s RAM values (no new data): OBJ=0x%04X (%d), AMB=0x%04X (%d)", 
             FW_VERSION, ram_obj, (int16_t)ram_obj, ram_amb, (int16_t)ram_amb);
    
    return;
  }
  
  // Calculate temperatures
  float ambient_temp = calculate_ambient_temperature();
  float object_temp = calculate_object_temperature();
  
  // Clear NewData flag so sensor can write new measurement!
  uint16_t status_clear = status & ~STATUS_NEW_DATA;
  if (!write_register16(REG_STATUS, status_clear)) {
    ESP_LOGW(TAG, "%s Failed to clear NewData flag", FW_VERSION);
  }
  
  ESP_LOGI(TAG, "%s Ambient: %.2f°C, Object: %.2f°C", FW_VERSION, ambient_temp, object_temp);
  
  // Publish object temperature to sensor
  ESP_LOGD(TAG, "%s Publishing state...", FW_VERSION);
  this->publish_state(object_temp);
  ESP_LOGD(TAG, "%s === UPDATE COMPLETE ===", FW_VERSION);
}

// Dump config
void MLX90632Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MLX90632:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Object Temperature", this);
}

}  // namespace mlx90632
}  // namespace esphome
