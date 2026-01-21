#include "mlx90632.h"
#include "esphome/core/log.h"


namespace esphome {
namespace mlx90632 {

static const char *const TAG = "mlx90632";
#define FW_VERSION "V.N1"

using namespace mlx90632_registers;

// I2C Helper: Read 16-bit register (big-endian)
bool MLX90632Component::read_register16(uint16_t reg, uint16_t *value) {
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
bool MLX90632Component::read_register32(uint16_t lsw_reg, uint32_t *value) {
  uint16_t lsw = 0, msw = 0;
  
  if (!read_register16(lsw_reg, &lsw)) return false;
  if (!read_register16(lsw_reg + 1, &msw)) return false;
  
  *value = ((uint32_t)msw << 16) | lsw;
  ESP_LOGD(TAG, "%s Read32 [0x%04X]: LSW=0x%04X MSW=0x%04X -> 0x%08X", 
           FW_VERSION, lsw_reg, lsw, msw, *value);
  return true;
}

// I2C Helper: Write 16-bit register (big-endian)
bool MLX90632Component::write_register16(uint16_t reg, uint16_t value) {
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
void MLX90632Component::setup() {
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
  
  uint64_t product_id = ((uint64_t)id0 << 32) | ((uint64_t)id1 << 16) | id2;
  ESP_LOGI(TAG, "%s Product ID: 0x%012llX", FW_VERSION, product_id);
  
  // Read calibration from EEPROM
  if (!read_calibration()) {
    ESP_LOGE(TAG, "%s Failed to read calibration", FW_VERSION);
    this->mark_failed();
    return;
  }
  
  // Read current control register
  uint16_t ctrl_before;
  if (read_register16(REG_CONTROL, &ctrl_before)) {
    ESP_LOGD(TAG, "%s Control register before: 0x%04X", FW_VERSION, ctrl_before);
  }
  
  // Wake sensor and set continuous measurement mode with SOB
  uint16_t ctrl_value = CTRL_MODE_CONTINUOUS | CTRL_SOB;
  if (!write_register16(REG_CONTROL, ctrl_value)) {
    ESP_LOGE(TAG, "%s Failed to set continuous mode", FW_VERSION);
    this->mark_failed();
    return;
  }
  
  // Verify control register was set (give sensor time to wake up)
  this->set_timeout(100, [this]() {
    uint16_t ctrl_after;
    if (read_register16(REG_CONTROL, &ctrl_after)) {
      ESP_LOGD(TAG, "%s Control register after: 0x%04X (expected 0x%04X)", 
               FW_VERSION, ctrl_after, CTRL_MODE_CONTINUOUS | CTRL_SOB);
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
  
  ESP_LOGI(TAG, "%s MLX90632 initialized successfully", FW_VERSION);
}

// Read calibration constants from EEPROM
bool MLX90632Component::read_calibration() {
  ESP_LOGD(TAG, "%s Reading calibration from EEPROM...", FW_VERSION);
  
  // Read 32-bit constants
  uint32_t ee_p_r, ee_p_g, ee_p_t, ee_p_o;
  uint32_t ee_aa, ee_ab, ee_ba, ee_bb, ee_ca, ee_cb, ee_da, ee_db;
  uint32_t ee_ea, ee_eb, ee_fa, ee_fb, ee_ga;
  
  if (!read_register32(EE_P_R_LSW, &ee_p_r)) return false;
  if (!read_register32(EE_P_G_LSW, &ee_p_g)) return false;
  if (!read_register32(EE_P_T_LSW, &ee_p_t)) return false;
  if (!read_register32(EE_P_O_LSW, &ee_p_o)) return false;
  if (!read_register32(EE_AA_LSW, &ee_aa)) return false;
  if (!read_register32(EE_AB_LSW, &ee_ab)) return false;
  if (!read_register32(EE_BA_LSW, &ee_ba)) return false;
  if (!read_register32(EE_BB_LSW, &ee_bb)) return false;
  if (!read_register32(EE_CA_LSW, &ee_ca)) return false;
  if (!read_register32(EE_CB_LSW, &ee_cb)) return false;
  if (!read_register32(EE_DA_LSW, &ee_da)) return false;
  if (!read_register32(EE_DB_LSW, &ee_db)) return false;
  if (!read_register32(EE_EA_LSW, &ee_ea)) return false;
  if (!read_register32(EE_EB_LSW, &ee_eb)) return false;
  if (!read_register32(EE_FA_LSW, &ee_fa)) return false;
  if (!read_register32(EE_FB_LSW, &ee_fb)) return false;
  if (!read_register32(EE_GA_LSW, &ee_ga)) return false;
  
  // Read 16-bit constants
  uint16_t ee_gb, ee_ka, ee_kb, ee_ha, ee_hb;
  if (!read_register16(EE_GB, &ee_gb)) return false;
  if (!read_register16(EE_KA, &ee_ka)) return false;
  if (!read_register16(EE_KB, &ee_kb)) return false;
  if (!read_register16(EE_HA, &ee_ha)) return false;
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
bool MLX90632Component::check_new_data() {
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

// Calculate ambient temperature
float MLX90632Component::calculate_ambient_temperature() {
  // Read RAM registers
  uint16_t ram_ambient, ram_ref;
  
  if (measurement_mode_ == MEASUREMENT_MODE_EXTENDED) {
    if (!read_register16(RAM_54, &ram_ambient)) return NAN;
    if (!read_register16(RAM_57, &ram_ref)) return NAN;
    ESP_LOGD(TAG, "%s [AMB-EXT] RAM_54=0x%04X RAM_57=0x%04X", FW_VERSION, ram_ambient, ram_ref);
  } else {
    if (!read_register16(RAM_6, &ram_ambient)) return NAN;
    if (!read_register16(RAM_9, &ram_ref)) return NAN;
    ESP_LOGD(TAG, "%s [AMB-MED] RAM_6=0x%04X RAM_9=0x%04X", FW_VERSION, ram_ambient, ram_ref);
  }
  
  int16_t ambient = (int16_t)ram_ambient;
  int16_t ref = (int16_t)ram_ref;
  
  ESP_LOGD(TAG, "%s [AMB] ambient=%d ref=%d", FW_VERSION, ambient, ref);
  
  // Ambient temperature calculation (Melexis algorithm)
  double VRTA = ref + (ref - ambient) / 0.02;
  double AMB = Aa + Ba * VRTA;
  double amb_diff = ambient - AMB;
  double PTAT = amb_diff + Gb * amb_diff * amb_diff;
  double ambient_temp = PTAT / (1 + Ka * P_R) + 25.0;
  
  ESP_LOGD(TAG, "%s [AMB] VRTA=%.2f AMB=%.2f PTAT=%.2f -> T=%.2f°C", 
           FW_VERSION, VRTA, AMB, PTAT, ambient_temp);
  
  return (float)ambient_temp;
}

// Calculate object temperature
float MLX90632Component::calculate_object_temperature() {
  // Read RAM registers
  uint16_t ram_52, ram_53, ram_54, ram_56;
  
  if (!read_register16(RAM_52, &ram_52)) return NAN;
  if (!read_register16(RAM_53, &ram_53)) return NAN;
  if (!read_register16(RAM_54, &ram_54)) return NAN;
  if (!read_register16(RAM_56, &ram_56)) return NAN;
  
  ESP_LOGD(TAG, "%s [OBJ] RAM_52=0x%04X RAM_53=0x%04X RAM_54=0x%04X RAM_56=0x%04X", 
           FW_VERSION, ram_52, ram_53, ram_54, ram_56);
  
  int16_t object_new = (int16_t)ram_52;
  int16_t object_old = (int16_t)ram_53;
  int16_t ambient_new = (int16_t)ram_54;
  int16_t ambient_old = (int16_t)ram_56;
  
  ESP_LOGD(TAG, "%s [OBJ] obj_new=%d obj_old=%d amb_new=%d amb_old=%d", 
           FW_VERSION, object_new, object_old, ambient_new, ambient_old);
  
  // Simplified object temperature calculation
  // TODO: Implement full Melexis DSPv5 algorithm with iterations
  double VR_Ta = ambient_new + Gb * (ambient_new - ambient_old);
  double amb_temp = VR_Ta / (1 + Ka * P_R) + 25.0;
  
  double S = object_new - object_old;
  double object_temp = amb_temp + S * 0.01;  // Placeholder calculation
  
  ESP_LOGD(TAG, "%s [OBJ] VR_Ta=%.2f amb=%.2f S=%.2f -> T=%.2f°C", 
           FW_VERSION, VR_Ta, amb_temp, S, object_temp);
  
  return (float)object_temp;
}

// Update: Read and publish temperatures
void MLX90632Component::update() {
  ESP_LOGD(TAG, "%s === UPDATE CALLED ===", FW_VERSION);
  
  if (this->is_failed()) {
    ESP_LOGD(TAG, "%s Component failed", FW_VERSION);
    return;
  }
  
  // Run setup logic in first update if not done
  if (!setup_complete_) {
    ESP_LOGI(TAG, "%s Running setup in first update...", FW_VERSION);
    
    // Read product ID
    uint16_t id0, id1, id2;
    if (!read_register16(EE_ID0, &id0) || 
        !read_register16(EE_ID1, &id1) || 
        !read_register16(EE_ID2, &id2)) {
      ESP_LOGE(TAG, "%s Failed to read product ID", FW_VERSION);
      this->mark_failed();
      return;
    }
    
    uint64_t product_id = ((uint64_t)id0 << 32) | ((uint64_t)id1 << 16) | id2;
    ESP_LOGI(TAG, "%s Product ID: 0x%012llX", FW_VERSION, product_id);
    
    // Read calibration from EEPROM
    if (!read_calibration()) {
      ESP_LOGE(TAG, "%s Failed to read calibration", FW_VERSION);
      this->mark_failed();
      return;
    }
    
    // Wake sensor and set continuous measurement mode with SOB
    uint16_t ctrl_value = CTRL_MODE_CONTINUOUS | CTRL_SOB;
    if (!write_register16(REG_CONTROL, ctrl_value)) {
      ESP_LOGE(TAG, "%s Failed to set continuous mode", FW_VERSION);
      this->mark_failed();
      return;
    }
    
    ESP_LOGI(TAG, "%s Setup complete!", FW_VERSION);
    setup_complete_ = true;
    return; // Skip first measurement, let sensor settle
  }
  
  // DEBUG: Check control register
  uint16_t ctrl_reg;
  if (read_register16(REG_CONTROL, &ctrl_reg)) {
    ESP_LOGD(TAG, "%s Control register: 0x%04X (Continuous=%d, SOB=%d)", 
             FW_VERSION, ctrl_reg,
             (ctrl_reg & CTRL_MODE_CONTINUOUS) ? 1 : 0,
             (ctrl_reg & CTRL_SOB) ? 1 : 0);
  }
  
  // DEBUG: Log calibration values
  ESP_LOGD(TAG, "%s Cal: P_R=%.6f P_G=%.9f Aa=%.6f Ba=%.9f Ga=%.9f Gb=%.6f Ka=%.6f",
           FW_VERSION, P_R, P_G, Aa, Ba, Ga, Gb, Ka);
  
  // Read status register
  uint16_t status;
  if (read_register16(REG_STATUS, &status)) {
    ESP_LOGD(TAG, "%s Status: 0x%04X (NewData=%d, Cycle=%d)", 
             FW_VERSION, status,
             (status & STATUS_NEW_DATA) ? 1 : 0,
             (status & STATUS_CYCLE_POS_MASK) >> 1);
  }
  
  // Check for new data
  if (!check_new_data()) {
    ESP_LOGD(TAG, "%s No new data available", FW_VERSION);
    return;
  }
  
  // Calculate temperatures
  float ambient_temp = calculate_ambient_temperature();
  float object_temp = calculate_object_temperature();
  
  ESP_LOGD(TAG, "%s Ambient: %.2f°C, Object: %.2f°C", FW_VERSION, ambient_temp, object_temp);
  
  // Publish to sensors
  if (ambient_temperature_sensor_ != nullptr) {
    ambient_temperature_sensor_->publish_state(ambient_temp);
  }
  if (object_temperature_sensor_ != nullptr) {
    object_temperature_sensor_->publish_state(object_temp);
  }
}

// Dump config
void MLX90632Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MLX90632:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Ambient Temperature", ambient_temperature_sensor_);
  LOG_SENSOR("  ", "Object Temperature", object_temperature_sensor_);
}

}  // namespace mlx90632
}  // namespace esphome
