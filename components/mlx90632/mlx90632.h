#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "mlx90632_registers.h"
#include <cmath>

namespace esphome {
namespace mlx90632 {

static const char *const TAG = "mlx90632";
static const char *const FW_VERSION = "V.N3";

enum MeasurementMode {
  MEASUREMENT_MODE_MEDICAL = 0,
  MEASUREMENT_MODE_EXTENDED = 1
};

enum RefreshRate {
  REFRESH_RATE_0_5HZ = 0,
  REFRESH_RATE_1HZ = 1,
  REFRESH_RATE_2HZ = 2,
  REFRESH_RATE_4HZ = 3,
  REFRESH_RATE_8HZ = 4,
  REFRESH_RATE_16HZ = 5,
  REFRESH_RATE_32HZ = 6,
  REFRESH_RATE_64HZ = 7
};

class MLX90632Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  MLX90632Sensor() : setup_complete_(false), update_count_(0) {}
  
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_measurement_mode(MeasurementMode mode) { measurement_mode_ = mode; }
  void set_refresh_rate(RefreshRate rate) { refresh_rate_ = rate; }
  void set_emissivity(float emissivity) { emissivity_ = emissivity; }
  
  bool is_ready() const { return setup_complete_; }

 protected:
  // I2C read/write helpers
  bool read_register16(uint16_t reg, uint16_t *value);
  bool read_register32(uint16_t lsw_reg, uint32_t *value);
  bool write_register16(uint16_t reg, uint16_t value);
  
  // Sensor functions
  bool check_new_data();
  bool read_calibration();
  float calculate_ambient_temperature();
  float calculate_object_temperature();
  
  // Calibration constants (from EEPROM)
  double P_R, P_G, P_T, P_O;
  double Aa, Ab, Ba, Bb, Ca, Cb, Da, Db;
  double Ea, Eb, Fa, Fb, Ga, Gb, Ka, Ha, Hb;
  int16_t Kb;
  double TO0{25.0};  // Previous object temperature
  double TA0{25.0};  // Previous ambient temperature
  
  MeasurementMode measurement_mode_{MEASUREMENT_MODE_EXTENDED};
  RefreshRate refresh_rate_{REFRESH_RATE_2HZ};
  float emissivity_{1.0};
  bool setup_complete_{false};
  uint8_t update_count_{0};  // Count updates before forcing setup
};

}  // namespace mlx90632
}  // namespace esphome
