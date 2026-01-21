#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "mlx90632_registers.h"
#include <cmath>

namespace esphome {
namespace mlx90632 {

static const char *const TAG = "mlx90632";
static const char *const FW_VERSION = "V.N1";

class MLX90632Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_emissivity(float emissivity) { emissivity_ = emissivity; }

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
  
  float emissivity_{1.0};
  bool setup_complete_{false};
};

}  // namespace mlx90632
}  // namespace esphome
