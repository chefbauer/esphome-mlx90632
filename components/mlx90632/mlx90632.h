#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include <cmath>

namespace esphome {
namespace mlx90632 {

static const char *const TAG = "mlx90632";
static const char *const FW_VERSION = "V.N6";

class MLX90632Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  // I2C helpers
  bool read_register16(uint16_t reg, uint16_t *value);
  bool write_register16(uint16_t reg, uint16_t value);
  
  // Temperature calculation (Melexis DSPv5 algorithm)
  float calculate_temperature(uint16_t raw_obj, uint16_t raw_amb);
};

}  // namespace mlx90632
}  // namespace esphome
