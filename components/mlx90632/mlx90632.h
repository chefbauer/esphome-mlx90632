#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "Adafruit_MLX90632.h"
#include "ESPHomeI2CAdapter.h"

namespace esphome {
namespace mlx90632 {

class MLX90632Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

 private:
  Adafruit_MLX90632 mlx90632_;
  ESPHomeI2CAdapter* i2c_adapter_;
};

}  // namespace mlx90632
}  // namespace esphome
