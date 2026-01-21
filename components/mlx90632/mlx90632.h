#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "Adafruit_MLX90632.h"
#include "ESPHomeI2CAdapter.h"

namespace esphome {
namespace mlx90632 {

class MLX90632Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_object_temperature_sensor(sensor::Sensor *sensor) { object_temperature_sensor_ = sensor; }
  void set_ambient_temperature_sensor(sensor::Sensor *sensor) { ambient_temperature_sensor_ = sensor; }

 private:
  Adafruit_MLX90632 mlx90632_;
  ESPHomeI2CAdapter* i2c_adapter_;
  sensor::Sensor *object_temperature_sensor_{nullptr};
  sensor::Sensor *ambient_temperature_sensor_{nullptr};
};

}  // namespace mlx90632
}  // namespace esphome
