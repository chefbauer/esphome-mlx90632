#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "Adafruit_MLX90632.h"

namespace esphome {
namespace mlx90632 {

class MLX90632Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_object_temperature_sensor(sensor::Sensor *sensor) { object_temperature_sensor_ = sensor; }
  void set_ambient_temperature_sensor(sensor::Sensor *sensor) { ambient_temperature_sensor_ = sensor; }
  void set_measurement_select(mlx90632_meas_select_t meas_select) { measurement_select_ = meas_select; }
  void set_refresh_rate(mlx90632_refresh_rate_t refresh_rate) { refresh_rate_ = refresh_rate; }
  void set_emissivity(double emissivity) { emissivity_ = emissivity; }
  double get_emissivity() const { return (emissivity_ == 0.0) ? 1.0 : emissivity_; }

 protected:
  Adafruit_MLX90632 mlx90632_;
  sensor::Sensor *object_temperature_sensor_{nullptr};
  sensor::Sensor *ambient_temperature_sensor_{nullptr};
  mlx90632_meas_select_t measurement_select_{MLX90632_MEAS_MEDICAL};
  mlx90632_refresh_rate_t refresh_rate_{MLX90632_REFRESH_2HZ};
  double emissivity_{1.0};
};

}  // namespace mlx90632
}  // namespace esphome
