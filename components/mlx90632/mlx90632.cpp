#include "esphome/core/log.h"
#include "mlx90632.h"

namespace esphome {
namespace mlx90632 {

static const char *TAG = "mlx90632.sensor";

void MLX90632Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MLX90632...");
  
  // Create I2C adapter
  i2c_adapter_ = new ESPHomeI2CAdapter(this->i2c_);
  
  // Initialize the Adafruit MLX90632 library with the ESPHome I2C adapter
  if (!mlx90632_.begin(this->address_, i2c_adapter_)) {
    ESP_LOGE(TAG, "Failed to initialize MLX90632");
    this->mark_failed();
    return;
  }
  
  // Set to continuous mode for polling
  if (!mlx90632_.setMode(MLX90632_MODE_CONTINUOUS)) {
    ESP_LOGE(TAG, "Failed to set measurement mode");
    this->mark_failed();
    return;
  }
  
  // Set medical mode (default)
  if (!mlx90632_.setMeasurementSelect(MLX90632_MEAS_MEDICAL)) {
    ESP_LOGE(TAG, "Failed to set measurement select");
    this->mark_failed();
    return;
  }
  
  // Set refresh rate to 2Hz
  if (!mlx90632_.setRefreshRate(MLX90632_REFRESH_2HZ)) {
    ESP_LOGE(TAG, "Failed to set refresh rate");
    this->mark_failed();
    return;
  }
  
  ESP_LOGD(TAG, "MLX90632 initialized successfully");
  
  uint64_t product_id = mlx90632_.getProductID();
  ESP_LOGI(TAG, "Product ID: 0x%012" PRIx64, product_id);
}

void MLX90632Sensor::update() {
  if (this->is_failed()) {
    return;
  }
  
  // Check if new data is available
  if (!mlx90632_.isNewData()) {
    ESP_LOGD(TAG, "No new data available yet");
    return;
  }
  
  // Read ambient temperature
  double ambient_temp = mlx90632_.getAmbientTemperature();
  ESP_LOGD(TAG, "Ambient Temperature: %.2f°C", ambient_temp);
  
  // Read object temperature
  double object_temp = mlx90632_.getObjectTemperature();
  ESP_LOGD(TAG, "Object Temperature: %.2f°C", object_temp);
  
  // Publish object temperature as the main sensor value
  this->publish_state(object_temp);
  
  // Reset new data flag for next measurement
  mlx90632_.resetNewData();
}

void MLX90632Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MLX90632 Temperature Sensor");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Failed to initialize MLX90632");
  }
}

}  // namespace mlx90632
}  // namespace esphome
