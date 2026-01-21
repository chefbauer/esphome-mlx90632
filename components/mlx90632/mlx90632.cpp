#include "esphome/core/log.h"
#include "mlx90632.h"
#include "ESPHomeI2CAdapter.h"

namespace esphome {
namespace mlx90632 {

static const char *TAG = "mlx90632";

void MLX90632Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MLX90632...");
  
  // Create I2C adapter using this I2CDevice and address
  // This adapter wraps I2CDevice's read_bytes/write_bytes methods
  ESPHomeI2CAdapter *i2c_adapter = new ESPHomeI2CAdapter(this, this->address_);
  
  // Initialize the Adafruit MLX90632 library with the ESPHome I2C adapter
  if (!mlx90632_.begin(this->address_, i2c_adapter)) {
    ESP_LOGE(TAG, "Failed to initialize MLX90632");
    this->mark_failed();
    return;
  }
  
  // Set ESPHome I2CDevice for atomic write-read transactions
  mlx90632_.set_esphome_i2c_device(this);
  
  // Set to continuous mode for polling
  if (!mlx90632_.setMode(MLX90632_MODE_CONTINUOUS)) {
    ESP_LOGE(TAG, "Failed to set measurement mode");
    this->mark_failed();
    return;
  }
  
  // Set medical mode or extended range (can be configured)
  if (!mlx90632_.setMeasurementSelect(measurement_select_)) {
    ESP_LOGE(TAG, "Failed to set measurement select");
    this->mark_failed();
    return;
  }
  
  // Log which mode is active
  if (measurement_select_ == MLX90632_MEAS_MEDICAL) {
    ESP_LOGI(TAG, "Using Medical measurement mode");
  } else {
    ESP_LOGI(TAG, "Using Extended Range measurement mode");
  }
  
  // Set emissivity
  mlx90632_.setEmissivity(emissivity_);
  
  // Set refresh rate (can be configured)
  // Note: Only set if different from current to avoid excessive EEPROM writes
  mlx90632_refresh_rate_t current_rate = mlx90632_.getRefreshRate();
  if (current_rate != refresh_rate_) {
    if (!mlx90632_.setRefreshRate(refresh_rate_)) {
      ESP_LOGW(TAG, "Failed to set refresh rate");
    } else {
      // Log refresh rate
      uint16_t refresh_ms = 0;
      switch (refresh_rate_) {
        case MLX90632_REFRESH_0_5HZ: refresh_ms = 2000; break;
        case MLX90632_REFRESH_1HZ: refresh_ms = 1000; break;
        case MLX90632_REFRESH_2HZ: refresh_ms = 500; break;
        case MLX90632_REFRESH_4HZ: refresh_ms = 250; break;
        case MLX90632_REFRESH_8HZ: refresh_ms = 125; break;
        case MLX90632_REFRESH_16HZ: refresh_ms = 62; break;
        case MLX90632_REFRESH_32HZ: refresh_ms = 31; break;
        case MLX90632_REFRESH_64HZ: refresh_ms = 16; break;
      }
      ESP_LOGI(TAG, "Refresh rate set to %u ms per measurement", refresh_ms);
    }
  } else {
    ESP_LOGD(TAG, "Refresh rate already correct, skipping EEPROM write");
  }
  
  ESP_LOGI(TAG, "MLX90632 initialized successfully");
  
  uint64_t product_id = mlx90632_.getProductID();
  ESP_LOGI(TAG, "Product ID: 0x%012" PRIx64, product_id);
}

void MLX90632Component::update() {
  ESP_LOGD(TAG, "=== UPDATE CALLED ===");
  if (this->is_failed()) {
    ESP_LOGD(TAG, "Component is failed, returning");
    return;
  }
  
  // Log calibration data at every update
  ESP_LOGD(TAG, "Calibration: P_R=%.6f P_G=%.9f Aa=%.6f Ba=%.9f Ga=%.9f Gb=%.6f Ka=%.6f",
           mlx90632_.P_R, mlx90632_.P_G, mlx90632_.Aa, mlx90632_.Ba, 
           mlx90632_.Ga, mlx90632_.Gb, mlx90632_.Ka);
  
  // Check if new data is available
  if (!mlx90632_.isNewData()) {
    ESP_LOGD(TAG, "No new data available yet");
    return;
  }
  
  ESP_LOGD(TAG, "New data available! Reading temperatures...");
  
  // Read ambient temperature
  double ambient_temp = mlx90632_.getAmbientTemperature();
  ESP_LOGD(TAG, "Ambient temp read: %.2f°C", ambient_temp);
  
  // Read object temperature
  double object_temp = mlx90632_.getObjectTemperature();
  ESP_LOGD(TAG, "Object temp read: %.2f°C", object_temp);
  
  // Publish ambient temperature if sensor is configured
  if (ambient_temperature_sensor_ != nullptr) {
    ambient_temperature_sensor_->publish_state(ambient_temp);
  }
  
  // Publish object temperature if sensor is configured
  if (object_temperature_sensor_ != nullptr) {
    object_temperature_sensor_->publish_state(object_temp);
  }
  
  // Reset new data flag for next measurement
  mlx90632_.resetNewData();
}

void MLX90632Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MLX90632 Temperature Sensor");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  
  if (object_temperature_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Object Temperature", object_temperature_sensor_);
  }
  if (ambient_temperature_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Ambient Temperature", ambient_temperature_sensor_);
  }
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Failed to initialize MLX90632");
  }
}

}  // namespace mlx90632
}  // namespace esphome
