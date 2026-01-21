#include "ESPHomeI2CAdapter.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mlx90632 {

static const char *TAG = "mlx90632.i2c_adapter";

void ESPHomeI2CAdapter::beginTransmission(uint8_t address) {
  tx_len_ = 0;
}

uint8_t ESPHomeI2CAdapter::endTransmission(uint8_t sendStop) {
  if (!device_) {
    return 1;  // Error
  }
  
  // Use I2CDevice's write_bytes method
  if (device_->write_bytes(i2c_addr_, tx_buf_, tx_len_)) {
    tx_len_ = 0;
    return 0;  // Success
  }
  
  ESP_LOGW(TAG, "I2C write failed for address 0x%02X", i2c_addr_);
  return 1;  // Error
}

size_t ESPHomeI2CAdapter::write(uint8_t data) {
  if (tx_len_ < sizeof(tx_buf_)) {
    tx_buf_[tx_len_++] = data;
    return 1;
  }
  ESP_LOGW(TAG, "I2C transmit buffer full");
  return 0;
}

size_t ESPHomeI2CAdapter::write(const uint8_t *data, size_t quantity) {
  size_t written = 0;
  for (size_t i = 0; i < quantity; i++) {
    if (write(data[i])) {
      written++;
    } else {
      break;
    }
  }
  return written;
}

uint8_t ESPHomeI2CAdapter::requestFrom(uint8_t addr, uint8_t quantity) {
  if (!device_) {
    return 0;
  }
  
  if (quantity > sizeof(rx_buf_)) {
    quantity = sizeof(rx_buf_);
  }
  
  // Use I2CDevice's read_bytes method
  if (device_->read_bytes(i2c_addr_, rx_buf_, quantity)) {
    rx_len_ = quantity;
    rx_pos_ = 0;
    return quantity;
  }
  
  ESP_LOGW(TAG, "I2C read failed for address 0x%02X", i2c_addr_);
  return 0;
}

int ESPHomeI2CAdapter::read(void) {
  if (rx_pos_ < rx_len_) {
    return rx_buf_[rx_pos_++];
  }
  return -1;
}

int ESPHomeI2CAdapter::available(void) {
  return rx_len_ - rx_pos_;
}

}  // namespace mlx90632
}  // namespace esphome

int ESPHomeI2CAdapter::available(void) {
  return (int)(rx_len_ - rx_pos_);
}
