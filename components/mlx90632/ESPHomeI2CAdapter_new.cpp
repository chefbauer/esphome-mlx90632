#include "ESPHomeI2CAdapter.h"
#include "esphome/core/log.h"

static const char *TAG = "mlx90632.i2c_adapter";

void ESPHomeI2CAdapter::beginTransmission(uint8_t address) {
  tx_addr_ = address;
  tx_len_ = 0;
}

uint8_t ESPHomeI2CAdapter::endTransmission(uint8_t sendStop) {
  if (!bus_) {
    return 1;  // Error
  }
  
  auto err = bus_->writev(tx_addr_, tx_buf_, tx_len_, nullptr, 0);
  tx_len_ = 0;
  
  if (err == esphome::i2c::ERROR_OK) {
    return 0;  // Success
  }
  
  ESP_LOGW(TAG, "I2C write failed: %d", err);
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
  if (!bus_) {
    return 0;
  }
  
  if (quantity > sizeof(rx_buf_)) {
    quantity = sizeof(rx_buf_);
  }
  
  auto err = bus_->readv(addr, nullptr, 0, rx_buf_, quantity);
  
  if (err == esphome::i2c::ERROR_OK) {
    rx_len_ = quantity;
    rx_pos_ = 0;
    return quantity;
  }
  
  ESP_LOGW(TAG, "I2C read failed: %d", err);
  return 0;
}

int ESPHomeI2CAdapter::read(void) {
  if (rx_pos_ < rx_len_) {
    return rx_buf_[rx_pos_++];
  }
  return -1;
}

int ESPHomeI2CAdapter::available(void) {
  return (int)(rx_len_ - rx_pos_);
}
