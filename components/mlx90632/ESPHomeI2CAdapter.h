#pragma once

#include "esphome/components/i2c/i2c.h"
#include "I2CInterface.h"

/**
 * @brief ESPHome I2C Adapter for Adafruit libraries
 * 
 * This adapter implements a standard I2C interface using ESPHome's I2C bus,
 * allowing Adafruit libraries (like MLX90632) to work seamlessly with ESPHome
 * without requiring Wire.h or TwoWire.
 */
class ESPHomeI2CAdapter : public I2CInterface {
 public:
  ESPHomeI2CAdapter(esphome::i2c::I2CBus *bus) : bus_(bus) {}

  // I2CInterface methods
  void beginTransmission(uint8_t address) override;
  uint8_t endTransmission(uint8_t sendStop = true) override;
  size_t write(uint8_t data) override;
  size_t write(const uint8_t *data, size_t quantity) override;
  uint8_t requestFrom(uint8_t addr, uint8_t quantity) override;
  int read(void) override;
  int available(void) override;

 private:
  esphome::i2c::I2CBus *bus_;
  uint8_t tx_addr_{0};
  uint8_t tx_buf_[255]{};  // I2C max write size
  uint8_t tx_len_{0};
  uint8_t rx_buf_[255]{};  // I2C max read size
  uint8_t rx_len_{0};
  uint8_t rx_pos_{0};
};
