#pragma once

#include "esphome/components/i2c/i2c.h"
#include <Adafruit_BusIO_Register.h>
#include <Wire.h>

/**
 * @brief ESPHome I2C Adapter for Adafruit BusIO
 * 
 * This adapter implements the TwoWire interface using ESPHome's I2C bus,
 * allowing Adafruit libraries (like MLX90632) to work seamlessly with ESPHome.
 */
class ESPHomeI2CAdapter : public TwoWire {
 public:
  ESPHomeI2CAdapter(esphome::i2c::I2CBus *bus) : bus_(bus) {}

  // Wire interface methods
  uint8_t requestFrom(uint8_t addr, uint8_t quantity, uint8_t sendStop = true) override;
  uint8_t requestFrom(int addr, int quantity, int sendStop = true) override {
    return requestFrom((uint8_t)addr, (uint8_t)quantity, (uint8_t)sendStop);
  }

  size_t write(uint8_t data) override;
  size_t write(const uint8_t *data, size_t quantity) override;
  
  int read(void) override;
  int peek(void) override;
  void flush(void) override;
  
  void beginTransmission(uint8_t address) override;
  void beginTransmission(int address) override {
    beginTransmission((uint8_t)address);
  }
  
  uint8_t endTransmission(uint8_t sendStop = true) override;
  uint8_t endTransmission(bool sendStop = true) override {
    return endTransmission((uint8_t)sendStop);
  }
  
  void begin(void) override {}
  void begin(uint8_t address) override {}
  void end(void) override {}
  
  void setClock(uint32_t freq) override {}
  void setWireTimeout(uint32_t timeout = 25000, bool reset_with_timeout = false) override {}

 private:
  esphome::i2c::I2CBus *bus_;
  uint8_t tx_addr_{0};
  uint8_t tx_buf_[32]{};
  uint8_t tx_len_{0};
  uint8_t rx_buf_[32]{};
  uint8_t rx_len_{0};
  uint8_t rx_pos_{0};
};
