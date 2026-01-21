#pragma once

#include <stdint.h>
#include <cstddef>

/**
 * @brief Abstract I2C interface for device-agnostic I2C communication
 */
class I2CInterface {
 public:
  virtual ~I2CInterface() = default;
  
  /**
   * @brief Begin I2C transmission to address
   */
  virtual void beginTransmission(uint8_t address) = 0;
  
  /**
   * @brief End I2C transmission
   * @return Status code (0 = success)
   */
  virtual uint8_t endTransmission(uint8_t sendStop = true) = 0;
  
  /**
   * @brief Write a byte to the I2C bus (after beginTransmission)
   */
  virtual size_t write(uint8_t data) = 0;
  
  /**
   * @brief Write multiple bytes to the I2C bus
   */
  virtual size_t write(const uint8_t *data, size_t quantity) = 0;
  
  /**
   * @brief Request data from I2C device
   */
  virtual uint8_t requestFrom(uint8_t addr, uint8_t quantity) = 0;
  
  /**
   * @brief Read a byte from the I2C bus (after requestFrom)
   */
  virtual int read(void) = 0;
  
  /**
   * @brief Get number of bytes available to read
   */
  virtual int available(void) = 0;
};
