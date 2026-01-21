/*!
 * @file Adafruit_I2CDevice.h
 *
 * Native I2C Device Implementation for ESP-IDF with TwoWire compatibility
 */

#ifndef _ADAFRUIT_I2CDEVICE_H
#define _ADAFRUIT_I2CDEVICE_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define I2C_TIMEOUT_MS 1000

// Include Wire.h if available (ESPHome includes this)
#ifdef __has_include
  #if __has_include(<Wire.h>)
    #include <Wire.h>
  #else
    // Forward declaration for environments without Wire.h
    class TwoWire;
  #endif
#else
  // Forward declaration - TwoWire is provided by ESPHome/Arduino environment
  class TwoWire;
#endif
class Adafruit_I2CDevice {
public:
  /**
   * @brief Constructor for Adafruit_I2CDevice
   * @param addr The I2C address of the device
   * @param theWire Pointer to TwoWire object (ESPHome compatible)
   */
  Adafruit_I2CDevice(uint8_t addr, void *theWire = nullptr);
  ~Adafruit_I2CDevice();

  /**
   * @brief Initialize I2C device
   * @param addr_lower Unused parameter (for compatibility)
   * @return True if device found on I2C bus, false otherwise
   */
  bool begin(bool addr_lower = true);

  /**
   * @brief Stop I2C device
   */
  void end(void);

  /**
   * @brief Scan for device on I2C bus
   * @return True if device found at specified address
   */
  bool detected(void);

  /**
   * @brief Write data to I2C device
   * @param buffer Pointer to data buffer
   * @param len Number of bytes to write
   * @param stop Whether to send STOP signal
   * @param prefix_buffer Optional prefix (register address)
   * @param prefix_len Length of prefix
   * @return True if write successful
   */
  bool write(const uint8_t *buffer, size_t len, bool stop = true,
             uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);

  /**
   * @brief Read data from I2C device
   * @param buffer Pointer to data buffer for reading into
   * @param len Number of bytes to read
   * @param stop Whether to send STOP signal
   * @return True if read successful
   */
  bool read(uint8_t *buffer, size_t len, bool stop = true);

  /**
   * @brief Write to device then read from it
   * @param write_buffer Pointer to write data
   * @param write_len Length of write data
   * @param read_buffer Pointer to read buffer
   * @param read_len Length of read data
   * @param stop Whether to send STOP signal
   * @return True if operation successful
   */
  bool read_then_write(const uint8_t *write_buffer, size_t write_len,
                       uint8_t *read_buffer, size_t read_len,
                       bool stop = false);

  /**
   * @brief Set the I2C clock speed
   * @param freq Frequency in Hz
   */
  void setSpeed(uint32_t freq);

  /**
   * @brief Get I2C address
   * @return Device I2C address
   */
  uint8_t address(void) const { return _addr; }

private:
  uint8_t _addr;
  void *_wire;  // TwoWire* (ESPHome I2C interface)
  bool _begun;
};

#endif // _ADAFRUIT_I2CDEVICE_H
