/*!
 * @file Adafruit_I2CDevice.h
 *
 * Native I2C Device Implementation for both Arduino and ESP-IDF
 */

#ifndef _ADAFRUIT_I2CDEVICE_H
#define _ADAFRUIT_I2CDEVICE_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// Platform detection: check for ESP-IDF first (more specific)
#if defined(ESP_IDF_VERSION) || defined(ESP_PLATFORM) || defined(CONFIG_IDF_TARGET)
  // ESP-IDF platform
  #define USING_ESP_IDF 1
  #define I2C_TIMEOUT_MS 1000
#else
  // Arduino platform
  #define USING_ESP_IDF 0
  #include "Arduino.h"
  #include <Wire.h>
  #define I2C_TIMEOUT_MS 1000
#endif

#define I2C_DEBUG 0

/*!
 *    @brief  Class that defines an I2C device - Compatible with both Arduino and ESP-IDF
 */
class Adafruit_I2CDevice {
public:
  /**
   * @brief Constructor for Adafruit_I2CDevice
   * @param addr The I2C address of the device
   * @param theWire Pointer to TwoWire object (Arduino) or can be NULL for ESP-IDF native
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
  void *_wire;  // Can be TwoWire* (Arduino) or i2c_master_bus_handle_t (ESP-IDF)
  bool _begun;

#ifdef ESP_IDF_VERSION
  // ESP-IDF specific
  bool _is_esp_idf_native = false;
#endif
};

#endif // _ADAFRUIT_I2CDEVICE_H
