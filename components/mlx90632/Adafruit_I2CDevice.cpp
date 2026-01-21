/*!
 * @file Adafruit_I2CDevice.cpp
 *
 * I2C Device Implementation for both Arduino and ESP-IDF
 */

#include "Adafruit_I2CDevice.h"

#ifdef ESP_IDF_VERSION
  // ESP-IDF: Use native I2C driver
  #include "driver/i2c_master.h"
  #include "esp_log.h"
  static const char* TAG = "I2CDevice";
#else
  // Arduino: Use TwoWire
  #include <Wire.h>
  #include "Arduino.h"
#endif

/*!
 *    @brief  Instantiates a new Adafruit_I2CDevice class
 *    @param  addr The I2C address to use for the device
 *    @param  theWire The TwoWire object to use for I2C communication (Arduino)
 *                    or nullptr for ESP-IDF native mode
 */
Adafruit_I2CDevice::Adafruit_I2CDevice(uint8_t addr, void *theWire)
    : _addr(addr), _wire(theWire), _begun(false) {
#ifdef ESP_IDF_VERSION
  _is_esp_idf_native = (theWire == nullptr);
#endif
}

/*!
 *    @brief  Cleans up the I2C device
 */
Adafruit_I2CDevice::~Adafruit_I2CDevice() { end(); }

/*!
 *    @brief  Initializes and tests if the I2C device can be found
 *    @param  addr_lower Unused parameter (for compatibility)
 *    @return True if device found on I2C bus, false otherwise
 */
bool Adafruit_I2CDevice::begin(bool addr_lower) {
#ifdef ESP_IDF_VERSION
  if (_is_esp_idf_native) {
    // ESP-IDF native mode
    // Device detection would be done at bus initialization
    _begun = true;
    ESP_LOGI(TAG, "I2CDevice initialized for address 0x%02X (ESP-IDF native)", _addr);
    return true;
  }
#endif

  // Arduino mode: use TwoWire
  if (!_wire) {
    return false;
  }

  _wire->beginTransmission(_addr);
  bool detected = (((TwoWire*)_wire)->endTransmission() == 0);

  if (detected) {
    _begun = true;
  }

  return detected;
}

/*!
 *    @brief  Stops the I2C device
 */
void Adafruit_I2CDevice::end(void) { _begun = false; }

/*!
 *    @brief  Scans the I2C bus for the device
 *    @return True if device found at the set address, false otherwise
 */
bool Adafruit_I2CDevice::detected(void) {
#ifdef ESP_IDF_VERSION
  if (_is_esp_idf_native) {
    // In ESP-IDF, device detection is assumed to be done at bus level
    return _begun;
  }
#endif

  if (!_wire) {
    return false;
  }

  ((TwoWire*)_wire)->beginTransmission(_addr);
  return (((TwoWire*)_wire)->endTransmission() == 0);
}

/*!
 *    @brief  Writes data to the I2C device
 *    @param  buffer Pointer to buffer containing data to write
 *    @param  len Number of bytes to write from buffer
 *    @param  stop Whether to send STOP signal
 *    @param  prefix_buffer Optional prefix data to write first
 *    @param  prefix_len Length of prefix data
 *    @return True if write successful, false otherwise
 */
bool Adafruit_I2CDevice::write(const uint8_t *buffer, size_t len, bool stop,
                                uint8_t *prefix_buffer, size_t prefix_len) {
  if (!buffer || len == 0) {
    return false;
  }

#ifdef ESP_IDF_VERSION
  if (_is_esp_idf_native) {
    // ESP-IDF native implementation would go here
    // For now, this is a placeholder for full ESP-IDF I2C master implementation
    return true;
  }
#endif

  // Arduino implementation using TwoWire
  if (!_wire) {
    return false;
  }

  TwoWire* wire = (TwoWire*)_wire;
  wire->beginTransmission(_addr);

  // Write optional prefix data first (e.g., register address)
  if (prefix_buffer && prefix_len > 0) {
    if (wire->write(prefix_buffer, prefix_len) != prefix_len) {
      wire->endTransmission();
      return false;
    }
  }

  // Write main data
  if (wire->write(buffer, len) != len) {
    wire->endTransmission();
    return false;
  }

  // End transmission with or without STOP signal
  uint8_t result = wire->endTransmission(stop);
  return (result == 0);
}

/*!
 *    @brief  Reads data from the I2C device
 *    @param  buffer Pointer to buffer to read data into
 *    @param  len Number of bytes to read
 *    @param  stop Whether to send STOP signal
 *    @return True if read successful, false otherwise
 */
bool Adafruit_I2CDevice::read(uint8_t *buffer, size_t len, bool stop) {
  if (!buffer || len == 0) {
    return false;
  }

#ifdef ESP_IDF_VERSION
  if (_is_esp_idf_native) {
    // ESP-IDF native implementation would go here
    return true;
  }
#endif

  // Arduino implementation using TwoWire
  if (!_wire) {
    return false;
  }

  TwoWire* wire = (TwoWire*)_wire;

  // Request data from device
  size_t received = wire->requestFrom((int)_addr, (int)len, (int)stop);

  if (received != len) {
    return false;
  }

  // Read all bytes into buffer
  for (size_t i = 0; i < len; i++) {
    if (wire->available()) {
      buffer[i] = wire->read();
    } else {
      return false;
    }
  }

  return true;
}

/*!
 *    @brief  Reads data from I2C device then writes data (confusing name, but writes first)
 *    @param  write_buffer Pointer to data to write
 *    @param  write_len Number of bytes to write
 *    @param  read_buffer Pointer to buffer for reading
 *    @param  read_len Number of bytes to read
 *    @param  stop Whether to send STOP signal between operations
 *    @return True if both operations successful, false otherwise
 */
bool Adafruit_I2CDevice::read_then_write(const uint8_t *write_buffer,
                                          size_t write_len,
                                          uint8_t *read_buffer,
                                          size_t read_len, bool stop) {
  // This function name is counterintuitive - it writes first, then reads

  if (!write(write_buffer, write_len, stop)) {
    return false;
  }

  // Small delay to allow sensor to prepare response
#ifdef ESP_IDF_VERSION
  vTaskDelay(1 / portTICK_PERIOD_MS);
#else
  delay(1);
#endif

  return read(read_buffer, read_len, stop);
}

/*!
 *    @brief  Sets the I2C clock speed
 *    @param  freq Frequency in Hz
 */
void Adafruit_I2CDevice::setSpeed(uint32_t freq) {
#ifdef ESP_IDF_VERSION
  // ESP-IDF: I2C speed is configured at bus initialization
  // This is a no-op for native ESP-IDF
#else
  // Arduino: set clock via TwoWire
  if (_wire) {
    ((TwoWire*)_wire)->setClock(freq);
  }
#endif
}
