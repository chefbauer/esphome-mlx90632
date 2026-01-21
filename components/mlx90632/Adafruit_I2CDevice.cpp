/*!
 * @file Adafruit_I2CDevice.cpp
 *
 * I2C Device Implementation for ESPHome with I2CInterface abstraction
 */

#include "Adafruit_I2CDevice.h"
#include "I2CInterface.h"

// Delay function for ESP-IDF/ESPHome
#if defined(delay)
  // Already defined (Arduino-compatible)
#else
  #include <unistd.h>
  #define delay(ms) usleep((ms) * 1000)
#endif

/*!
 *    @brief  Instantiates a new Adafruit_I2CDevice class
 *    @param  addr The I2C address to use for the device
 *    @param  theWire The I2CInterface object to use for I2C communication
 */
Adafruit_I2CDevice::Adafruit_I2CDevice(uint8_t addr, void *theWire)
    : _addr(addr), _wire(theWire), _begun(false) {}

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
  if (!_wire) {
    return false;
  }

  I2CInterface* wire = (I2CInterface*)_wire;
  wire->beginTransmission(_addr);
  bool detected = (wire->endTransmission() == 0);

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
  if (!_wire) {
    return false;
  }

  I2CInterface* wire = (I2CInterface*)_wire;
  wire->beginTransmission(_addr);
  return (wire->endTransmission() == 0);
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
  if (!_wire || !buffer || len == 0) {
    return false;
  }

  I2CInterface* wire = (I2CInterface*)_wire;
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
  if (!_wire || !buffer || len == 0) {
    return false;
  }

  I2CInterface* wire = (I2CInterface*)_wire;

  // Request data from device
  uint8_t received = wire->requestFrom(_addr, len);

  if (received != len) {
    return false;
  }

  // Read all bytes into buffer
  for (size_t i = 0; i < len; i++) {
    if (wire->available() > 0) {
      buffer[i] = wire->read();
    } else {
      return false;
    }
  }

  return true;
}

/*!
 *    @brief  Reads data from I2C device then writes data (writes first, then reads)
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
  if (!_wire || !write_buffer || !read_buffer || write_len == 0 || read_len == 0) {
    return false;
  }

  I2CInterface* wire = (I2CInterface*)_wire;
  
  // Write register address then read data (combined I2C transaction)
  wire->beginTransmission(_addr);
  
  // Write register address
  if (wire->write(write_buffer, write_len) != write_len) {
    wire->endTransmission();
    return false;
  }
  
  // End transmission but don't send STOP (repeated START)
  wire->endTransmission(false);
  
  // Request and read data
  uint8_t received = wire->requestFrom(_addr, read_len);
  if (received != read_len) {
    return false;
  }
  
  for (size_t i = 0; i < read_len; i++) {
    if (wire->available() > 0) {
      read_buffer[i] = wire->read();
    } else {
      return false;
    }
  }
  
  return true;
}

/*!
 *    @brief  Sets the I2C clock speed
 *    @param  freq Frequency in Hz
 */
void Adafruit_I2CDevice::setSpeed(uint32_t freq) {
  // I2CInterface doesn't expose clock setting - ESPHome handles this
  // This is a no-op for compatibility
}
