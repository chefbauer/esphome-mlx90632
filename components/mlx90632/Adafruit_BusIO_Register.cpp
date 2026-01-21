/*!
 * @file Adafruit_BusIO_Register.cpp
 *
 * Register-based I2C Communication for ESP-IDF
 */

#include "Adafruit_BusIO_Register.h"
#include "esphome/core/log.h"
#include <string.h>

static const char* TAG = "mlx90632";

/*!
 *    @brief  Instantiates a new Adafruit_BusIO_Register class
 *    @param  i2c_device Pointer to I2CDevice to use for register access
 *    @param  reg_addr Address of the register
 *    @param  width Register width in bytes (1, 2, or 4)
 *    @param  byteOrder Byte order (LSBFIRST or MSBFIRST)
 *    @param  regWidth Internal register width in bytes
 */
Adafruit_BusIO_Register::Adafruit_BusIO_Register(
    Adafruit_I2CDevice *i2c_device, uint16_t reg_addr, uint8_t width,
    uint8_t byteOrder, uint8_t regWidth)
    : _i2cDevice(i2c_device), _address(reg_addr), _width(width),
      _byteOrder(byteOrder), _regWidth(regWidth) {}

/*!
 *    @brief  Instantiates a new Adafruit_BusIO_Register class for sub-register access
 *    @param  i2c_device Pointer to I2CDevice
 *    @param  reg_addr Address of the register
 *    @param  bits Number of bits in sub-register
 *    @param  shift Bit shift position
 *    @param  byteOrder Byte order
 *    @param  regWidth Internal register width
 */
Adafruit_BusIO_Register::Adafruit_BusIO_Register(
    Adafruit_I2CDevice *i2c_device, uint16_t reg_addr, uint8_t bits,
    uint8_t shift, uint8_t byteOrder, uint8_t regWidth)
    : _i2cDevice(i2c_device), _address(reg_addr), _width(bits), _shift(shift),
      _byteOrder(byteOrder), _regWidth(regWidth) {}

/*!
 *    @brief  Read register value
 *    @return The value read from the register
 */
uint32_t Adafruit_BusIO_Register::read() {
  uint8_t buffer[4] = {0};
  uint8_t addr_buffer[2];

  // Convert register address to big-endian format
  if (_regWidth == 2) {
    addr_buffer[0] = (_address >> 8) & 0xFF;  // MSB first
    addr_buffer[1] = _address & 0xFF;         // LSB second
  } else {
    addr_buffer[0] = _address & 0xFF;
  }

  ESP_LOGD(TAG, "Reading reg 0x%04X", _address);

  // Use Adafruit_I2CDevice's read method which will handle the write-read transaction
  if (!_i2cDevice->read_then_write(addr_buffer, _regWidth, buffer, _width)) {
    ESP_LOGW(TAG, "Read failed for reg 0x%04X", _address);
    return 0;
  }

  // Log raw bytes for ALL reads
  ESP_LOGD(TAG, "Reg 0x%04X: [0x%02X 0x%02X 0x%02X 0x%02X]", 
           _address, buffer[0], buffer[1], buffer[2], buffer[3]);

  // Convert bytes to 32-bit value based on byte order
  uint32_t value = 0;

  if (_byteOrder == MSBFIRST) {
    // Most significant byte first
    for (int i = 0; i < _width; i++) {
      value = (value << 8) | buffer[i];
    }
  } else {
    // Least significant byte first
    for (int i = _width - 1; i >= 0; i--) {
      value = (value << 8) | buffer[i];
    }
  }

  // Apply bit shift if this is a bit-register
  if (_shift > 0) {
    value = (value >> _shift) & ((1 << _width) - 1);
  }

  return value;
}

/*!
 *    @brief  Write register value
 *    @param  value Value to write
 *    @return True if write successful, false otherwise
 */
bool Adafruit_BusIO_Register::write(uint32_t value) {
  uint8_t buffer[4 + 2]; // Data + potential register address
  uint8_t pos = 0;

  // Add register address
  uint16_t addr = _address;
  if (_regWidth == 2) {
    buffer[pos++] = (addr >> 8) & 0xFF; // MSB first for register address
    buffer[pos++] = addr & 0xFF;
  } else if (_regWidth == 1) {
    buffer[pos++] = addr & 0xFF;
  }

  // Handle bit shift for sub-registers
  if (_shift > 0) {
    value = (value & ((1 << _width) - 1)) << _shift;
  }

  // Convert value to bytes
  if (_byteOrder == MSBFIRST) {
    // Most significant byte first
    for (int i = _width - 1; i >= 0; i--) {
      buffer[pos++] = (value >> (i * 8)) & 0xFF;
    }
  } else {
    // Least significant byte first
    for (int i = 0; i < _width; i++) {
      buffer[pos++] = (value >> (i * 8)) & 0xFF;
    }
  }

  // Write to I2C device
  return _i2cDevice->write(buffer, pos);
}

/*!
 *    @brief  Instantiates a new Adafruit_BusIO_RegisterBits class
 *    @param  register_object Pointer to register containing the bits
 *    @param  bits Number of bits in the field
 *    @param  shift Bit shift position
 */
Adafruit_BusIO_RegisterBits::Adafruit_BusIO_RegisterBits(
    Adafruit_BusIO_Register *register_object, uint8_t bits, uint8_t shift)
    : _register(register_object), _bits(bits), _shift(shift) {}

/*!
 *    @brief  Read register bits
 *    @return Value of the bits
 */
uint32_t Adafruit_BusIO_RegisterBits::read() {
  uint32_t val = _register->read();
  val >>= _shift;
  return val & ((1 << _bits) - 1);
}

/*!
 *    @brief  Write register bits
 *    @param  bits Value to write
 *    @return True if write successful, false otherwise
 */
bool Adafruit_BusIO_RegisterBits::write(uint32_t bits) {
  uint32_t val = _register->read();

  // Clear the bits we're about to write
  val &= ~(((1 << _bits) - 1) << _shift);

  // Set the new bits
  val |= (bits & ((1 << _bits) - 1)) << _shift;

  return _register->write(val);
}
