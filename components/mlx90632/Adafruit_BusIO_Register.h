/*!
 * @file Adafruit_BusIO_Register.h
 *
 * Native ESPHome/ESP-IDF Register-based I2C Communication
 */

#ifndef _ADAFRUIT_BUSIO_REGISTER_H
#define _ADAFRUIT_BUSIO_REGISTER_H

#include "Arduino.h"
#include "Adafruit_I2CDevice.h"
#include <stdint.h>

// Byte order constants (Arduino-compatible)
#define LSBFIRST 0
#define MSBFIRST 1
#define LITTLE_ENDIAN 0
#define BIG_ENDIAN 1

/*!
 *    @brief  Class that defines a register on an I2C device
 */
class Adafruit_BusIO_Register {
public:
  /**
   * @brief Constructor for 8/16/32-bit register access
   * @param i2c_device Pointer to I2CDevice
   * @param reg_addr Register address
   * @param width Register width in bytes
   * @param byteOrder LSBFIRST or MSBFIRST
   * @param regWidth Address width (1 or 2 bytes)
   */
  Adafruit_BusIO_Register(Adafruit_I2CDevice *i2c_device, uint16_t reg_addr,
                          uint8_t width = 1, uint8_t byteOrder = MSBFIRST,
                          uint8_t regWidth = 2);

  /**
   * @brief Constructor for bit-field register access
   * @param i2c_device Pointer to I2CDevice
   * @param reg_addr Register address
   * @param bits Number of bits
   * @param shift Bit shift position
   * @param byteOrder LSBFIRST or MSBFIRST
   * @param regWidth Address width
   */
  Adafruit_BusIO_Register(Adafruit_I2CDevice *i2c_device, uint16_t reg_addr,
                          uint8_t bits, uint8_t shift, uint8_t byteOrder,
                          uint8_t regWidth);

  /**
   * @brief Read register value
   * @return Register value as uint32_t
   */
  uint32_t read();

  /**
   * @brief Write register value
   * @param value Value to write
   * @return True if successful
   */
  bool write(uint32_t value);

  /**
   * @brief Get register address
   * @return Register address
   */
  uint16_t getRegisterAddress() const { return _address; }

private:
  Adafruit_I2CDevice *_i2cDevice;
  uint16_t _address;
  uint8_t _width;
  uint8_t _byteOrder;
  uint8_t _regWidth;
  uint8_t _shift = 0; // For bit-field access
};

/*!
 *    @brief  Class for accessing individual bits/fields in a register
 */
class Adafruit_BusIO_RegisterBits {
public:
  /**
   * @brief Constructor
   * @param register_object Pointer to register
   * @param bits Number of bits
   * @param shift Bit shift position
   */
  Adafruit_BusIO_RegisterBits(Adafruit_BusIO_Register *register_object,
                              uint8_t bits, uint8_t shift);

  /**
   * @brief Read bits from register
   * @return Bit field value
   */
  uint32_t read();

  /**
   * @brief Write bits to register
   * @param bits Value to write
   * @return True if successful
   */
  bool write(uint32_t bits);

private:
  Adafruit_BusIO_Register *_register;
  uint8_t _bits;
  uint8_t _shift;
};

#endif // _ADAFRUIT_BUSIO_REGISTER_H
