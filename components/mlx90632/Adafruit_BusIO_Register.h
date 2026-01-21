/*!
 * @file Adafruit_BusIO_Register.h
 *
 * Abstraction layer for register-based IO
 */

#ifndef _ADAFRUIT_BUSIO_REGISTER_H
#define _ADAFRUIT_BUSIO_REGISTER_H

#include "Arduino.h"
#include "Adafruit_I2CDevice.h"

#define REGISTER_WRITE_MASK 0x0
#define REGISTER_READ_MASK 0xFF

/*!
 *    @brief  Class that defines a register on an I2C device
 */
class Adafruit_BusIO_Register {
public:
  Adafruit_BusIO_Register(Adafruit_I2CDevice *i2c_dev, uint16_t reg_addr,
                          uint8_t width = 1, uint8_t byteorder = LITTLE_ENDIAN,
                          uint8_t address_width = 1);

  bool read(uint8_t *buffer, uint8_t len = 1);
  bool read(uint8_t *value);
  uint32_t read(void);

  bool write(uint8_t *buffer, uint8_t len, bool bulk_write = false);
  bool write(uint8_t value);
  bool write(uint32_t value, uint8_t numbytes = 4);

  uint16_t getRegisterAddress(void) const { return _register; }
  void setRegisterAddress(uint16_t reg_addr) { _register = reg_addr; }

private:
  Adafruit_I2CDevice *_i2c_dev;
  uint16_t _register;
  uint8_t _width;
  uint8_t _byteorder;
  uint8_t _address_width;
};

#endif // _ADAFRUIT_BUSIO_REGISTER_H
