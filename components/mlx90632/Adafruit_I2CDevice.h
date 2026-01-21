/*!
 * @file Adafruit_I2CDevice.h
 *
 * Abstraction layer for I2C Device
 */

#ifndef _ADAFRUIT_I2CDEVICE_H
#define _ADAFRUIT_I2CDEVICE_H

#include "Arduino.h"
#include <Wire.h>

#define I2C_DEBUG 0

/*!
 *    @brief  Class that defines an I2C device
 */
class Adafruit_I2CDevice {
public:
  Adafruit_I2CDevice(uint8_t addr, TwoWire *theWire = &Wire);
  ~Adafruit_I2CDevice();

  bool begin(bool addr_lower = true);
  void end(void);

  bool detected(void);

  /*!
   *    @brief  Writes 8-bits to the specified destination register
   *    @param  reg
   *            The register address to write to
   *    @param  buffer
   *            The data buffer to write from
   *    @param  len
   *            The number of bytes to write
   *    @return True if the write was successful, otherwise false
   */
  bool write(const uint8_t *buffer, size_t len, bool stop = true,
             uint8_t *prefix_buffer = NULL, size_t prefix_len = 0);

  /*!
   *    @brief  Reads 8-bits from the specified destination register
   *    @param  buffer
   *            The data buffer to read into
   *    @param  len
   *            The number of bytes to read
   *    @param  stop
   *            Whether to send a STOP signal on the bus
   *    @return True if the read was successful, otherwise false
   */
  bool read(uint8_t *buffer, size_t len, bool stop = true);

  /*!
   *    @brief  Reads 8-bits from the specified destination register
   *    @param  buffer
   *            The data buffer to read into
   *    @param  len
   *            The number of bytes to read
   *    @return True if the read was successful, otherwise false
   */
  bool read_then_write(const uint8_t *write_buffer, size_t write_len,
                       uint8_t *read_buffer, size_t read_len,
                       bool stop = false);

  uint8_t address(void) const { return _addr; }

private:
  uint8_t _addr;
  TwoWire *_wire;
  bool _begun;
};

#endif // _ADAFRUIT_I2CDEVICE_H
