/*!
 *  @file Adafruit_MLX90632.cpp
 *
 * 	I2C Driver for MLX90632 Far Infrared Temperature Sensor
 *
 * 	This is a library for the Adafruit MLX90632 breakout:
 * 	http://www.adafruit.com/products
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  Written by Limor "Ladyada" Fried with assistance from Claude Code.
 *
 *	MIT license, see LICENSE for more information
 */

#include "Adafruit_MLX90632.h"
#include "I2CInterface.h"
#include <math.h>
#include <unistd.h>

// ESP-IDF logging
#include "esp_log.h"

// Define delay if not already defined
#ifndef delay
  #define delay(ms) usleep((ms) * 1000)
#endif

// #define MLX90632_DEBUG

/*!
 *    @brief  Instantiates a new MLX90632 class
 */
Adafruit_MLX90632::Adafruit_MLX90632() {
  TO0 = 25.0;
  TA0 = 25.0;
  i2c_dev = nullptr;
}

/*!
 *    @brief  Cleans up the MLX90632
 */
Adafruit_MLX90632::~Adafruit_MLX90632() {
  if (i2c_dev) {
    delete i2c_dev;
  }
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address The I2C address to be used.
 *    @param  i2c_interface The I2CInterface object to be used for I2C communications.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MLX90632::begin(uint8_t i2c_address, I2CInterface* i2c_interface) {
  if (i2c_dev) {
    delete i2c_dev;
  }
  if (i2c_interface) {
    i2c_dev = new Adafruit_I2CDevice(i2c_address, (void*)i2c_interface);
  } else {
    return false;  // No I2C interface provided
  }

  if (!i2c_dev->begin()) {
    return false;
  }

  // Check product code
  Adafruit_BusIO_Register product_code_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_PRODUCT_CODE), 2, MSBFIRST, 2);
  uint16_t product_code = product_code_reg.read();

  if (product_code == 0xFFFF || product_code == 0x0000) {
    return false;
  }

  // Load calibration constants automatically
  if (!getCalibrations()) {
    return false;
  }

  return true;
}

/*!
 *    @brief  Read the 48-bit product ID
 *    @return Product ID (48-bit value in uint64_t)
 */
uint64_t Adafruit_MLX90632::getProductID() {
  Adafruit_BusIO_Register id0_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_ID0), 2, MSBFIRST, 2);
  Adafruit_BusIO_Register id1_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_ID1), 2, MSBFIRST, 2);
  Adafruit_BusIO_Register id2_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_ID2), 2, MSBFIRST, 2);

  uint16_t id0 = id0_reg.read();
  uint16_t id1 = id1_reg.read();
  uint16_t id2 = id2_reg.read();

  return ((uint64_t)id2 << 32) | ((uint64_t)id1 << 16) | id0;
}

/*!
 *    @brief  Read the product code
 *    @return Product code (16-bit value)
 */
uint16_t Adafruit_MLX90632::getProductCode() {
  Adafruit_BusIO_Register product_code_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_PRODUCT_CODE), 2, MSBFIRST, 2);
  return product_code_reg.read();
}

/*!
 *    @brief  Read the EEPROM version
 *    @return EEPROM version (16-bit value)
 */
uint16_t Adafruit_MLX90632::getEEPROMVersion() {
  Adafruit_BusIO_Register version_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_VERSION), 2, MSBFIRST, 2);
  return version_reg.read();
}

/*!
 *    @brief  Start a single measurement (SOC)
 *    @return True if write succeeded, false otherwise
 */
bool Adafruit_MLX90632::startSingleMeasurement() {
  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_CONTROL), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits soc_bit =
      Adafruit_BusIO_RegisterBits(&control_reg, 1, 3);

  return soc_bit.write(1);
}

/*!
 *    @brief  Start a full measurement table (SOB)
 *    @return True if write succeeded, false otherwise
 */
bool Adafruit_MLX90632::startFullMeasurement() {
  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_CONTROL), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits sob_bit =
      Adafruit_BusIO_RegisterBits(&control_reg, 1, 4);

  return sob_bit.write(1);
}

/*!
 *    @brief  Set the measurement mode
 *    @param  mode The measurement mode to set
 *    @return True if write succeeded, false otherwise
 */
bool Adafruit_MLX90632::setMode(mlx90632_mode_t mode) {
  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_CONTROL), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits mode_bits =
      Adafruit_BusIO_RegisterBits(&control_reg, 2, 1);

  return mode_bits.write((uint8_t)mode);
}

/*!
 *    @brief  Get the measurement mode
 *    @return The current measurement mode
 */
mlx90632_mode_t Adafruit_MLX90632::getMode() {
  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_CONTROL), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits mode_bits =
      Adafruit_BusIO_RegisterBits(&control_reg, 2, 1);

  return (mlx90632_mode_t)mode_bits.read();
}

/*!
 *    @brief  Set the measurement select type
 *    @param  meas_select The measurement select type to set
 *    @return True if write succeeded, false otherwise
 */
bool Adafruit_MLX90632::setMeasurementSelect(mlx90632_meas_select_t meas_select) {
  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_CONTROL), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits meas_select_bits =
      Adafruit_BusIO_RegisterBits(&control_reg, 5, 4);

  return meas_select_bits.write((uint8_t)meas_select);
}

/*!
 *    @brief  Get the measurement select type
 *    @return The current measurement select type
 */
mlx90632_meas_select_t Adafruit_MLX90632::getMeasurementSelect() {
  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_CONTROL), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits meas_select_bits =
      Adafruit_BusIO_RegisterBits(&control_reg, 5, 4);

  return (mlx90632_meas_select_t)meas_select_bits.read();
}

/*!
 *    @brief  Check if device is busy with measurement
 *    @return True if device is busy, false otherwise
 */
bool Adafruit_MLX90632::isBusy() {
  Adafruit_BusIO_Register status_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_STATUS), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits device_busy_bit =
      Adafruit_BusIO_RegisterBits(&status_reg, 1, 10);

  return device_busy_bit.read();
}

/*!
 *    @brief  Check if EEPROM is busy
 *    @return True if EEPROM is busy, false otherwise
 */
bool Adafruit_MLX90632::isEEPROMBusy() {
  Adafruit_BusIO_Register status_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_STATUS), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits ee_busy_bit =
      Adafruit_BusIO_RegisterBits(&status_reg, 1, 9);

  return ee_busy_bit.read();
}

/*!
 *    @brief  Reset device using addressed reset command
 *    @return True if reset succeeded, false otherwise
 */
bool Adafruit_MLX90632::reset() {
  // Send addressed reset command: 0x3005, 0x0006
  uint8_t reset_cmd[] = {0x30, 0x05, 0x00, 0x06};
  if (!i2c_dev->write(reset_cmd, 4)) {
    return false;
  }

  // Wait for reset to complete (at least 150us as per datasheet)
  delay(1);

  return true;
}

/*!
 *    @brief  Read the cycle position
 *    @return Current cycle position (0-31)
 */
uint8_t Adafruit_MLX90632::readCyclePosition() {
  Adafruit_BusIO_Register status_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_STATUS), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits cycle_position_bits =
      Adafruit_BusIO_RegisterBits(&status_reg, 5, 2);

  return cycle_position_bits.read();
}

/*!
 *    @brief  Reset the new data flag to 0
 *    @return True if write succeeded, false otherwise
 */
bool Adafruit_MLX90632::resetNewData() {
  Adafruit_BusIO_Register status_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_STATUS), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits new_data_bit =
      Adafruit_BusIO_RegisterBits(&status_reg, 1, 0);

  return new_data_bit.write(0);
}

/*!
 *    @brief  Check if new data is available
 *    @return True if new data is available, false otherwise
 */
bool Adafruit_MLX90632::isNewData() {
  Adafruit_BusIO_Register status_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_STATUS), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits new_data_bit =
      Adafruit_BusIO_RegisterBits(&status_reg, 1, 0);

  return new_data_bit.read();
}

/*!
 *    @brief  Set the refresh rate for both measurement registers
 *    @param  refresh_rate The refresh rate to set
 *    @return True if both writes succeeded, false otherwise
 */
bool Adafruit_MLX90632::setRefreshRate(mlx90632_refresh_rate_t refresh_rate) {
  Adafruit_BusIO_Register meas1_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_MEAS_1), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits meas1_rate_bits =
      Adafruit_BusIO_RegisterBits(&meas1_reg, 3, 7);

  Adafruit_BusIO_Register meas2_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_MEAS_2), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits meas2_rate_bits =
      Adafruit_BusIO_RegisterBits(&meas2_reg, 3, 7);

  return meas1_rate_bits.write((uint8_t)refresh_rate) &&
         meas2_rate_bits.write((uint8_t)refresh_rate);
}

/*!
 *    @brief  Get the refresh rate from EE_MEAS_1 register
 *    @return The current refresh rate
 */
mlx90632_refresh_rate_t Adafruit_MLX90632::getRefreshRate() {
  Adafruit_BusIO_Register meas1_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_MEAS_1), 2, MSBFIRST, 2);
  Adafruit_BusIO_RegisterBits meas1_rate_bits =
      Adafruit_BusIO_RegisterBits(&meas1_reg, 3, 7);

  return (mlx90632_refresh_rate_t)meas1_rate_bits.read();
}

/*!
 *    @brief  Helper function to read 32-bit values from consecutive registers
 *    @param  lsw_addr Address of the least significant word register
 *    @return 32-bit value (LSW + MSW)
 */
uint32_t Adafruit_MLX90632::read32BitRegister(uint16_t lsw_addr) {
  Adafruit_BusIO_Register lsw_reg =
      Adafruit_BusIO_Register(i2c_dev, swapBytes(lsw_addr), 2, MSBFIRST, 2);
  Adafruit_BusIO_Register msw_reg =
      Adafruit_BusIO_Register(i2c_dev, swapBytes(lsw_addr + 1), 2, MSBFIRST, 2);

  uint16_t lsw = lsw_reg.read();
  uint16_t msw = msw_reg.read();

  return ((uint32_t)msw << 16) | lsw;
}

/*!
 *    @brief  Read all calibration constants from EEPROM
 *    @return True if all reads succeeded, false otherwise
 */
bool Adafruit_MLX90632::getCalibrations() {
  // Read 32-bit calibration constants
  uint32_t ee_p_r = read32BitRegister(MLX90632_REG_EE_P_R_LSW);
  uint32_t ee_p_g = read32BitRegister(MLX90632_REG_EE_P_G_LSW);
  uint32_t ee_p_t = read32BitRegister(MLX90632_REG_EE_P_T_LSW);
  uint32_t ee_p_o = read32BitRegister(MLX90632_REG_EE_P_O_LSW);
  uint32_t ee_aa = read32BitRegister(MLX90632_REG_EE_AA_LSW);
  uint32_t ee_ab = read32BitRegister(MLX90632_REG_EE_AB_LSW);
  uint32_t ee_ba = read32BitRegister(MLX90632_REG_EE_BA_LSW);
  uint32_t ee_bb = read32BitRegister(MLX90632_REG_EE_BB_LSW);
  uint32_t ee_ca = read32BitRegister(MLX90632_REG_EE_CA_LSW);
  uint32_t ee_cb = read32BitRegister(MLX90632_REG_EE_CB_LSW);
  uint32_t ee_da = read32BitRegister(MLX90632_REG_EE_DA_LSW);
  uint32_t ee_db = read32BitRegister(MLX90632_REG_EE_DB_LSW);
  uint32_t ee_ea = read32BitRegister(MLX90632_REG_EE_EA_LSW);
  uint32_t ee_eb = read32BitRegister(MLX90632_REG_EE_EB_LSW);
  uint32_t ee_fa = read32BitRegister(MLX90632_REG_EE_FA_LSW);
  uint32_t ee_fb = read32BitRegister(MLX90632_REG_EE_FB_LSW);
  uint32_t ee_ga = read32BitRegister(MLX90632_REG_EE_GA_LSW);

  // Read 16-bit calibration constants
  Adafruit_BusIO_Register gb_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_GB), 2, MSBFIRST, 2);
  Adafruit_BusIO_Register ka_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_KA), 2, MSBFIRST, 2);
  Adafruit_BusIO_Register kb_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_KB), 2, MSBFIRST, 2);
  Adafruit_BusIO_Register ha_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_HA), 2, MSBFIRST, 2);
  Adafruit_BusIO_Register hb_reg = Adafruit_BusIO_Register(
      i2c_dev, swapBytes(MLX90632_REG_EE_HB), 2, MSBFIRST, 2);

  // Convert to proper double values with scaling factors from datasheet
  P_R = (double)(int32_t)ee_p_r * pow(2, -8);      // 2^-8
  P_G = (double)(int32_t)ee_p_g * pow(2, -20);     // 2^-20
  P_T = (double)(int32_t)ee_p_t * pow(2, -44);     // 2^-44
  P_O = (double)(int32_t)ee_p_o * pow(2, -8);      // 2^-8
  Aa = (double)(int32_t)ee_aa * pow(2, -16);       // 2^-16
  Ab = (double)(int32_t)ee_ab * pow(2, -16);       // 2^-16
  Ba = (double)(int32_t)ee_ba * pow(2, -32);       // 2^-32
  Bb = (double)(int32_t)ee_bb * pow(2, -32);       // 2^-32
  Ca = (double)(int32_t)ee_ca * pow(2, -28);       // 2^-28
  Cb = (double)(int32_t)ee_cb * pow(2, -32);       // 2^-32
  Da = (double)(int32_t)ee_da * pow(2, -22);       // 2^-22
  Db = (double)(int32_t)ee_db * pow(2, -22);       // 2^-22
  Ea = (double)(int32_t)ee_ea * pow(2, -20);       // 2^-20
  Eb = (double)(int32_t)ee_eb * pow(2, -20);       // 2^-20
  Fa = (double)(int32_t)ee_fa * pow(2, -12);       // 2^-12
  Fb = (double)(int32_t)ee_fb * pow(2, -10);       // 2^-10
  Ga = (double)(int32_t)ee_ga * pow(2, -20);       // 2^-20

  Gb = (double)(int16_t)gb_reg.read() * pow(2, -10); // 2^-10
  Ka = (double)(int16_t)ka_reg.read() * pow(2, -8);  // 2^-8
  Kb = (int16_t)kb_reg.read();
  Ha = (double)(int16_t)ha_reg.read() * pow(2, -10); // 2^-10
  Hb = (double)(int16_t)hb_reg.read() * pow(2, -10); // 2^-10

  return true;
}

/*!
 *    @brief  Calculate ambient temperature
 *    @return Ambient temperature in degrees Celsius
 */
double Adafruit_MLX90632::getAmbientTemperature() {
  // Check measurement mode to determine which RAM registers to use
  mlx90632_meas_select_t meas_mode = getMeasurementSelect();

  int16_t ram_ambient, ram_ref;

  if (meas_mode == MLX90632_MEAS_EXTENDED_RANGE) {
    // Extended range mode: use RAM_54 and RAM_57
    Adafruit_BusIO_Register ram54_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_54), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram57_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_57), 2, MSBFIRST, 2);

    ram_ambient = (int16_t)ram54_reg.read();
    ram_ref = (int16_t)ram57_reg.read();
    ESP_LOGI("MLX90632", "[AMBIENT] Extended mode - RAM_54=0x%04X (%d), RAM_57=0x%04X (%d)",
             (uint16_t)ram_ambient, ram_ambient, (uint16_t)ram_ref, ram_ref);
  } else {
    // Medical mode: use RAM_6 and RAM_9 (default)
    Adafruit_BusIO_Register ram6_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_6), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram9_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_9), 2, MSBFIRST, 2);

    ram_ambient = (int16_t)ram6_reg.read();
    ram_ref = (int16_t)ram9_reg.read();
    ESP_LOGI("MLX90632", "[AMBIENT] Medical mode - RAM_6=0x%04X (%d), RAM_9=0x%04X (%d)",
             (uint16_t)ram_ambient, ram_ambient, (uint16_t)ram_ref, ram_ref);
  }

  // Pre-calculations for ambient temperature (same for both modes)
  double VRTA = ram_ref + (ram_ref - ram_ambient) / 0.02;
  double AMB = Aa + Ba * VRTA;
  double amb_diff = ram_ambient - AMB;
  double P_R = 0.0;  // From calibration
  double PTAT = amb_diff + Gb * amb_diff * amb_diff;
  double ambient_temp = PTAT / (1 + Ka * P_R) + 25.0;

  TA0 = ambient_temp;
  return ambient_temp;
}

/*!
 *    @brief  Calculate object temperature (Melexis algorithm with 5 iterations for maximum accuracy)
 *    @return Object temperature in degrees Celsius
 */
double Adafruit_MLX90632::getObjectTemperature() {
  // Check measurement mode
  mlx90632_meas_select_t meas_mode = getMeasurementSelect();

  int16_t object_new_raw, object_old_raw, ambient_new_raw, ambient_old_raw;

  if (meas_mode == MLX90632_MEAS_EXTENDED_RANGE) {
    // Extended range mode: read RAM_52-59
    Adafruit_BusIO_Register ram52_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_52), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram53_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_53), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram54_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_54), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram55_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_55), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram56_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_56), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram57_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_57), 2, MSBFIRST, 2);

    object_new_raw = (int16_t)ram52_reg.read();
    object_old_raw = (int16_t)ram53_reg.read();
    ambient_new_raw = (int16_t)ram54_reg.read();
    int16_t ambient_new_old = (int16_t)ram55_reg.read();
    ambient_old_raw = (int16_t)ram56_reg.read();
    
    ESP_LOGI("MLX90632", "[OBJECT] Extended - RAM_52=0x%04X RAM_53=0x%04X RAM_54=0x%04X RAM_55=0x%04X RAM_56=0x%04X",
             (uint16_t)object_new_raw, (uint16_t)object_old_raw, (uint16_t)ambient_new_raw, 
             (uint16_t)ambient_new_old, (uint16_t)ambient_old_raw);

    return calcObjectTemperatureExtended(object_new_raw, object_old_raw, ambient_new_raw, 
                                         ambient_old_raw, Ka, Gb, Ea, Eb, Ga, Fa / 2.0, 
                                         Fb, Ha, Hb);
  } else {
    // Medical mode: read RAM_4-9
    Adafruit_BusIO_Register ram4_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_4), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram5_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_5), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram6_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_6), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram7_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_7), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram8_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_8), 2, MSBFIRST, 2);
    Adafruit_BusIO_Register ram9_reg = Adafruit_BusIO_Register(
        i2c_dev, swapBytes(MLX90632_REG_RAM_9), 2, MSBFIRST, 2);

    object_new_raw = (int16_t)ram4_reg.read();
    object_old_raw = (int16_t)ram5_reg.read();
    ambient_new_raw = (int16_t)ram6_reg.read();
    ambient_old_raw = (int16_t)ram9_reg.read();

    return calcObjectTemperatureMedical(object_new_raw, object_old_raw, ambient_new_raw,
                                       ambient_old_raw, Ka, Gb, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
  }
}

/*!
 *    @brief  Byte swap helper for register addresses
 *    @param  value 16-bit value to swap
 *    @return Byte-swapped value
 */
uint16_t Adafruit_MLX90632::swapBytes(uint16_t value) {
  // ESPHome I2C adapter already handles big-endian register addresses
  // No byte swapping needed - return value as-is
  return value;
}

// Global emissivity variable for temperature calculations
static double mlx90632_emissivity = 0.0;

/*!
 *    @brief  Preprocess ambient temperature measurement (Melexis algorithm)
 *    @param  ambient_new_raw New ambient raw reading
 *    @param  ambient_old_raw Old ambient raw reading
 *    @param  Gb Ambient gain coefficient
 *    @return Preprocessed ambient value
 */
double Adafruit_MLX90632::preprocessAmbient(int16_t ambient_new_raw, int16_t ambient_old_raw, double Gb) {
  const double MLX90632_REF_3 = 12.0;
  double kGb = Gb / 1024.0;
  double VR_Ta = ambient_old_raw + kGb * (ambient_new_raw / MLX90632_REF_3);
  return ((ambient_new_raw / MLX90632_REF_3) / VR_Ta) * 524288.0;
}

/*!
 *    @brief  Preprocess object temperature measurement (Melexis algorithm, medical mode)
 *    @param  object_new_raw New object raw reading
 *    @param  object_old_raw Old object raw reading
 *    @param  ambient_new_raw New ambient raw reading
 *    @param  Ka IR gain coefficient
 *    @return Preprocessed object value
 */
double Adafruit_MLX90632::preprocessObject(int16_t object_new_raw, int16_t object_old_raw,
                                           int16_t ambient_new_raw, double Ka) {
  const double MLX90632_REF_3 = 12.0;
  const double MLX90632_REF_12 = 12.0;
  double kKa = Ka / 1024.0;
  double VR_IR = object_old_raw + kKa * (ambient_new_raw / MLX90632_REF_3);
  return ((((object_new_raw + object_old_raw) / 2.0) / MLX90632_REF_12) / VR_IR) * 524288.0;
}

/*!
 *    @brief  Preprocess object (extended mode, Melexis algorithm)
 *    @param  object_new_raw New object raw reading
 *    @param  ambient_new_raw New ambient raw reading
 *    @param  ambient_old_raw Old ambient raw reading
 *    @param  Ka IR gain coefficient
 *    @return Preprocessed object value
 */
double Adafruit_MLX90632::preprocessObjectExtended(int16_t object_new_raw,
                                                   int16_t ambient_new_raw,
                                                   int16_t ambient_old_raw,
                                                   double Ka) {
  const double MLX90632_REF_3 = 12.0;
  const double MLX90632_REF_12 = 12.0;
  double kKa = Ka / 1024.0;
  double VR_IR = ambient_old_raw + kKa * (ambient_new_raw / MLX90632_REF_3);
  return ((object_new_raw / MLX90632_REF_12) / VR_IR) * 524288.0;
}

/*!
 *    @brief  Iterative object temperature calculation (medical mode)
 *    @param  prev_object_temp Previous temperature estimate
 *    @param  object Preprocessed object value
 *    @param  TAdut Ambient temperature (Kelvin reference)
 *    @param  Ga Gain coefficient
 *    @param  Fa Emissivity coefficient
 *    @param  Fb Emissivity coefficient
 *    @param  Ha Ambient offset
 *    @param  Hb Temperature offset
 *    @return Refined temperature estimate
 */
double Adafruit_MLX90632::calcObjectIteration(double prev_object_temp, double object, double TAdut,
                                              double Ga, double Fa, double Fb, double Ha, double Hb) {
  double Ha_customer = Ha / 16384.0;
  double Hb_customer = Hb / 1024.0;
  
  double calcedGa = (Ga * (prev_object_temp - 25.0)) / 68719476736.0;  // 2^36
  double KsTAtmp = Fb * (TAdut - 25.0);
  double calcedGb = KsTAtmp / 68719476736.0;  // 2^36
  
  const double POW10 = 1000000000000000000.0;  // 10^18
  double Alpha_corr = ((Fa * POW10) * Ha_customer * (1.0 + calcedGa + calcedGb)) / 70368744177664.0;  // 2^46
  
  double emissivity = getEmissivity();  // Returns 1.0 if not set
  double calcedFa = object / (emissivity * (Alpha_corr / POW10));
  
  double TAdut4 = (TAdut + 273.15) * (TAdut + 273.15) * (TAdut + 273.15) * (TAdut + 273.15);
  double first_sqrt = sqrt(calcedFa + TAdut4);
  
  return sqrt(first_sqrt) - 273.15 - Hb_customer;
}

/*!
 *    @brief  Iterative object temperature calculation (extended mode)
 *    @param  prev_object_temp Previous temperature estimate
 *    @param  object Preprocessed object value
 *    @param  TAdut Ambient temperature (Kelvin reference)
 *    @param  TaTr4 Reflected temperature to the 4th power
 *    @param  Ga Gain coefficient
 *    @param  Fa Emissivity coefficient (halved for extended mode)
 *    @param  Fb Emissivity coefficient
 *    @param  Ha Ambient offset
 *    @param  Hb Temperature offset
 *    @return Refined temperature estimate
 */
double Adafruit_MLX90632::calcObjectIterationExtended(double prev_object_temp, double object, double TAdut,
                                                      double TaTr4, double Ga, double Fa, double Fb,
                                                      double Ha, double Hb) {
  double Ha_customer = Ha / 16384.0;
  double Hb_customer = Hb / 1024.0;
  
  double calcedGa = (Ga * (prev_object_temp - 25.0)) / 68719476736.0;  // 2^36
  double KsTAtmp = Fb * (TAdut - 25.0);
  double calcedGb = KsTAtmp / 68719476736.0;  // 2^36
  
  const double POW10 = 1000000000000000000.0;  // 10^18
  double Alpha_corr = ((Fa * POW10) * Ha_customer * (1.0 + calcedGa + calcedGb)) / 70368744177664.0;  // 2^46
  
  double emissivity = getEmissivity();
  double calcedFa = object / (emissivity * (Alpha_corr / POW10));
  
  double first_sqrt = sqrt(calcedFa + TaTr4);
  
  return sqrt(first_sqrt) - 273.15 - Hb_customer;
}

/*!
 *    @brief  Calculate object temperature (medical mode, Melexis exact algorithm)
 *    @return Object temperature in degrees Celsius
 */
double Adafruit_MLX90632::calcObjectTemperatureMedical(int16_t object_new_raw, int16_t object_old_raw,
                                                      int16_t ambient_new_raw, int16_t ambient_old_raw,
                                                      double Ka, double Gb, double Ea, double Eb,
                                                      double Ga, double Fa, double Fb, double Ha, double Hb) {
  // Preprocess measurements
  double AMB = preprocessAmbient(ambient_new_raw, ambient_old_raw, Gb);
  double object = preprocessObject(object_new_raw, object_old_raw, ambient_new_raw, Ka);
  
  // Convert ambient to device temperature
  double kEa = Ea / 65536.0;
  double kEb = Eb / 256.0;
  double TAdut = ((AMB - kEb) / kEa) + 25.0;
  
  // Initial temperature estimate
  double temp = 25.0;
  
  // 5 iterations for accuracy (Melexis DSPv5 requirement)
  for (int i = 0; i < 5; i++) {
    temp = calcObjectIteration(temp, object, TAdut, Ga, Fa, Fb, Ha, Hb);
  }
  
  TO0 = temp;
  return temp;
}

/*!
 *    @brief  Calculate object temperature (extended mode, Melexis exact algorithm)
 *    @return Object temperature in degrees Celsius
 */
double Adafruit_MLX90632::calcObjectTemperatureExtended(int16_t object_new_raw, int16_t object_old_raw,
                                                       int16_t ambient_new_raw, int16_t ambient_old_raw,
                                                       double Ka, double Gb, double Ea, double Eb,
                                                       double Ga, double Fa_half, double Fb, double Ha, double Hb) {
  // Preprocess measurements
  double AMB = preprocessAmbient(ambient_new_raw, ambient_old_raw, Gb);
  double object = preprocessObjectExtended(object_new_raw, ambient_new_raw, ambient_old_raw, Ka);
  
  // Convert ambient to device temperature
  double kEa = Ea / 65536.0;
  double kEb = Eb / 256.0;
  double TAdut = ((AMB - kEb) / kEa) + 25.0;
  
  // Reflected temperature calculation
  double TaTr4 = (object_old_raw + 273.15);  // Use old ambient as reference
  TaTr4 = TaTr4 * TaTr4 * TaTr4 * TaTr4;  // Fourth power
  double ta4 = (TAdut + 273.15);
  ta4 = ta4 * ta4 * ta4 * ta4;
  
  double emissivity = getEmissivity();
  TaTr4 = TaTr4 - (TaTr4 - ta4) / emissivity;
  
  // Initial temperature estimate
  double temp = 25.0;
  
  // 5 iterations for accuracy
  for (int i = 0; i < 5; i++) {
    temp = calcObjectIterationExtended(temp, object, TAdut, TaTr4, Ga, Fa_half, Fb, Ha, Hb);
  }
  
  TO0 = temp;
  return temp;
}

/*!
 *    @brief  Set the emissivity value for temperature calculations
 *    @param  value Emissivity value (0.0-1.0, where 0.0 defaults to 1.0)
 */
void Adafruit_MLX90632::setEmissivity(double value) {
  mlx90632_emissivity = value;
}

/*!
 *    @brief  Get the current emissivity value
 *    @return Emissivity value (1.0 if not set or set to 0.0)
 */
double Adafruit_MLX90632::getEmissivity() const {
  if (mlx90632_emissivity == 0.0) {
    return 1.0;
  } else {
    return mlx90632_emissivity;
  }
}

/*!
 *    @brief  Set ESPHome I2CDevice for direct atomic transactions
 *    @param  device Pointer to ESPHome I2CDevice
 */
void Adafruit_MLX90632::set_esphome_i2c_device(esphome::i2c::I2CDevice *device) {
  if (i2c_dev) {
    i2c_dev->set_esphome_device(device);
  }
}

/*!
 *    @brief  Set ESPHome I2CDevice for direct atomic transactions
 *    @param  device Pointer to ESPHome I2CDevice
 */
void Adafruit_MLX90632::set_esphome_i2c_device(esphome::i2c::I2CDevice *device) {
  if (i2c_dev) {
    i2c_dev->set_esphome_device(device);
  }
}
