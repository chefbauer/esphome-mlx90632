#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include <cmath>

namespace esphome {
namespace mlx90632 {

static const char *const TAG = "mlx90632";
static const char *const FW_VERSION = "V.N6";

// MLX90632 I2C Address
#define MLX90632_DEFAULT_ADDR 0x3A

// EEPROM Registers (calibration constants)
#define MLX90632_REG_EE_P_R_LSW 0x240C
#define MLX90632_REG_EE_P_R_MSW 0x240D
#define MLX90632_REG_EE_P_G_LSW 0x240E
#define MLX90632_REG_EE_P_G_MSW 0x240F
#define MLX90632_REG_EE_P_T_LSW 0x2410
#define MLX90632_REG_EE_P_T_MSW 0x2411
#define MLX90632_REG_EE_P_O_LSW 0x2412
#define MLX90632_REG_EE_P_O_MSW 0x2413
#define MLX90632_REG_EE_AA_LSW 0x2414
#define MLX90632_REG_EE_AA_MSW 0x2415
#define MLX90632_REG_EE_AB_LSW 0x2416
#define MLX90632_REG_EE_AB_MSW 0x2417
#define MLX90632_REG_EE_BA_LSW 0x2418
#define MLX90632_REG_EE_BA_MSW 0x2419
#define MLX90632_REG_EE_BB_LSW 0x241A
#define MLX90632_REG_EE_BB_MSW 0x241B
#define MLX90632_REG_EE_CA_LSW 0x241C
#define MLX90632_REG_EE_CA_MSW 0x241D
#define MLX90632_REG_EE_CB_LSW 0x241E
#define MLX90632_REG_EE_CB_MSW 0x241F
#define MLX90632_REG_EE_DA_LSW 0x2420
#define MLX90632_REG_EE_DA_MSW 0x2421
#define MLX90632_REG_EE_DB_LSW 0x2422
#define MLX90632_REG_EE_DB_MSW 0x2423
#define MLX90632_REG_EE_EA_LSW 0x2424
#define MLX90632_REG_EE_EA_MSW 0x2425
#define MLX90632_REG_EE_EB_LSW 0x2426
#define MLX90632_REG_EE_EB_MSW 0x2427
#define MLX90632_REG_EE_FA_LSW 0x2428
#define MLX90632_REG_EE_FA_MSW 0x2429
#define MLX90632_REG_EE_FB_LSW 0x242A
#define MLX90632_REG_EE_FB_MSW 0x242B
#define MLX90632_REG_EE_GA_LSW 0x242C
#define MLX90632_REG_EE_GA_MSW 0x242D
#define MLX90632_REG_EE_GB 0x242E
#define MLX90632_REG_EE_KA 0x242F
#define MLX90632_REG_EE_KB 0x2430
#define MLX90632_REG_EE_HA 0x2481
#define MLX90632_REG_EE_HB 0x2482
#define MLX90632_REG_EE_PRODUCT_CODE 0x2409
#define MLX90632_REG_EE_VERSION 0x240B
#define MLX90632_REG_EE_CONTROL 0x24D4
#define MLX90632_REG_EE_I2C_ADDRESS 0x24D5
#define MLX90632_REG_EE_MEAS_1 0x24E1
#define MLX90632_REG_EE_MEAS_2 0x24E2

// Control and Status Registers
#define MLX90632_REG_I2C_ADDRESS 0x3000
#define MLX90632_REG_CONTROL 0x3001
#define MLX90632_REG_STATUS 0x3FFF

// RAM Registers
#define MLX90632_REG_RAM_1 0x4000
#define MLX90632_REG_RAM_2 0x4001
#define MLX90632_REG_RAM_3 0x4002
#define MLX90632_REG_RAM_4 0x4003
#define MLX90632_REG_RAM_5 0x4004
#define MLX90632_REG_RAM_6 0x4005
#define MLX90632_REG_RAM_7 0x4006
#define MLX90632_REG_RAM_8 0x4007
#define MLX90632_REG_RAM_9 0x4008
#define MLX90632_REG_RAM_52 0x4033
#define MLX90632_REG_RAM_53 0x4034
#define MLX90632_REG_RAM_54 0x4035
#define MLX90632_REG_RAM_55 0x4036
#define MLX90632_REG_RAM_56 0x4037
#define MLX90632_REG_RAM_57 0x4038
#define MLX90632_REG_RAM_58 0x4039
#define MLX90632_REG_RAM_59 0x403A

// Measurement modes
typedef enum {
  MLX90632_MODE_HALT = 0x00,
  MLX90632_MODE_SLEEPING_STEP = 0x01,
  MLX90632_MODE_STEP = 0x02,
  MLX90632_MODE_CONTINUOUS = 0x03
} mlx90632_mode_t;

// Measurement types
typedef enum {
  MLX90632_MEAS_MEDICAL = 0x00,
  MLX90632_MEAS_EXTENDED_RANGE = 0x11
} mlx90632_meas_select_t;

// Refresh rates
typedef enum {
  MLX90632_REFRESH_0_5HZ = 0,
  MLX90632_REFRESH_1HZ = 1,
  MLX90632_REFRESH_2HZ = 2,
  MLX90632_REFRESH_4HZ = 3,
  MLX90632_REFRESH_8HZ = 4,
  MLX90632_REFRESH_16HZ = 5,
  MLX90632_REFRESH_32HZ = 6,
  MLX90632_REFRESH_64HZ = 7
} mlx90632_refresh_rate_t;

class MLX90632Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Adafruit-style functions
  bool begin();
  bool reset();
  bool getCalibrations();
  bool setMode(mlx90632_mode_t mode);
  mlx90632_mode_t getMode();
  bool setMeasurementSelect(mlx90632_meas_select_t meas_select);
  mlx90632_meas_select_t getMeasurementSelect();
  bool setRefreshRate(mlx90632_refresh_rate_t refresh_rate);
  mlx90632_refresh_rate_t getRefreshRate();
  bool isNewData();
  bool resetNewData();
  uint8_t readCyclePosition();
  double getAmbientTemperature();
  double getObjectTemperature();
  uint16_t getProductCode();
  uint16_t getEEPROMVersion();

 protected:
  // I2C helpers
  bool read_register16(uint16_t reg, uint16_t *value);
  bool write_register16(uint16_t reg, uint16_t value);
  uint16_t swapBytes(uint16_t value);
  uint32_t read32BitRegister(uint16_t lsw_addr);

  // Calibration constants (from Adafruit)
  double P_R, P_G, P_T, P_O;
  double Aa, Ab, Ba, Bb;
  double Ca, Cb, Da, Db;
  double Ea, Eb, Fa, Fb;
  double Ga, Gb, Ka;
  int16_t Kb;
  double Ha, Hb;

  // Temperature calculation variables
  double TO0 = 25.0;  // Previous object temperature
  double TA0 = 25.0;  // Previous ambient temperature
};

}  // namespace mlx90632
}  // namespace esphome
