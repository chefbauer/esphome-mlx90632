#pragma once

#include <cstdint>

// MLX90632 Register Addresses (16-bit addresses)
namespace mlx90632_registers {

// RAM registers (measurement data)
constexpr uint16_t RAM_4 = 0x4003;
constexpr uint16_t RAM_5 = 0x4004;
constexpr uint16_t RAM_6 = 0x4005;
constexpr uint16_t RAM_7 = 0x4006;
constexpr uint16_t RAM_8 = 0x4007;
constexpr uint16_t RAM_9 = 0x4008;

// Extended range RAM registers
constexpr uint16_t RAM_52 = 0x4005;  // Object new
constexpr uint16_t RAM_53 = 0x4006;  // Object old
constexpr uint16_t RAM_54 = 0x4007;  // Ambient new
constexpr uint16_t RAM_55 = 0x4008;  // Ambient new/old
constexpr uint16_t RAM_56 = 0x4009;  // Ambient old
constexpr uint16_t RAM_57 = 0x400A;  // Ambient ref

// Control and status registers
constexpr uint16_t REG_STATUS = 0x3FFF;
constexpr uint16_t REG_CONTROL = 0x3001;
constexpr uint16_t REG_I2C_ADDRESS = 0x3000;

// EEPROM calibration registers (32-bit, stored as LSW/MSW pairs)
constexpr uint16_t EE_P_R_LSW = 0x243D;
constexpr uint16_t EE_P_G_LSW = 0x243F;
constexpr uint16_t EE_P_T_LSW = 0x2441;
constexpr uint16_t EE_P_O_LSW = 0x2443;
constexpr uint16_t EE_AA_LSW = 0x2445;
constexpr uint16_t EE_AB_LSW = 0x2447;
constexpr uint16_t EE_BA_LSW = 0x2449;
constexpr uint16_t EE_BB_LSW = 0x244B;
constexpr uint16_t EE_CA_LSW = 0x244D;
constexpr uint16_t EE_CB_LSW = 0x244F;
constexpr uint16_t EE_DA_LSW = 0x2451;
constexpr uint16_t EE_DB_LSW = 0x2453;
constexpr uint16_t EE_EA_LSW = 0x2455;
constexpr uint16_t EE_EB_LSW = 0x2457;
constexpr uint16_t EE_FA_LSW = 0x2459;
constexpr uint16_t EE_FB_LSW = 0x245B;
constexpr uint16_t EE_GA_LSW = 0x245D;

// EEPROM calibration registers (16-bit)
constexpr uint16_t EE_GB = 0x245F;
constexpr uint16_t EE_KA = 0x2460;
constexpr uint16_t EE_KB = 0x2461;
constexpr uint16_t EE_HA = 0x2481;
constexpr uint16_t EE_HB = 0x2482;

// Product ID
constexpr uint16_t EE_ID0 = 0x2405;
constexpr uint16_t EE_ID1 = 0x2406;
constexpr uint16_t EE_ID2 = 0x2407;

// Control register bits
constexpr uint16_t CTRL_MODE_CONTINUOUS = 0x0001;
constexpr uint16_t CTRL_MODE_STEP = 0x0002;

// Status register bits
constexpr uint16_t STATUS_NEW_DATA = 0x0001;
constexpr uint16_t STATUS_CYCLE_POS_MASK = 0x001E;
constexpr uint16_t STATUS_EEPROM_BUSY = 0x0020;
constexpr uint16_t STATUS_DEVICE_BUSY = 0x0040;

}  // namespace mlx90632_registers
