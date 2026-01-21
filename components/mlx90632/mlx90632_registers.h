#pragma once

#include <cstdint>

// MLX90632 Register Addresses (16-bit addresses)
namespace mlx90632_registers {

// RAM registers (measurement data) - Medical Mode
constexpr uint16_t RAM_4 = 0x4003;
constexpr uint16_t RAM_5 = 0x4004;
constexpr uint16_t RAM_6 = 0x4005;
constexpr uint16_t RAM_7 = 0x4006;
constexpr uint16_t RAM_8 = 0x4007;
constexpr uint16_t RAM_9 = 0x4008;

// Extended range RAM registers - Official from Datasheet Rev 13
constexpr uint16_t RAM_52 = 0x4033;  // Object new
constexpr uint16_t RAM_53 = 0x4034;  // Object old
constexpr uint16_t RAM_54 = 0x4035;  // Ambient new
constexpr uint16_t RAM_55 = 0x4036;  // Ambient new/old
constexpr uint16_t RAM_56 = 0x4037;  // Ambient old
constexpr uint16_t RAM_57 = 0x4038;  // Ambient ref

// Control and status registers
constexpr uint16_t REG_STATUS = 0x3FFF;
constexpr uint16_t REG_CONTROL = 0x3001;
constexpr uint16_t REG_I2C_ADDRESS = 0x3000;

// EEPROM calibration registers (32-bit, stored as LSW/MSW pairs)
// Official addresses from MLX90632 Datasheet Rev 13 (June 2025)
constexpr uint16_t EE_P_R_LSW = 0x240C;  // P_R[15:0]
constexpr uint16_t EE_P_G_LSW = 0x240E;  // P_G[15:0]
constexpr uint16_t EE_P_T_LSW = 0x2410;  // P_T[15:0]
constexpr uint16_t EE_P_O_LSW = 0x2412;  // P_O[15:0]
constexpr uint16_t EE_AA_LSW = 0x2414;   // Aa[15:0]
constexpr uint16_t EE_AB_LSW = 0x2416;   // Ab[15:0]
constexpr uint16_t EE_BA_LSW = 0x2418;   // Ba[15:0]
constexpr uint16_t EE_BB_LSW = 0x241A;   // Bb[15:0]
constexpr uint16_t EE_CA_LSW = 0x241C;   // Ca[15:0]
constexpr uint16_t EE_CB_LSW = 0x241E;   // Cb[15:0]
constexpr uint16_t EE_DA_LSW = 0x2420;   // Da[15:0]
constexpr uint16_t EE_DB_LSW = 0x2422;   // Db[15:0]
constexpr uint16_t EE_EA_LSW = 0x2424;   // Ea[15:0]
constexpr uint16_t EE_EB_LSW = 0x2426;   // Eb[15:0]
constexpr uint16_t EE_FA_LSW = 0x2428;   // Fa[15:0]
constexpr uint16_t EE_FB_LSW = 0x242A;   // Fb[15:0]
constexpr uint16_t EE_GA_LSW = 0x242C;   // Ga[15:0]

// EEPROM calibration registers (16-bit)
constexpr uint16_t EE_GB = 0x242E;  // Gb[15:0]
constexpr uint16_t EE_KA = 0x242F;  // Ka[15:0]
constexpr uint16_t EE_KB = 0x2430;  // Kb[15:0]
constexpr uint16_t EE_HA = 0x2481;  // Ha[15:0] - Customer calibration (R/W)
constexpr uint16_t EE_HB = 0x2482;  // Hb[15:0] - Customer calibration (R/W)

// Product ID
constexpr uint16_t EE_ID0 = 0x2405;
constexpr uint16_t EE_ID1 = 0x2406;
constexpr uint16_t EE_ID2 = 0x2407;

// Control register bits
constexpr uint16_t CTRL_MODE_CONTINUOUS = 0x0001;
constexpr uint16_t CTRL_MODE_STEP = 0x0002;
constexpr uint16_t CTRL_SOB = 0x0010;  // Start of Burst (wake sensor)
constexpr uint16_t CTRL_SOC = 0x0008;  // Start of Conversion

// Status register bits
constexpr uint16_t STATUS_NEW_DATA = 0x0001;
constexpr uint16_t STATUS_CYCLE_POS_MASK = 0x001E;
constexpr uint16_t STATUS_EEPROM_BUSY = 0x0020;
constexpr uint16_t STATUS_DEVICE_BUSY = 0x0040;

}  // namespace mlx90632_registers
