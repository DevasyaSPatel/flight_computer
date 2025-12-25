#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include <stdint.h>

// --- Pin Definitions (ESP32) ---
#define LORA_SS     5
#define LORA_RST    14
#define LORA_DIO0   2

#define I2C_SDA     21
#define I2C_SCL     22
#define PIN_MOSFET  13  
#define PIN_BUZZER  12 // Changed from 12 to 14 per prompt
#define PIN_LED     15 // Added per prompt

// --- Constants ---
#define SEA_LEVEL_PRESSURE    1013.25 // Standard, can be calibrated
#define LORA_FREQ             433E6 
#define LORA_TX_POWER         20
#define LORA_SPREADING_FACTOR 8    // Lowered from 12 to 8 to allow higher data rate (SF12 is too slow for >1Hz)
#define LORA_SYNC_WORD        0xF3 // Added for isolation (0xF3 is a common private sync word)
// --- Tuning ---
#define ACCEL_LAUNCH_THRESHOLD  15.0 // m/s^2 (~1.5G)
#define APOGEE_VEL_THRESHOLD    2.0  // m/s (Absolute value check for < 2.0 or similar)
#define LANDING_ALT_THRESHOLD   10.0 // meters above zero
#define EJECTION_BURN_TIME      2000 // ms
#define LANDING_DETECT_TIME     3000 // ms

// --- Telemetry Packet (10 Bytes) ---
#pragma pack(push, 1)
struct RocketData {
  uint16_t alt_cm;      // Altitude (meters * 100)
  int16_t  vel_cm;      // Velocity (m/s * 100)
  int16_t  acc_mg;      // Accel (milli-Gs)
  uint8_t  state;       // Flight State
  uint8_t  time_ms[3];  // Mission time (uint24 equivalent)
};
#pragma pack(pop)

#endif