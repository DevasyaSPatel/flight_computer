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
#define PIN_BUZZER  12 // User preferred Pin 12
#define PIN_LED     15 

// --- Constants ---
#define SEA_LEVEL_PRESSURE    1013 
#define LORA_FREQ             433E6 
#define LORA_TX_POWER         20
#define LORA_SPREADING_FACTOR 8     // Balanced for range/speed
#define LORA_SYNC_WORD        0xF3  // Network Isolation

// --- Tuning ---
#define ACCEL_LAUNCH_THRESHOLD  15.0 
#define APOGEE_VEL_THRESHOLD    2.0 
#define LANDING_ALT_THRESHOLD   10.0 
#define EJECTION_BURN_TIME      2000 
#define LANDING_DETECT_TIME     3000 

// --- Telemetry Packet (14 Bytes) ---
// Switched to standard types for reliability. 
// Size increase is negligible for LoRa.
#pragma pack(push, 1)
struct RocketData {
  uint32_t timestamp;   // 4 bytes
  uint8_t  state;       // 1 byte
  int16_t  alt_cm;      // 2 bytes
  int16_t  vel_cm;      // 2 bytes
  int16_t  acc_mg;      // 2 bytes
  int16_t  max_alt_cm;  // 2 bytes (Added for redundancy)
  uint8_t  checksum;    // 1 byte (XOR checksum)
};
#pragma pack(pop)

#endif
