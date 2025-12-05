#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

// --- Pin Definitions (ESP32) ---

// LoRa Module (SX1278/RFM95)
#define LORA_SS     5
#define LORA_RST    14
#define LORA_DIO0   2

// I2C (MPU6050, BMP085)
#define I2C_SDA     21
#define I2C_SCL     22

// Actuators
#define PIN_MOSFET  13  // Pyro channel for ejection
#define PIN_BUZZER  12  // Optional buzzer for status

// --- Constants ---

// Altimeter
#define SEA_LEVEL_PRESSURE 1013.25 // hPa, update before flight if possible

// Flight Logic
#define ACCEL_LAUNCH_THRESHOLD  15.0 // m/s^2 (approx 1.5g)
#define APOGEE_DROP_THRESHOLD   2.0  // meters (detect apogee after dropping this much)
#define EJECTION_BURN_TIME      2000 // milliseconds to keep mosfet ON

// Telemetry
#define LORA_FREQ   433E6 // or 915E6 depending on region
#define TELEMETRY_RATE 100 // ms between packets

#endif
