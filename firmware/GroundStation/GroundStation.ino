#include <SPI.h>
#include <LoRa.h>

// --- SHARED CONFIG ---
// IMPORTANT: Adjust path if your IDE doesn't support relative includes. 
// You can also copy FlightConfig.h to the GroundStation folder.
#include "../FlightComputer/FlightConfig.h" 

// --- Constants ---
// Data struct and LoRa settings are now pulled from FlightConfig.h!

// --- Global Tracking ---
float maxAltReceived = 0;

void setup() {
  Serial.begin(115200);
  delay(1000); 

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa Init Failed!");
    while (1);
  }
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  
  Serial.println("GS Ready. Linked to FlightConfig.");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) { 
    if (packetSize == sizeof(RocketData)) {
      RocketData data;
      LoRa.readBytes((uint8_t*)&data, sizeof(RocketData));

      // 1. Verify Checksum
      uint8_t calcChecksum = 0;
      uint8_t *ptr = (uint8_t*)&data;
      for (int i = 0; i < sizeof(RocketData) - 1; i++) {
        calcChecksum ^= ptr[i];
      }

      if (calcChecksum == data.checksum) {
        // Valid Packet!
        
        // 2. Unpack
        float altitude = data.alt_cm / 100.0;
        float velocity = data.vel_cm / 100.0;
        float accelG   = data.acc_mg / 1000.0;
        float maxAlt   = data.max_alt_cm / 100.0;

        // 3. Serial Output (CSV)
        // Timestamp, State, Alt, MaxAlt, Vel, Acc
        Serial.print(data.timestamp); Serial.print(",");
        Serial.print(data.state); Serial.print(",");
        Serial.print(altitude, 2); Serial.print(",");
        Serial.print(maxAlt, 2); Serial.print(",");
        Serial.print(velocity, 2); Serial.print(",");
        Serial.print(accelG, 2); Serial.print(",");
        
        Serial.print(LoRa.packetRssi()); Serial.print(",");
        Serial.println(LoRa.packetSnr());

      } else {
        Serial.println("Error: Checksum Fail");
      }

    } else {
      Serial.print("Error: Size Mismatch. Got: ");
      Serial.println(packetSize);
      LoRa.readString(); // Flush
    }
  }
}
