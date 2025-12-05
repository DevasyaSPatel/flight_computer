#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include "FlightConfig.h"

#include "KalmanFilter.h"

// --- Objects ---
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
SimpleKalmanFilter pressureKalmanFilter(1.0, 1.0, 0.01); // measurement uncertainty, estimation uncertainty, process noise

// --- Variables ---
float currentAltitude = 0;
float filteredAltitude = 0;
float maxAltitude = -10000;
float groundAltitude = 0;
float velocity = 0;
float temperature = 0;

// Acceleration & Gyro
float ax, ay, az;
float gx, gy, gz;

// State Machine
enum FlightState {
  STATE_IDLE,
  STATE_ASCENT,
  STATE_APOGEE,
  STATE_DESCENT,
  STATE_LANDED
};

FlightState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastVelocityTime = 0;
unsigned long ejectionStartTime = 0;
bool ejectionTriggered = false;

long packetCount = 0;

void setup() {
  Serial.begin(115200);
  
  // Pin Setup
  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, LOW); // Ensure OFF
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // I2C Setup
  Wire.begin(I2C_SDA, I2C_SCL);

  // Sensor Initialization
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  // LoRa Initialization
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // Calibration / Initial Readings
  delay(1000); // Wait for sensors to settle
  float sumAlt = 0;
  for(int i=0; i<10; i++) {
     sumAlt += bmp.readAltitude(SEA_LEVEL_PRESSURE * 100);
     delay(50);
  }
  groundAltitude = sumAlt / 10.0;
  
  // Initialize Kalman Filter with ground altitude
  // We assume initial estimate is ground altitude
  // But the filter starts at 0 if not set, so let's just let it converge or we could add a setEstimate method.
  // For now, the first few readings will converge it.
  
  Serial.println("System Ready.");
  tone(PIN_BUZZER, 1000, 500); // Startup beep
}

void loop() {
  unsigned long currentTime = millis();

  // 1. Read Sensors
  readSensors(currentTime);

  // 2. State Machine Logic
  runStateMachine(currentTime);

  // 3. Telemetry
  if (currentTime - lastTelemetryTime >= TELEMETRY_RATE) {
    sendTelemetry();
    lastTelemetryTime = currentTime;
  }
}

void readSensors(unsigned long currentTime) {
  // MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
  
  // BMP085
  float rawAlt = bmp.readAltitude(SEA_LEVEL_PRESSURE * 100);
  currentAltitude = rawAlt;
  
  // Kalman Filter
  filteredAltitude = pressureKalmanFilter.updateEstimate(currentAltitude);
  
  // Calculate Velocity
  // v = (alt_new - alt_old) / dt
  if (lastVelocityTime == 0) lastVelocityTime = currentTime;
  unsigned long dt = currentTime - lastVelocityTime;
  if (dt >= 50) { // Calculate velocity every 50ms to avoid noise amplification
      static float lastFilteredAlt = groundAltitude;
      velocity = (filteredAltitude - lastFilteredAlt) / (dt / 1000.0);
      lastFilteredAlt = filteredAltitude;
      lastVelocityTime = currentTime;
  }
  
  temperature = bmp.readTemperature();

  // Update Max Altitude
  if (currentState == STATE_ASCENT && filteredAltitude > maxAltitude) {
    maxAltitude = filteredAltitude;
  }
}

void runStateMachine(unsigned long currentTime) {
  switch (currentState) {
    case STATE_IDLE:
      // Detect Launch: High acceleration upwards (usually Z axis depending on mounting)
      // Assuming Z is vertical. Check magnitude to be safe.
      float accMag = sqrt(ax*ax + ay*ay + az*az);
      if (accMag > ACCEL_LAUNCH_THRESHOLD) {
        currentState = STATE_ASCENT;
        stateStartTime = currentTime;
        Serial.println("LAUNCH DETECTED!");
      }
      break;

    case STATE_ASCENT:
      // Detect Apogee: Current altitude drops below max altitude
      // Use filtered altitude for smoother apogee detection
      if (maxAltitude - filteredAltitude > APOGEE_DROP_THRESHOLD) {
        currentState = STATE_APOGEE;
        stateStartTime = currentTime;
        Serial.println("APOGEE DETECTED!");
      }
      break;

    case STATE_APOGEE:
      // Trigger Ejection
      if (!ejectionTriggered) {
        digitalWrite(PIN_MOSFET, HIGH);
        ejectionTriggered = true;
        ejectionStartTime = currentTime;
        Serial.println("EJECTION TRIGGERED!");
      }
      
      // Turn off after burn time
      if (ejectionTriggered && (currentTime - ejectionStartTime > EJECTION_BURN_TIME)) {
        digitalWrite(PIN_MOSFET, LOW);
        currentState = STATE_DESCENT;
        Serial.println("EJECTION COMPLETE. DESCENDING.");
      }
      break;

    case STATE_DESCENT:
      // Detect Landing: Altitude stable for a while (simplified here: low altitude)
      if (filteredAltitude < groundAltitude + 5.0) { // Near ground
         // Could add a timer check for stability
         currentState = STATE_LANDED;
         Serial.println("LANDED.");
         tone(PIN_BUZZER, 2000, 1000); // Landing beep
      }
      break;

    case STATE_LANDED:
      // Beacon mode or stop
      break;
  }
}

void sendTelemetry() {
  String packet = "";
  packet += String(packetCount++) + ",";
  packet += String(currentState) + ",";
  packet += String(filteredAltitude, 2) + ",";
  packet += String(maxAltitude, 2) + ",";
  packet += String(velocity, 2) + ","; // TODO: Calculate real velocity
  packet += String(ax, 2) + ",";
  packet += String(ay, 2) + ",";
  packet += String(az, 2) + ",";
  packet += String(gx, 2) + ",";
  packet += String(gy, 2) + ",";
  packet += String(gz, 2) + ",";
  packet += String(temperature, 2);

  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

  // Also print to Serial for debugging
  Serial.println(packet);
}
