
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include "FlightConfig.h"
#include "KalmanFilter.h"

// --- Components ---
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
KalmanFilter2D fkalman; // 2D Kalman Filter (Pos, Vel)

// --- State Machine ---
enum FlightState {
  STATE_PRE_FLIGHT = 0,
  STATE_ARMED,
  STATE_POWERED_FLIGHT,
  STATE_COASTING,
  STATE_APOGEE,
  STATE_DESCENT,
  STATE_LANDED
};
FlightState currentState = STATE_PRE_FLIGHT;

// --- Global Variables ---
RocketData dataPacket;
unsigned long lastLoopTime = 0;
unsigned long lastTelemetryTime = 0;
const int LOOP_PERIOD_MS = 5; // 200Hz Control Loop

// Physics Tracking
float groundAltitude = 0;
float maxAltitude = -10000;
unsigned long stateStartTime = 0;

// Landing Detection
unsigned long landingStableStartTime = 0;

// Mosfet Safety
bool mosfetActive = false;
unsigned long mosfetStartTime = 0;

// Attitude Estimation (Complementary Filter)
float pitch = 0;
float roll = 0;
float alpha = 0.98; // Filter weight
unsigned long lastSensorRead = 0;

void setup() {
  Serial.begin(115200);
  
  // 1. Pin Setup
  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  
  // 2. I2C & Sensors
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz I2C
  
  if (!mpu.begin()) {
    Serial.println("MPU Fail!");
    while(1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!bmp.begin()) {
    Serial.println("BMP Fail!");
    while(1);
  }

  // 3. LoRa Setup
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa Fail!");
    while(1);
  }
  LoRa.setTxPower(17); // Reduced from 20 to 17dBm to prevent power sag issues
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSyncWord(LORA_SYNC_WORD); // Isolate network
  LoRa.enableCrc(); // Enable Data Integrity Check

  // 4. Calibration
  Serial.println("Calibrating Baro...");
  digitalWrite(PIN_LED, HIGH);
  float sum = 0;
  for(int i=0; i<100; i++) {
    sum += bmp.readAltitude(SEA_LEVEL_PRESSURE * 100);
    delay(10);
  }
  groundAltitude = sum / 100.0;
  digitalWrite(PIN_LED, LOW);
  
  // Initialize Kalman Filter
  // Q_pos=0.1, Q_vel=0.1, R_alt=2.0 (Tune these!)
  fkalman.begin(groundAltitude, 0.1, 0.1, 2.0); 
  
  // Beep to indicate ready
  tone(PIN_BUZZER, 2000, 200);
  Serial.println("READY.");
  currentState = STATE_PRE_FLIGHT; // Start in Pre-flight (Calibration done)
}

void loop() {
  unsigned long currentTime = millis();
  
  // Main Control Loop at 200Hz
  if (currentTime - lastLoopTime >= LOOP_PERIOD_MS) {
    float dt = (currentTime - lastLoopTime) / 1000.0;
    lastLoopTime = currentTime;

    // A. Read Sensors & Run Physics
    runPhysics(dt);

    // B. State Machine
    updateFSM(currentTime);

    // C. Safety Checks
    checkSafety(currentTime);
  }

  // Telemetry Loop
  // SF8 at 125kHz allows ~10Hz easily.
  // We'll set 200ms (5Hz) as a safe baseline for reliable long range.
  int telem_period = (currentState >= STATE_POWERED_FLIGHT && currentState <= STATE_DESCENT) ? 200 : 1000;
  
  if (currentTime - lastTelemetryTime >= telem_period) {
    sendTelemetry();
    lastTelemetryTime = currentTime;
  }
}

void runPhysics(float dt) {
  // 1. MPU Read
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 2. Attitude Estimation (Complementary Filter)
  // Calculate accelerometer angles
  float accPitch = atan2(a.acceleration.y, a.acceleration.z);
  float accRoll  = atan2(-a.acceleration.x, a.acceleration.z); // Simple approximation
  
  // Integrate Gyro
  pitch += g.gyro.x * dt;
  roll  += g.gyro.y * dt;
  
  // Fuse
  pitch = alpha * pitch + (1.0 - alpha) * accPitch;
  roll  = alpha * roll  + (1.0 - alpha) * accRoll;
  
  // 3. Resolve Vertical Acceleration
  // Rotate body acceleration to earth frame Z
  // a_vert_earth = -sin(pitch)*ax + sin(roll)*cos(pitch)*ay + cos(roll)*cos(pitch)*az
  // Assuming typical orientation relative to sensor frame. 
  // Simplified projection: a_z_earth = a_body_z / cos(tilt) approx, but correct rotation is:
  // We want the component opposing gravity.
  // Let's use the dot product with the estimated gravity vector.
  // But strictly per prompt: "remove gravity and resolve Z-vector"
  
  // True vertical acceleration (removing 9.81 gravity)
  // Using simplified rotation for small angles or standard calculation:
  // a_earth_z = -sin(theta)*ax + cos(theta)*sin(phi)*ay + cos(theta)*cos(phi)*az
  // theta = pitch, phi = roll
  
  float grav_corrected_accel = 
      -sin(pitch) * a.acceleration.x + 
       sin(roll) * cos(pitch) * a.acceleration.y + 
       cos(roll) * cos(pitch) * a.acceleration.z;
       
  float vertical_accel = grav_corrected_accel - 9.81;

  // 4. Kalman Prediction
  fkalman.predict(vertical_accel, dt);
  
  // 5. Kalman Update (Barometer)
  // Only update if new data available? BMP085 is slow (~20-40ms).
  // We can read every loop but it yields same data. 
  // Better to check if needed, but for simplicity read every 20ms or just run it.
  // BMP085 read is blocking on some libs, Adafruit_BMP085 is blocking IO.
  // We might want to limit baro reads to 50Hz.
  static unsigned long lastBaro = 0;
  if (millis() - lastBaro > 20) {
    float rawAlt = bmp.readAltitude(SEA_LEVEL_PRESSURE * 100);
    fkalman.update(rawAlt);
    lastBaro = millis();
  }

  // Update Global Data
  dataPacket.acc_mg = (int16_t)(vertical_accel * 1000.0 / 9.81 * 1000.0); // Wait, mg? 1G = 1000mg.
  // vertical_accel is m/s^2. 1G = 9.81.
  dataPacket.acc_mg = (int16_t)((vertical_accel / 9.81) * 1000.0);
  
  dataPacket.alt_cm = (uint16_t)(fkalman.pos * 100.0);
  dataPacket.vel_cm = (int16_t)(fkalman.vel * 100.0);
  
  if (fkalman.pos > maxAltitude) maxAltitude = fkalman.pos;
}

void updateFSM(unsigned long now) {
  // Thresholds
  float vel = fkalman.vel;
  float alt = fkalman.pos;
  float accel = dataPacket.acc_mg / 1000.0 * 9.81; // retrieve accel m/s^2

  static int launchDetectCount = 0;

  switch (currentState) {
    case STATE_PRE_FLIGHT:
      digitalWrite(PIN_LED, (now/500)%2); // Blink Slow
      // Move to ARMED if we are stable? Or just direct to ARMED.
      // Simple logic: Go to ARMED after calibration.
      currentState = STATE_ARMED;
      tone(PIN_BUZZER, 1000, 100);
      break;
      
    case STATE_ARMED:
      digitalWrite(PIN_LED, HIGH); // Solid ON
      // Detect Launch: Accel > Threshold for continuous samples
      if (accel > ACCEL_LAUNCH_THRESHOLD) {
         launchDetectCount++;
         if (launchDetectCount >= 3) {
            currentState = STATE_POWERED_FLIGHT;
            stateStartTime = now;
            dataPacket.state = STATE_POWERED_FLIGHT; 
            Serial.println("LAUNCH!");
         }
      } else {
         launchDetectCount = 0;
      }
      break;

    case STATE_POWERED_FLIGHT:
      // Burnout detection: Accel drops (this is hard if drag is high).
      // Time failsafe is good.
      if (now - stateStartTime > 5000) { // 5s timeout
         currentState = STATE_COASTING;
      }
      // Accel drop detect: if accel < 0 (drag starts slowing it down)
      if (accel < 0) {
         currentState = STATE_COASTING;
      }
      break;

    case STATE_COASTING:
      // Apogee Detect: Velocity goes near zero or negative
      if (vel < APOGEE_VEL_THRESHOLD && vel > -APOGEE_VEL_THRESHOLD) {
          // Near zero? Or just when it turns negative?
          // Robust: vel < 0.
      }
      if (vel < -1.0) { // Definitely coming down
         currentState = STATE_APOGEE;
         stateStartTime = now;
         Serial.println("APOGEE!");
         fireEjection(now);
      }
      break;

    case STATE_APOGEE:
      // Deployment is instant event, then move to Descent
      // Just wait for burn time?
      if (now - mosfetStartTime > EJECTION_BURN_TIME + 500) {
        currentState = STATE_DESCENT;
      }
      break;

    case STATE_DESCENT:
      // Landing Detect
      if (abs(alt - groundAltitude) < LANDING_ALT_THRESHOLD && abs(vel) < 0.5) {
        if (landingStableStartTime == 0) landingStableStartTime = now;
        if (now - landingStableStartTime > LANDING_DETECT_TIME) {
          currentState = STATE_LANDED;
          tone(PIN_BUZZER, 3000, 2000); // Long beep
        }
      } else {
        landingStableStartTime = 0;
      }
      break;

    case STATE_LANDED:
      // Siren
      if ((now / 500) % 2 == 0) tone(PIN_BUZZER, 2000);
      else noTone(PIN_BUZZER);
      
      // Stop LoRa? Prompt says "Stop LoRa".
      // But maybe we want beacon? Prompt: "Action: Stop LoRa; start 'Siren' on Buzzer."
      // I will restrict LoRa in main loop based on state if strictly followed, but usually beacon is good.
      // I'll leave LoRa running for recovery coordinates if requested, but prompt says "Stop LoRa".
      // I will skip sendTelemetry if STATE_LANDED.
      break;
  }
  dataPacket.state = (uint8_t)currentState;
}

void fireEjection(unsigned long now) {
  digitalWrite(PIN_MOSFET, HIGH);
  mosfetActive = true;
  mosfetStartTime = now;
  Serial.println("PYRO FIRE!");
}

void checkSafety(unsigned long now) {
  if (mosfetActive && (now - mosfetStartTime > EJECTION_BURN_TIME)) {
    digitalWrite(PIN_MOSFET, LOW);
    mosfetActive = false;
  }
}

void sendTelemetry() {
  if (currentState == STATE_LANDED) return;

  // Populate Data
  dataPacket.timestamp = millis();
  dataPacket.state = (uint8_t)currentState;
  dataPacket.alt_cm = (int16_t)(fkalman.pos * 100);
  dataPacket.vel_cm = (int16_t)(fkalman.vel * 100);
  dataPacket.acc_mg = (int16_t)((dataPacket.acc_mg / 1000.0) * 1000.0); // Just copying existing var, optimizing:
  // Recalculate acc_mg from physics loop or just trust the global one.
  // We need to make sure dataPacket.acc_mg was set in runPhysics correctly.
  
  dataPacket.max_alt_cm = (int16_t)(maxAltitude * 100);

  // Calculate Checksum (XOR of all bytes)
  dataPacket.checksum = 0;
  uint8_t *ptr = (uint8_t*)&dataPacket;
  for (int i = 0; i < sizeof(RocketData) - 1; i++) {
    dataPacket.checksum ^= ptr[i];
  }

  // Send
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&dataPacket, sizeof(RocketData));
  LoRa.endPacket();

  // Visual Feedback
  digitalWrite(PIN_LED, HIGH);
  delay(5);
  digitalWrite(PIN_LED, LOW);
  
  // Debug
  if (currentState < STATE_POWERED_FLIGHT) {
      Serial.print("T:"); Serial.print(dataPacket.timestamp);
      Serial.print(" Alt:"); Serial.println(dataPacket.alt_cm);
  }
}
