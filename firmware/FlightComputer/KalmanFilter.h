#ifndef KalmanFilter2D_h
#define KalmanFilter2D_h

#include <math.h>

class KalmanFilter2D {
public:
  // State: [position, velocity]
  float pos;
  float vel;

  // Covariance Matrix P
  // [ P00 P01 ]
  // [ P10 P11 ]
  float P00, P01, P10, P11;

  // Process Noise Q (TUNING)
  float Q_pos; // Variance of position process noise
  float Q_vel; // Variance of velocity process noise (accelerometer noise integration)

  // Measurement Noise R (TUNING)
  float R_alt; // Variance of barometer measurement

  KalmanFilter2D() {
    // Initial State
    pos = 0.0f;
    vel = 0.0f;
    
    // Initial Covariance (High uncertainty initially)
    P00 = 100.0f; P01 = 0.0f;
    P10 = 0.0f;   P11 = 100.0f;

    // Default Tuning (Can be adjusted)
    Q_pos = 0.01f;
    Q_vel = 0.1f;
    R_alt = 0.5f; // Barometer noise variance
  }

  void begin(float initial_alt, float q_pos, float q_vel, float r_alt) {
    pos = initial_alt;
    vel = 0.0f;
    Q_pos = q_pos;
    Q_vel = q_vel;
    R_alt = r_alt;
  }

  // Prediction Step (Physics Update)
  // u = acceleration (m/s^2)
  // dt = time step (seconds)
  void predict(float u, float dt) {
    // State Extrapolation: x = Fx + Bu
    // F = [1, dt; 0, 1]
    // B = [0.5*dt^2; dt]
    float old_pos = pos;
    float old_vel = vel;
    
    pos = old_pos + old_vel * dt + 0.5f * u * dt * dt;
    vel = old_vel + u * dt;

    // Covariance Extrapolation: P = FPF' + Q
    // FPF' expansion for 2x2:
    float dt2 = dt * dt;
    float next_P00 = P00 + dt * (P10 + P01) + dt2 * P11 + Q_pos;
    float next_P01 = P01 + dt * P11;
    float next_P10 = P10 + dt * P11;
    float next_P11 = P11 + Q_vel;

    P00 = next_P00;
    P01 = next_P01;
    P10 = next_P10;
    P11 = next_P11;
  }

  // Update Step (Correction from Barometer)
  // z = measured altitude
  void update(float z) {
    // Observation Matrix H = [1, 0]
    // Innovation (Residual) y = z - Hx
    float y = z - pos;

    // Innovation Covariance S = HPH' + R
    float S = P00 + R_alt;

    // Kalman Gain K = PH' * inv(S)
    float K0 = P00 / S;
    float K1 = P10 / S;

    // State Correction: x = x + Ky
    pos = pos + K0 * y;
    vel = vel + K1 * y;

    // Covariance Correction: P = (I - KH)P
    float old_P00 = P00;
    float old_P01 = P01;
    
    P00 = P00 - K0 * old_P00;
    P01 = P01 - K0 * old_P01;
    P10 = P10 - K1 * old_P00;
    P11 = P11 - K1 * old_P01;
  }
};

#endif
