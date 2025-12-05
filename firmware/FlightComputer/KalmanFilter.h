#ifndef SimpleKalmanFilter_h
#define SimpleKalmanFilter_h

class SimpleKalmanFilter {
public:
  SimpleKalmanFilter(float mea_e, float est_e, float q);
  float updateEstimate(float mea);
  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
  float getKalmanGain();
  float getEstimateError();

private:
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate = 0;
  float _last_estimate = 0;
  float _kalman_gain = 0;
};

SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q) {
  _err_measure = mea_e;
  _err_estimate = est_e;
  _q = q;
}

float SimpleKalmanFilter::updateEstimate(float mea) {
  _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + _last_estimate * _q; // simplified
  // Note: The standard formula for error covariance update is P = (1 - K) * P_minus
  // But for process noise addition: P_minus = P_prev + Q
  // So full step:
  // 1. Prediction:
  //    x_pred = x_last
  //    p_pred = p_last + q
  // 2. Update:
  //    k = p_pred / (p_pred + r)
  //    x = x_pred + k * (mea - x_pred)
  //    p = (1 - k) * p_pred
  
  // Re-implementing with standard steps for clarity and correctness
  // Prediction
  // _current_estimate (predicted) is just _last_estimate (assuming constant state model)
  float p_pred = _err_estimate + _q;
  
  // Update
  _kalman_gain = p_pred / (p_pred + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate = (1.0 - _kalman_gain) * p_pred;
  
  _last_estimate = _current_estimate;

  return _current_estimate;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e) {
  _err_measure = mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e) {
  _err_estimate = est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q) {
  _q = q;
}

float SimpleKalmanFilter::getKalmanGain() {
  return _kalman_gain;
}

float SimpleKalmanFilter::getEstimateError() {
  return _err_estimate;
}

#endif
