#ifndef _IMU_H_
#define _IMU_H_


class IMU;
#include "Base.h"

class IMU {
 public:
  AP_ADC_ADS7844 adc;

  IMU ();

  static const uint8_t maps_sensor[6];
  static const bool signs_sensor[6];
  static const float kP_rollpitch, kI_rollpitch, kP_yaw, kI_yaw;
  static const float gains_gyro[3];
  static const float gain_accel;

  Vector3f vec_corr_I, vec_corr_P,
           vec_corr_C, vec_corr;
  Matrix3f mat_dcm;

  Vector3f vec_accel, vec_gyro,
           vec_offset_accel, vec_offset_gyro;

  float roll, pitch, yaw;

  void init ();
  void calibrate ();
  
  void update ();
  float update_raw ();
  void update_dcm (float dt);
  
} imu;

#endif
