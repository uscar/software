#include "IMU.h"

float const IMU::kP_rollpitch = 0.13, IMU::kI_rollpitch = 0.000005,
            IMU::kP_yaw = 0.4, IMU::kI_yaw = 0.00005;
            
uint8_t const IMU::maps_sensor[6]= {1,2,0,4,5,6};
bool const IMU::signs_sensor[6] = {1,0,0,1,0,0};
float const IMU::gains_gyro[3] = {ToRad(0.4), ToRad(0.41), ToRad(0.41)};
float const IMU::gain_accel = 9.80665 / 423.8;

IMU::IMU () {
  mat_dcm.rotation(ROTATION_NONE);
}

void IMU::init () {
  adc.Init(&base.adc_scheduler);
  calibrate();
}

void IMU::calibrate () {
  float avg[6] = {0,0,0,0,0,0}; size_t i;

  // Clear first few unstable readings.
  for (i = 0; i < 100; ++i)
    for (size_t j = 0; j < 6; ++j)
      adc.Ch(maps_sensor[j]);

  // Get readings for averaging.
  for (i = 0; i < 1000; ++i)
    for (size_t j = 0; j < 6; ++j)
      avg[j] += adc.Ch(maps_sensor[j]);
  
  // Calcualte the average and do scaling.
  for (size_t j = 0; j < 6; ++j) {
    avg[j] /= i;
    if (!signs_sensor[j]) avg[j] = - avg[j];
  }
  for (size_t j = 0; j < 3; ++j) avg[j] *= gains_gyro[j];
  for (int i = 3; i < 6; i++) avg[i] *= gain_accel;

  // Store the offset vectors.
  vec_offset_gyro = Vector3f(avg[0], avg[1], avg[2]);
  vec_offset_accel = Vector3f(avg[3], avg[4], avg[5] + 9.805);
}

void IMU::update () { update_dcm(update_raw()); }

float IMU::update_raw () {
  float data[6];
  
  // Get data and do scaling.
  float dt = adc.Ch6(maps_sensor, data) * 1.0e-6;
  for (int i = 0; i < 6; i++) if (!signs_sensor[i]) data[i] = - data[i];
  for (int i = 0; i < 3; i++) data[i] *= gains_gyro[i];
  for (int i = 3; i < 6; i++) data[i] *= gain_accel;

  // Take out the offset.
  vec_gyro = Vector3f(data[0] ,data[1], data[2]) - vec_offset_gyro;
  vec_accel = Vector3f(data[3], data[4], data[5]) - vec_offset_accel;

  return dt;
}

void IMU::update_dcm (float dt) {

  if (dt > 0.1) return;

  // Update DCM matrix.
  vec_corr_C = vec_gyro + vec_corr_I;
  vec_corr = vec_corr_C + vec_corr_P;
  mat_dcm.rotate(vec_corr * dt);

  // Normalize.
  Matrix3f mat_tmp;
  float error = mat_dcm.a * mat_dcm.b * 0.5;
  mat_tmp.a = mat_dcm.a - mat_dcm.b * error;
  mat_tmp.b = mat_dcm.b - mat_dcm.a * error;
  mat_tmp.c = mat_tmp.a % mat_tmp.b;
  mat_dcm = Matrix3f(mat_tmp.a * (3 - mat_tmp.a * mat_tmp.a) * 0.5,
                     mat_tmp.b * (3 - mat_tmp.b * mat_tmp.b) * 0.5,
                     mat_tmp.c * (3 - mat_tmp.c * mat_tmp.c) * 0.5);

  // Correct. 
  Vector3f vec_error_rot = mat_dcm.c % vec_accel;
  vec_corr_P = vec_error_rot * kP_rollpitch;
  vec_corr_I += vec_error_rot * kI_rollpitch;

  // Update euler.
  mat_dcm.to_euler(&roll, &pitch, &yaw);
}

