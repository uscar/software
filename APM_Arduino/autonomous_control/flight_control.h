#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

#include <AP_ADC_HIL.h>
#include <AP_ADC.h>

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_InertialSensor.h>
#include <GCS_MAVLink.h>

#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>

#include <Filter.h>
#include <LowPassFilter2p.h>

#include <AP_Notify.h>

#include <PID.h> //pid controller
#include <AC_PID.h>

#include "constants.h"

extern const AP_HAL::HAL& hal;
extern AP_InertialSensor_MPU6000 ins;

struct kPID {
  float P;
  float I;
  float D;
  float Imax;

  kPID(float p, float i, float d, float max) : P(p), I(i), D(d), Imax(max) { }
};

// enum kDebug
// {
//   DEBUG_ALL,
//   DEBUG_GYRERR,
//   DEBUG_ACCERR,
//   DEBUG_ACTUAL,
//   DEBUG_CONTROL,
//   DEBUG_PID,
//   DEBUG_CNTRL,
//   DEBUG_PULSE,
//   DEBUG_OUTPUT,
//   DEBUG_MOTOR,
//   DEBUG_ENABLE,
//   DEBUG_PWM
// }

class Flight_Control {
public:
  Flight_Control();
  
  ~Flight_Control() { }
  void execute(Vector3f& cntrl_up, float cntrl_throttle, float cntrl_yaw = 0);

  void setRollPID(kPID& rPid) { setACPid(pid_roll, rPid); }
  kPID& getRollPID() { return rPid; }
  void setPitchPID(kPID& pPid) { setACPid(pid_pitch, pPid); }
  kPID& getPitchPID() { return pPid; }
  void setThrottlePID(kPID& tPid) { setACPid(pid_throttle, tPid); }
  kPID& getThrottlePID() { return tPid; }
  void setYawPID(kPID& yPid) { setACPid(pid_yaw, yPid); }
  kPID& getYawPID() { return yPid; }

  void set_gyr_err_scale(float scale) { gyr_err_scale_ = scale; }
  float gyr_err_scale() const { return gyr_err_scale_; };
  
  void set_armed(bool armed);
  bool is_armed() const { return armed; }

private:
  void setACPid(AC_PID& ac_pid, kPID& pid) {
    ac_pid.kP(pid.P); 
    ac_pid.kI(pid.I); 
    ac_pid.kD(pid.D); 
  }
  
  bool armed;
  RC_Channel m_roll, m_pitch, m_throttle, m_yaw;
  AP_MotorsQuad motors;
  
  kPID rPid, pPid, tPid, yPid; 
  AC_PID pid_roll, pid_pitch, pid_throttle, pid_yaw;
  
  Vector3f acc_offset;

  float curr_height();
  float gyr_err_scale_;
  int timestamp;
};

#endif
