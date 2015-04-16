#ifndef ALTITUDE_HOLD_H_
#define ALTITUDE_HOLD_H_

#include <AC_PID.h>

#include "flight_control.h"
#include "routine.h"

class Altitude_Hold : public Routine {
public:
  Altitude_Hold(Flight_Control* flight_control, int routine_code)
               : Routine(flight_control, routine_code, "ALT_HOLD"),
                 height_pid(ALT_HOLD_H_P, ALT_HOLD_H_I,
                            ALT_HOLD_H_D, ALT_HOLD_H_IMAX),
								 up_cntrl_(0, 0, 1),
                 hover_throttle_(THROTTLE_MID), target_height_(1) { }
  ~Altitude_Hold() { }
  bool ExecuteCycle();

	Vector3f& up_cntrl() { return up_cntrl_; }

	float hover_throttle() { return hover_throttle_; }
	void set_hover_throttle(float hover_throttle) { hover_throttle_ = hover_throttle; }

  float target_height() { return target_height_; }
	void set_target_height(float target_height) { target_height_ = target_height; }

private:
  static const float THROTTLE_CORRECTION_SCALE = 0.05;
  static const float ALT_HOLD_H_P = 8.00;
  static const float ALT_HOLD_H_I = 0.00;
  static const float ALT_HOLD_H_D = 8.00;
  static const float ALT_HOLD_H_IMAX = 5;

  AC_PID height_pid;
	Vector3f up_cntrl_;
	float hover_throttle_;
  float target_height_;
};

#endif
