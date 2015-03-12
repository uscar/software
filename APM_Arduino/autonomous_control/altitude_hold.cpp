#include "altitude_hold.h"

bool Altitude_Hold::ExecuteCycle() {
  int t = hal.scheduler->micros();
  float dt = (t - timestamp()) / 1000.0;
  set_timestamp(t);

  update_curr_height();
  float height_err = target_height() - curr_height();
  int height_correction = height_pid.get_pid(height_err, dt);
  float throttle = hover_throttle() + height_correction * THROTTLE_SCALE;
  set_hover_throttle(throttle);

  flight_control()->execute(up_cntrl(), throttle, 0);
  return false;
}
