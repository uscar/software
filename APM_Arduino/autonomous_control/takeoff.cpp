#include "takeoff.h"

bool Takeoff::ExecuteCycle() {
  update_curr_height();
  if(curr_height() < END_HEIGHT) {
    flight_control()->execute(up_cntrl(), CalculateThrottle(), 0);
    return false;
  }
  else return true;
}

/** TODO: Calculate necessary thrust value from current height **/
float Takeoff::CalculateThrottle() {
  set_curr_throttle(THROTTLE_MID);
  return curr_throttle();
}
