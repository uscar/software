#include "landing.h"

bool Landing::ExecuteCycle() {
  set_curr_height(baro.get_altitude());
	if(curr_height() > END_HEIGHT) {
		flight_control()->execute(up_cntrl(), CalculateThrust(), 0); //May be incorrect
		return false;
	}
	else {
    flight_control()->arm(false);
    return true;
  }
}

int Landing::CalculateThrust() {
  return 1;
}
