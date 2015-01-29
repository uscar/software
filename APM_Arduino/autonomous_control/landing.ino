#include "landing.h"

bool Landing::ExecuteCycle() {
  update_curr_height();
	if(curr_height() > END_HEIGHT) {
	  flight_control()->execute(up_cntrl(), CalculateThrust(), 0); //May be incorrect
    return false;
  }
	else {
    return true;
  }
}

int Landing::CalculateThrust() {
  return 1;
}
