#include "takeoff.h"
#include <Vector3f>

extern AP_InertialSensor_MPU6000 ins;
extern const AP_HAL::HAL& hal;

bool Takeoff::ExecuteCycle() {
  if(curr_height() < END_HEIGHT) {
    flight_control()->execute(up_cmd(), 1, 0);
    return false;
  }
  else return true;
}
