#include "landing.h"
#include <Vector3f>

extern AP_InertialSensor_MPU6000 ins;
extern const AP_HAL::HAL& hal;
bool Landing::ExecuteCycle() {
	if(curr_height() > END_HEIGHT) {
		flight_control() -> execute(up_cmd(), 1, 0); //May be incorrect
		return false;
	}
	else return true;
}

