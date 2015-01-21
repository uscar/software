#ifndef LANDING_H_
#define LANDING_H_

// #define END_HEIGHT .15 //Needs to be changed when end height for landing is known
#include <AP_Baro.h>

#include "routine.h"

extern AP_Baro_MS5611 baro;

class Landing : public Routine {
public:
	Landing(Flight_Control* flight_control, int routine_code)
												: Routine(flight_control, routine_code),
												up_cntrl_(0, 0, 1) { }; //May be incorrect
	~Landing() { };

	bool ExecuteCycle();

	int CalculateThrust();

	float curr_height() { return curr_height_; }
	void set_curr_height(float curr_height) { curr_height_ = curr_height; }
	Vector3f& up_cntrl() { return up_cntrl_; }

private:
	float curr_height_;
	static const float END_HEIGHT = .15;
	Vector3f up_cntrl_;
};

#endif
