#ifndef LANDING_H_
#define LANDING_H_

#include "routine.h"

extern AP_Baro_MS5611 baro;

class Landing : public Routine {
public:
	Landing(Flight_Control* flight_control, int routine_code)
												: Routine(flight_control, routine_code, "LANDING"),
												up_cntrl_(0, 0, 1) { }; //May be incorrect
	~Landing() { };

	bool ExecuteCycle();

	int CalculateThrust();
		
	Vector3f& up_cntrl() { return up_cntrl_; }
private:
	float curr_height_;
	static const float END_HEIGHT = 0.05;
	Vector3f up_cntrl_;
};

#endif
