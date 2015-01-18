#ifndef LANDING_H_
#define LANDING_H_

#define END_HEIGHT 0 //Needs to be changed when end height for landing is known

#include "routine.h"

class Landing : public Routine{
public:
	Landing(Flight_Control* flight_control) : Routine(flight_control),
												curr_height_(/*Read from Sensor*/),
												up_cmd_(0, 0, 1) { }; //May be incorrect
	~Landing() { };

	bool ExecuteCycle();

	float curr_height() { return curr_height_; }
	void set_curr_height(float curr_height) { curr_height_ = curr_height; }
	Vector3f& up_cmd() { return up_cmd_; }

private:
	float curr_height_;
	Vector3f up_cmd_;
};

#endif