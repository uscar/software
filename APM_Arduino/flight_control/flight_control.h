#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

#include <AP_ADC_HIL.h>
#include <AP_ADC.h>

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_InertialSensor.h>
#include <GCS_MAVLink.h>

#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>

#include <Filter.h>
#include <LowPassFilter2p.h>

#include <AP_Notify.h>

#include <PID.h> //pid controller
#include <AC_PID.h>

#include <RangeFinder.h>

extern AP_HAL::HAL& hal;
extern AP_InertialSensor_MPU6000 ins;

struct gPID{
	float P;
	float I;
	float D;
	float Imax;
	gPID(float p, float i, float d, float max){
		P = p;
		I = i;
		D = d;
		Imax = max;
	}
};

enum DEBUG
{
	DEBUG_ALL,
	DEBUG_GYRERR,
	DEBUG_ACCERR,
	DEBUG_ACTUAL,
	DEBUG_CONTROL,
	DEBUG_PID,
	DEBUG_CNTRL,
	DEBUG_PULSE,
	DEBUG_OUTPUT,
	DEBUG_MOTOR,
	DEBUG_ENABLE,
	DEBUG_PWM
};

class Flight_Control {
public:
	Flight_Control();
	void arm(bool armed);
	void execute(Vector3f cntrl_up, float cntrl_throttle, float cntrl_yaw = 0);

	void setRollPID(gPID rPid);
	gPID getRollPID();
	void setPitchPID(gPID pPid);
	gPID getPitchPID();
	void setYawPID(gPID yPid);
	gPID getYawPID();

	void setGyrFactor(int f);
	int getGyrFactor();

private:
	bool armed;
	RC_Channel m_roll(uint8_t r = 2), m_pitch(uint8_t p = 3), m_throttle(uint8_t t = 1), m_yaw(uint8_t y = 4);
	
	AP_MotorsQuad motors(RC_Channel & m = &Flight_Control::m_roll, RC_Channel & p = &Flight_Control::m_pitch, RC_Channel & t = &Flight_Control::m_throttle, RC_Channel & y = &Flight_Control::m_yaw);

	gPID rPid (float p = 0.125, float i = 0.00, float d = 0.008, float m = 4);
	gPID pPid (float p = 0.125, float i = 0.00, float d = 0.008, float m = 4);
	gPID yPid (float p = 5.000, float i = 0.005, float d = 0.000, float m = 8);
	gPID tPid (float p = 1.0, float i = 0.001, float d = 0.02, float m = 0.5);

	AC_PID pid_roll     (float p = rPid.P, float i = rPid.I, float d = rPid.D, float m = rPid.Imax);
	AC_PID pid_pitch    (float p = pPid.P, float i = pPid.I, float d = pPid.D, float m = pPid.Imax);
	AC_PID pid_throttle (float p = tPid.P, float i = tPid.I, float d = tPid.D, float m = tPid.Imax);
	AC_PID pid_yaw      (float p = yPid.P, float i = yPid.I, float d = yPid.D, float m = yPid.Imax);

	int gyrErrScale;

	int timestamp;

	Vector3f acc_offset(float x = -.18, float y = .66, float z = 1.1);

	static const int CNTRL_RANGE = 1000;
};

#endif
