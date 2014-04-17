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

struct kPID{
    float P;
    float I;
    float D;
    float Imax;
};

class Flight_Control {
public:
	Flight_Control();
	void arm(bool);
	void execute(Vector3f up, float throttle, float yaw = 0);

        void setRollPID(kPID rPid);
        kPID getRollPID();
        void setPitchPID(kPID pPid);
        kPID getPitchPID();
        void setYawPID(kPID yPid);
        kPID getYawPID();
        
        void setGyrFactor(int f);
        int getGyrFactor();

private:
	bool armed;
	RC_Channel m_roll(2), m_pitch(3), m_throttle(1), m_yaw(4);
	AP_MotorsQuad motors;
	AC_PID* pid;
	RangeFinder ultrasonic;
	AC_PID pid_roll     (r_p,r_i,r_d,r_imax);
	AC_PID pid_pitch    (p_p,p_i,p_d,p_imax);
	AC_PID pid_throttle (t_p,t_i,t_d,t_imax);
	AC_PID pid_yaw      (y_p,y_i,y_d,y_imax);

	kPID roll_pid  = {0.125,0.00,0.008,4};
	kPID pitch_pid = {0.125,0.00,0.008,4};
	kPID yaw_pid   = {5.000,0.005,0.000,8};
        int gyrErrScale = 150;
};

#endif
