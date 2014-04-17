/***********************************NOTES***************************************

RC_Channel:
radio_in = the pwm value given in
control_in = the range mapped output value 
.output() doesn't need to be called (it is for reading from a hal rc
radio_max = max input value
radio_min = min input value
.set_range(int min, int max) : min = min output, max = max output
AP_Motors:
uses the radio_in     
 *******************************************************************************/


//General board dependencies
#include <AP_ADC_HIL.h>
#include <AP_ADC.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

//IMU sensor dependencies
#include <AP_InertialSensor.h>
#include <GCS_MAVLink.h>
#include <AP_Notify.h>

//Motor output dependencies
#include <RC_Channel.h> 
#include <AP_Motors.h>
#include <AP_Curve.h>

//Algorithm tools
#include <LowPassFilter2p.h>
#include <AC_PID.h>

//Custom tools
#include "calibration.h"
#include "printing.h"

//board MUST be declared like this for libraries to function (they use extern)
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_InertialSensor_MPU6000 ins;

//setup the filters for removing noise from acc.
static LowPassFilter2p filt_x(update_hz,cutout);
static LowPassFilter2p filt_y(update_hz,cutout);
static LowPassFilter2p filt_z(update_hz,cutout);

//set up the PID calculators with the hard coded values
AC_PID pid_roll     (r_p,r_i,r_d,r_imax);
AC_PID pid_pitch    (p_p,p_i,p_d,p_imax);
AC_PID pid_throttle (t_p,t_i,t_d,t_imax);
AC_PID pid_yaw      (y_p,y_i,y_d,y_imax);

//needed by the AP_Motors type to define inputs
RC_Channel m_roll(2), m_pitch(3), m_throttle(1), m_yaw(4);

//vector command storage
Vector3f cntrl_up;
Vector3f old_cntrl_up;

//non-vector command storage
float cntrl_yaw;
float old_cntrl_yaw;
int cntrl_throttle;

//motors abstractions handles the mapping of command to motor output
//CAREFULL -> the motors use the pwm in value not the mapped value.
AP_MotorsQuad motors(&m_roll,&m_pitch,&m_throttle,&m_yaw);

//used for calculating update rate, time elapsed etc
int timestamp;
int t0;
int counter = 0;

int  armed;
//reads remote control inputs and sets c vals
void read_rc_inputs(){

    int roll        = hal.rcin->read(CH_1);
    int pitch       = hal.rcin->read(CH_2);
    int throttle    = hal.rcin->read(CH_3);
    int yaw         = hal.rcin->read(CH_4);

    if (yaw < REMOTE_MIN + 50 && throttle < REMOTE_MIN +50){
        armed = true;
    }
    if (yaw > REMOTE_MAX - 50 && throttle < REMOTE_MIN+50){
        armed = false;
    }


    float range = REMOTE_MAX-REMOTE_MIN;

    //calculate control value from input value
    cntrl_up.x = (INV_PITCH?-1:1)*(pitch - REMOTE_TRIM)*2.0/range;
    cntrl_up.y = (INV_ROLL?-1:1)*(roll - REMOTE_TRIM)*2.0/range;
    cntrl_up.z = CNTRL_Z_COMP; //dampens the control.

    cntrl_up = cntrl_up.normalized();
    cntrl_yaw  = (INV_YAW?-1:1)*(yaw - REMOTE_TRIM)*2.0/range;
    cntrl_throttle = (throttle - REMOTE_MIN);
}   

//linearly maps a value within one range to another range
int lin_map(int value, int min_v, int max_v, int min_o, int max_o){
    return (int)((value-min_v)*((float)(max_o-min_o)/(float)(max_v-min_v))+min_o);
}

void setup_m_rc(){

    //setting to less than 1000 scales up control effect
    m_throttle.set_range(0,1000);//should be 1000 probs.
    m_roll.set_range    (0,1000);
    m_pitch.set_range   (0,1000);
    m_yaw.set_range     (0,1000);

    m_throttle.radio_min = 1000;
    m_throttle.radio_max = 2200;

    m_roll.servo_out = 0;
    m_pitch.servo_out = 0;
    m_yaw.servo_out = 0;
}

//called once on reset
void setup(void)
{
    armed = false;
    ins.init(AP_InertialSensor::COLD_START,AP_InertialSensor::RATE_100HZ);

    // HAL will start serial port at 115200.
    hal.console->println_P(PSTR("Starting!"));

    //defines the mapping system from RC command to motor effect (defined in quad)
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);//motors.set_frame_orientation(AP_MOTORS_H_FRAME);
    //motors.min_throttle is in terms of servo values not output values.

    //setup rc
    setup_m_rc();

    //setup motors
    motors.Init();
    motors.set_update_rate(500);
    motors.enable();

    //setup timing
    t0 = timestamp = hal.scheduler->micros();

    //kill the barometer
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40,1);

    ///not the right place but it is a nice thought    
    old_cntrl_up = cntrl_up;
    old_cntrl_yaw = cntrl_yaw;
}

//called itteratively
void loop(void){
    //for periodic outputs
    counter++;
    counter %= 50;
    if(armed){
        motors.armed(true);
    }
    else{
        motors.armed(false);
    }

    //update the time locks/ time updates
    int t = hal.scheduler->micros();
    float dt = (t - timestamp)/1000.0;
    timestamp = t;


    //get new instrument measurement
    ins.update();
    /*
       if(hal.console->available()){
       char c = hal.console->read();
       char d = hal.console->read();
       if(d == '='){
       if(c == 'p'){
       float kp;
       hal.console->scan("%f",&kp);
       pid_roll.kP(kp);
       pid_pitch.kP(kp);
       }
       if(c == 'd'){
       float kd = hal.console->parseFloat();
       pid_roll.kD(kd);
       pid_pitch.kD(kd);
       }
       if(c == 'i'){
       float ki = hal.console->parseFloat();
       pid_roll.kI(ki);
       pid_pitch.kI(ki);
       }
       }
       while(hal.console->available()){
       hal.console->read();
       }
       }
     */
    //compute the control value coresponding to current IMU output
    //should be scaled to -100 -> 100 for now
    Vector3f acc = ins.get_accel() - acc_offset;
    Vector3f gyr = ins.get_gyro();
    Vector3f filtered_acc;

    //apply a low pass filter    
    //    filtered_acc.x = filt_x.apply(acc.x);
    //    filtered_acc.y = filt_y.apply(acc.y);
    //    filtered_acc.z = filt_z.apply(acc.z);

    //turn off the filtering
    filtered_acc.x = acc.x;
    filtered_acc.y = acc.y;
    filtered_acc.z = acc.z;


    //normalize (length = 1)
    Vector3f down = filtered_acc.normalized(); // may break in free fall...

    if(DEBUG_FILTER || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("down = %3.3f %3.3f %3.3f\n",down.x,down.y,down.z);
    }

    Vector3f err = down + cntrl_up;

    //make response smaller near center
    //    err.x = (1-cos(err.x*PI))*(abs(err.x)/err.x)*.1+err.x*.9;
    //    err.y = (1-cos(err.y*PI))*(abs(err.y)/err.y)*.1+err.y*.9;





    //take appropriate componenets and scale
    int roll_actual = (-1)*down.y*CNTRL_RANGE;
    int pitch_actual = down.x*CNTRL_RANGE;
    int throttle_actual = cntrl_throttle; //TODO: change this out with some f(height)
    int yaw_actual = (-1)*gyr.z*YAW_SCALE;                  //this may require some scaling


    read_rc_inputs();

    //    Vector3f d_cntrl = (cntrl_up - old_cntrl_up)/dt;
    old_cntrl_up = cntrl_up;
    //    float d_cntrl_yaw = (cntrl_yaw - old_cntrl_yaw)/dt;
    old_cntrl_yaw = cntrl_yaw;
    Vector3f gyr_err = (gyr)*GYR_ERR_SCALE;
    //TODO: consider adding a gyr_err  threshold value.

    //    if(counter==0){
    //      printv3f(d_cntrl);
    //      printv3f(gyr);
    //      printv3f(gyr_err);
    //    }


    int int_cntrl_roll      = cntrl_up.y*CNTRL_RANGE;
    int int_cntrl_pitch     = cntrl_up.x*CNTRL_RANGE;
    int int_cntrl_yaw       = cntrl_yaw*CNTRL_RANGE;
    int int_cntrl_throttle  = cntrl_throttle;

    int roll_error     =  err.y*CNTRL_RANGE ;
    int pitch_error    =  err.x*CNTRL_RANGE ;
    int throttle_error =  cntrl_throttle - throttle_actual;
    int yaw_error      =  int_cntrl_yaw - yaw_actual;
    int pitch_pid_err = pid_pitch.get_pid(pitch_error,dt);
    int roll_pid_err = pid_roll.get_pid(roll_error,dt);
    if(DEBUG_GYRERR || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("gyr_err roll = %i pitch = %i\n",int((-1)*error_scale*gyr_err.x),int(error_scale*gyr_err.y));
    }
    if(DEBUG_ACCERR || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("acc_err roll = %04i pitch = %04i\n", error_scale*roll_pid_err,error_scale*pitch_pid_err);
    }
    int r_correction = error_scale*(roll_pid_err - int(gyr_err.x));
    int p_correction = error_scale*(pitch_pid_err + int(gyr_err.y));
    int t_correction = error_scale*pid_throttle.get_pid(throttle_error,dt);
    int y_correction = error_scale*pid_yaw.get_pid(yaw_error,dt);


    if(DEBUG_ACTUAL || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("actual: roll= %04i, pitch= %04i, throttle= %04i, yaw= %04i\n",roll_actual,pitch_actual,throttle_actual,yaw_actual);
    }

    if(DEBUG_CONTROL || DEBUG_ALL){
        if(counter == 0){
            hal.console->printf("cntrl: roll= %04i, pitch= %04i, throttle= %04i, yaw= %04i\n",int_cntrl_roll, int_cntrl_pitch, int_cntrl_throttle,int_cntrl_yaw); 
        }
    }

    if(DEBUG_PID || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("correction: roll = %04i, pitch = %04i, throttle = %04i, yaw = %04i\n",r_correction,p_correction,t_correction,y_correction);
    }

    int roll_out = int_cntrl_roll + r_correction;
    int pitch_out = int_cntrl_pitch - p_correction;
    int throttle_out = int_cntrl_throttle + t_correction;
    int yaw_out = int_cntrl_yaw - y_correction;

    if(DEBUG_CNTRL || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("out: roll = %04i, pitch = %04i, throttle = %04i, yaw = %04i\n",roll_out,pitch_out,throttle_out,yaw_out);
    }

    //adjust motor outputs approrpiately for the pid term
    int roll_pulse = roll_out;
    int pitch_pulse = pitch_out;
    int throttle_pulse = throttle_out;
    int yaw_pulse = yaw_out;

    m_roll.servo_out = roll_pulse;
    m_pitch.servo_out = pitch_pulse;
    m_throttle.servo_out = throttle_pulse;
    m_yaw.servo_out = yaw_pulse;

    if(DEBUG_PULSE || DEBUG_ALL){
        if(counter == 0){
            hal.console->printf("pulses roll=%4i pitch=%4i throttle=%4i yaw=%4i\n",roll_pulse,pitch_pulse,throttle_pulse,yaw_pulse);
        } 
    }

    motors.output();



    if(DEBUG_OUTPUT|| DEBUG_ALL){
        if(counter == 0){
            hal.console->printf("servo output roll= %04i, pitch= %04i, throttle= %04i, yaw= %04i\n",m_roll.servo_out, m_pitch.servo_out, m_throttle.servo_out,m_yaw.servo_out); 
            hal.console->printf("pwm   output roll= %04i, pitch= %04i, throttle= %04i, yaw= %04i\n",m_roll.pwm_out, m_pitch.pwm_out, m_throttle.pwm_out,m_yaw.pwm_out);    
        }
    }
    if(DEBUG_MOTOR || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("motor one:%i two:%i three:%i four:%i\n",(int)motors.motor_out[0], (int)motors.motor_out[1], (int)motors.motor_out[2], (int)motors.motor_out[3]);
    }
    if(DEBUG_ENABLE || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("enabled one:%i two:%i three:%i four:%i\n",(int)motors.motor_enabled[0], (int)motors.motor_enabled[1], (int)motors.motor_enabled[2], (int)motors.motor_enabled[3]);
    }
    if(DEBUG_PWM|| DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("pwm roll:%i pitch:%i throttle:%i yaw:%i\n",(int)m_roll.servo_out, (int)m_pitch.servo_out, (int)m_throttle.servo_out, (int)m_yaw.servo_out);
    }


    //TODO:provide user interface for calibrating the accelerometer
    //also same thing for pid stuff
}



//calls setup and loop appropriately
AP_HAL_MAIN();
