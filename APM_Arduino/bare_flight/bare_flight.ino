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



#include <PID.h>

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

#include "calibration.h"
#include "printing.h"


const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
AP_InertialSensor_MPU6000 ins;

//setupt the filters (for getting down)
static LowPassFilter2p filt_x(update_hz,cutout);
static LowPassFilter2p filt_y(update_hz,cutout);
static LowPassFilter2p filt_z(update_hz,cutout);

//this may need to be inside loop for some reason...
AC_PID pid_roll(    r_p,r_i,r_d,r_imax);
AC_PID pid_pitch(   p_p,p_i,p_d,p_imax);
AC_PID pid_throttle(t_p,t_i,t_d,t_imax);
AC_PID pid_yaw(     y_p,y_i,y_d,y_imax);

//rc channel is a useful data type for defining a command.
//not restricted to remote control inputs. Good for computer defined too.
//needed by the AP_Motors type to define inputs
RC_Channel m_roll(2), m_pitch(3), m_throttle(1), m_yaw(4);
RC_Channel c_roll(12), c_pitch(13), c_throttle(13), c_yaw(14);

//motors abstractions handles the mapping of command to motor output

//CAREFULL -> the motors use the pwm in value not the mapped value.
AP_MotorsQuad motors(&m_roll,&m_pitch,&m_throttle,&m_yaw);

int timestamp;
int t0;
int counter = 0;

int tempy = 0;

//sets the initial values as desired (deprecated)
void init_rc(RC_Channel* rc){
    rc->set_range((int16_t)CNTRL_MIN,(int16_t)CNTRL_MAX);
    rc->radio_min = CNTRL_MIN;
    rc->radio_max = CNTRL_MAX;
    rc->set_pwm((int16_t)CNTRL_MIN);
    rc->trim();
    rc->set_pwm((int16_t)CNTRL_MIN);
}

//reads remote control inputs and sets c vals
void read_rc_inputs(){
//    tempy++;
//    tempy%=1000;
//    int val = sin(tempy*PI*2/1000)*400+1500;
//    c_roll.set_pwm(1500);
//    c_pitch.set_pwm(1500);
//    c_throttle.set_pwm(val);
//    c_yaw.set_pwm(1500);
    
    c_roll.set_pwm(hal.rcin->read(CH_1));
    c_pitch.set_pwm(hal.rcin->read(CH_2));
    c_throttle.set_pwm(hal.rcin->read(CH_3));
    c_yaw.set_pwm(hal.rcin->read(CH_4));    
    
}   

//linearly maps a value within one range to another range
int lin_map(int value, int min_v, int max_v, int min_o, int max_o){
    return (int)((value-min_v)*((float)(max_o-min_o)/(float)(max_v-min_v))+min_o);
}

//called once on reset
void setup(void)
{
    ins.init(AP_InertialSensor::COLD_START,AP_InertialSensor::RATE_100HZ);

    // HAL will start serial port at 115200.
    hal.console->println_P(PSTR("Starting!"));

    //defines the mapping system from RC command to motor effect (defined in quad)
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);//motors.set_frame_orientation(AP_MOTORS_H_FRAME);
    //the times per second to update the outpus (from the rc vals)
    motors.set_min_throttle((uint16_t) 130);
    motors.slow_start(false);
    //motors.set_max_throttle((uint16_t) MOTOR_MAX);
    //motors.set_mid_throttle((uint16_t) MOTOR_HOVER);
    motors.Init();
    motors.set_update_rate(500);

    motors.enable();
    motors.output_min();

    motors.armed(true);


    t0 = timestamp = hal.scheduler->micros();

    init_rc(&m_roll);     init_rc(&c_roll);   
    init_rc(&m_pitch);    init_rc(&c_pitch);  
    init_rc(&m_throttle); init_rc(&c_throttle);
    init_rc(&m_yaw);      init_rc(&c_yaw);

    //for reading from the actual remote control input
    //radio_min/max are the input range
    c_roll.radio_min = c_pitch.radio_min = c_throttle.radio_min = c_yaw.radio_min = REMOTE_MIN;
    c_roll.radio_max = c_pitch.radio_max = c_throttle.radio_max = c_yaw.radio_max = REMOTE_MAX;
    c_roll.radio_trim = c_pitch.radio_trim = c_yaw.radio_trim = REMOTE_TRIM; //don't trim the throttle.
    c_throttle.radio_trim = REMOTE_MIN;
    //this min/max defines the output range
    c_roll.set_range(CNTRL_MIN,CNTRL_MAX);
    c_throttle.set_range(CNTRL_MIN,CNTRL_MAX);
    c_yaw.set_range(CNTRL_MIN,CNTRL_MAX);
    c_pitch.set_range(CNTRL_MIN,CNTRL_MAX);


    m_roll.radio_min = m_pitch.radio_min = m_throttle.radio_min = m_yaw.radio_min = MOTOR_MIN;
    m_roll.radio_max = m_pitch.radio_max = m_throttle.radio_max = m_yaw.radio_max = MOTOR_MAX;
    /*
       m_roll.set_range(MOTOR_MIN,MOTOR_MAX);
       m_pitch.set_range(MOTOR_MIN,MOTOR_MAX);
       m_yaw.set_range(MOTOR_MIN,MOTOR_MAX);
       m_throttle.set_range(MOTOR_MIN,MOTOR_MAX);
     */
    //kills the barometer
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40,1);

}

//called itteratively
void loop(void)
{
    //for periodic outputs
    counter++;
    counter %= 10;

    //update the time locks/ time updates
    int t = hal.scheduler->micros();
    float dt = (t - timestamp)/1000.0;
    timestamp = t;

    if(DEBUG_TIMER || DEBUG_ALL){
        hal.console->printf("%f",dt);
        if(counter == 0 )
            hal.console->printf("\n");
    }


    //get new instrument measurement
    ins.update();


    //compute the control value coresponding to current IMU output
    //should be scaled to -100 -> 100 for now
    Vector3f acc = ins.get_accel();
    Vector3f gyr = ins.get_gyro();
    Vector3f filtered_acc;

    //apply a low pass filter    
    filtered_acc.x = filt_x.apply(acc.x);
    filtered_acc.y = filt_y.apply(acc.y);
    filtered_acc.z = filt_z.apply(acc.z);

    //normalize (length = 1)
    Vector3f down = filtered_acc.normalized();

    if(DEBUG_FILTER || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("down = %3.3f %3.3f %3.3f\n",down.x,down.y,down.z);
    }

    //take appropriate componenets and scale
    int roll_actual = down.y*(-1)*(CNTRL_MAX-CNTRL_MIN)+(CNTRL_MAX+CNTRL_MIN)/2;
    int pitch_actual = down.x*(CNTRL_MAX-CNTRL_MIN)+(CNTRL_MAX+CNTRL_MIN)/2;
    int throttle_actual = c_throttle.control_in; //TODO: change this out with some f(height)
    int yaw_actual = gyr.z*(CNTRL_MAX-CNTRL_MIN)+(CNTRL_MAX+CNTRL_MIN)/2;                  //this may require some scaling

    if(DEBUG_ACTUAL || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("actual roll= %04i, actual pitch= %04i, actual throttle= %04i, actual yaw= %04i\n",roll_actual,pitch_actual,throttle_actual,yaw_actual);
    }

    read_rc_inputs();



    int roll_error     =  c_roll.control_in     - roll_actual;
    int pitch_error    =  c_pitch.control_in    - pitch_actual;
    int throttle_error =  c_throttle.control_in - throttle_actual;
    int yaw_error      =  c_yaw.control_in      - yaw_actual;

    int r_correction = error_scale*pid_roll.get_pid(roll_error,dt);
    int p_correction = error_scale*pid_pitch.get_pid(pitch_error,dt);
    int t_correction = error_scale*pid_throttle.get_pid(throttle_error,dt);
    int y_correction = error_scale*pid_yaw.get_pid(yaw_error,dt);

    if(DEBUG_PID || DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("roll correction= %04i, pitch correction= %04i, throttle correction= %04i, yaw correction= %04i\n",r_correction,p_correction,t_correction,y_correction);
    }
    int roll_out = c_roll.control_in + r_correction;
    int pitch_out = c_pitch.control_in + p_correction;
    int throttle_out = c_throttle.control_in + p_correction;
    int yaw_out = c_yaw.control_in + y_correction;
    roll_out = max(roll_out, CNTRL_MIN);
    pitch_out = max(pitch_out, CNTRL_MIN);
    throttle_out = max(throttle_out, CNTRL_MIN);
    yaw_out = max(yaw_out, CNTRL_MIN);

    roll_out = min(roll_out, CNTRL_MAX);
    pitch_out = min(pitch_out, CNTRL_MAX);
    throttle_out = min(throttle_out, CNTRL_MAX);
    yaw_out = min(yaw_out, CNTRL_MAX);

    //adjust motor outputs approrpiately for the pid term
    int roll_pulse = lin_map(roll_out, CNTRL_MIN, CNTRL_MAX, MOTOR_MIN, MOTOR_MAX);
    int pitch_pulse = lin_map(pitch_out, CNTRL_MIN, CNTRL_MAX, MOTOR_MIN, MOTOR_MAX);
    int throttle_pulse = lin_map(throttle_out, CNTRL_MIN, CNTRL_MAX, MOTOR_MIN, MOTOR_MAX);
    int yaw_pulse = lin_map(yaw_out, CNTRL_MIN, CNTRL_MAX, MOTOR_MIN, MOTOR_MAX);

    m_roll.set_pwm(roll_pulse);
    m_pitch.set_pwm(pitch_pulse);
    m_throttle.set_pwm(throttle_pulse);
    m_yaw.set_pwm(yaw_pulse);

    if(DEBUG_PULSE || DEBUG_ALL){
        if(counter == 0){
            hal.console->printf("pulses roll=%4i pitch=%4i throttle=%4i yaw=%4i\n",roll_pulse,pitch_pulse,throttle_pulse,yaw_pulse);
        } 
    }

    motors.output();


    if(DEBUG_CONTROL || DEBUG_ALL){
        if(counter == 0){
            hal.console->printf("rcout: roll= %04i, pitch= %04i, throttle= %04i, yaw= %04i\n",c_roll.control_in, c_pitch.control_in, c_throttle.control_in,c_yaw.control_in); 
            hal.console->printf("rcin: roll= %04i, pitch= %04i, throttle= %04i, yaw= %04i\n",c_roll.radio_in, c_pitch.radio_in, c_throttle.radio_in,c_yaw.radio_in);   
        }
    }
    if(DEBUG_OUTPUT|| DEBUG_ALL){
        if(counter == 0)
            hal.console->printf("output roll= %04i, pitch= %04i, throttle= %04i, yaw= %04i\n",m_roll.radio_in, m_pitch.radio_in, m_throttle.radio_in,m_yaw.radio_in); 
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
