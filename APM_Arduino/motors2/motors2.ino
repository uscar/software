/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */


//////notes
/*
the servo_out value must be greater than 0 for output to go
it then checks the pwm_out and radio_out values
which seems to imply that it wants an angle_raw type of RC_Channel.
*/


// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AP_Notify.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(&rc1, &rc2, &rc3, &rc4);
AP_MotorsQuad   motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsHexa	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsY6	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsOcta	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsOctaQuad	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsHeli	motors(&rc1, &rc2, &rc3, &rc4);


// setup
void setup()
{
    hal.console->println("AP_Motors library test ver 1.0");
//    rc1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//    rc2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//    rc3.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//    rc4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc3.set_type(RC_CHANNEL_TYPE_RANGE);
    rc3.set_range(0,1000); // the range of values expected from servo_out.
    rc3.radio_min = 1000; // the range of output values
    rc3.radio_max = 2000;
    
    rc1.set_range(0,1000); 
    rc2.set_range(0,1000);
    rc4.set_range(0,1000);

    rc2.servo_out = 1000;

    // motor initialisation
    motors.set_update_rate(490);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
//    motors.set_min_throttle(1150);
//    motors.set_mid_throttle(1500);
    rc3.set_pwm(1000);
    rc3.trim();
    
    motors.Init();      // initialise motors
    motors.enable();

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int value;

    // display help
    hal.console->println("Press 't' to test motors.  Be careful they will spin!");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    // test motors
    if( value == 't' || value == 'T' ) {
        hal.console->println("testing motors...");
        motors.armed(true);
        motors.output_test();
        motors.armed(false);
        hal.console->println("finished test.");
    }
    if( value == 'n'){
       hal.console->println("testing nam");
       motors.armed(true);
      for(int i = 0;i<200;i++){
         rc3.servo_out = 5*i;
//         rc3.calc_pwm();
         motors.output();
//         motors.throttle_pass_through();
         print_motor_output();
//         hal.console->printf("%i%i%i%i",(int)motors.limit.throttle_upper,(int)motors.limit.throttle_lower,(int)motors.limit.yaw,(int)motors.limit.roll_pitch);
         hal.console->printf("\nservo | radio | pwm  ||servo | radio | pwm  \n");
         hal.console->printf("%04i   %04i    %04i    %04i   %04i    %04i   \n",(int)rc3.servo_out,(int)rc3.radio_out,(int)rc3.pwm_out,(int)rc2.servo_out,(int)rc2.radio_out,(int)rc2.pwm_out);
         hal.scheduler->delay(50);
       }
       motors.output_min();
       motors.armed(false);
       hal.console->println("done"); 
    }
    if(value == 'b'){
      motors.armed(true);
      for(int i = 1150;i<2000;i+=10){
          hal.rcout->write(CH_1,i);
          hal.rcout->write(CH_2,i);
          hal.rcout->write(CH_3,i);
          hal.rcout->write(CH_4,i);
          hal.console->printf("outputting: %i\n",i);
          hal.scheduler->delay(20);
      }
      
      
      
      hal.scheduler->delay(500);
      hal.rcout->write(CH_1,0);
      hal.rcout->write(CH_3,0);
    }
    
    
      if(value == 'h'){
        motors.armed(true);
         hal.rcout->write(CH_1,2000);
         hal.rcout->write(CH_2,2000);
         hal.rcout->write(CH_3,2000);
         hal.rcout->write(CH_4,2000); 
      }
      if(value == 'l'){
        motors.armed(true);
        hal.rcout->write(CH_1,1000);
        hal.rcout->write(CH_2,1000); 
        hal.rcout->write(CH_3,1000);
        hal.rcout->write(CH_4,1000);
      }
      if(value == '0'){
        motors.armed(true);
        hal.rcout->write(CH_1,0);
        hal.rcout->write(CH_2,0);
        hal.rcout->write(CH_3,0);
        hal.rcout->write(CH_4,0);
      }
      if(value == 'r'){
        motors.armed(true);
        while(1){
          int val = hal.rcin->read(CH_3);
          hal.rcout->write(CH_1,val);
          hal.rcout->write(CH_2,val);
          hal.rcout->write(CH_3,val);
          hal.rcout->write(CH_4,val);
          hal.console->printf("outputting: %i\n",val);
        }
      }
}

// print motor output
void print_motor_output()
{
    int8_t i;
    for(i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if( motors.motor_enabled[i] ) {
            hal.console->printf("\t%d %d",i,motors.motor_out[i]);
        }
    }
}

AP_HAL_MAIN();
