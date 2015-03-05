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
    
    rc1.radio_min = rc2.radio_min = rc3.radio_min = rc4.radio_min = 1000;
    rc1.radio_max = rc2.radio_max = rc3.radio_max = rc4.radio_max = 2000;
    rc1.set_range(1000,2000);
    rc2.set_range(1000,2000);
    rc3.set_range(1000,2000);
    rc4.set_range(1000,2000);
    
    
    rc3.servo_out = 1;    
    rc3.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    // motor initialisation
    motors.set_update_rate(490);
    motors.set_frame_orientation(AP_MOTORS_H_FRAME);
//    motors.set_min_throttle(1000);
//    motors.set_mid_throttle(1500);


    hal.console->printf("setup : %i\n",(int)motors.setup_throttle_curve());
    

    
    motors.Init();      // initialise motors

    hal.console->printf("%i,%i",(int)rc3.radio_min,(int)rc3.radio_max);

    motors.enable();
    motors.output_min();
    rc1.set_pwm(1000);
    rc2.set_pwm(1000);
    rc4.set_pwm(1000);
    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int value;

    // display help
    hal.console->println("\nPress 't' to test motors.  Be careful they will spin! \nPress 'n' to test nam\nPress 'b' to do something\nPress 'q' to quit");
    
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
    if( value == 'n' || value == 'N'){
       hal.console->println("testing nam");
       motors.armed(true);
      for(int i = 0;i<500;i++){
         rc3.servo_out = 2*i;
//        rc3.set_pwm(8*i);
         hal.console->printf("\n - - - %i",(int)rc3.servo_out);
         hal.console->printf("\n---%i",(int)motors.get_max_throttle());
         hal.console->printf("\n%i %i -> %i , %i",rc3.control_in,rc3.radio_in,rc3.pwm_out,rc3.radio_out);
         hal.console->printf("\n%i - ",i);
//       motors.output_min();
         motors.output();
//         motors.throttle_pass_through();
         print_motor_output();
         hal.console->printf("\n%i%i%i%i",(int)motors.limit.throttle_upper,(int)motors.limit.throttle_lower,(int)motors.limit.yaw,(int)motors.limit.roll_pitch);

         hal.scheduler->delay(50);
       }
       motors.output_min();
       motors.armed(false);
       hal.console->println("done"); 
    }
    if(value == 'b' || value == 'B'){
      for(int i = 1150;i<2000;i+=10){
          hal.rcout->write(CH_1,i);
          hal.rcout->write(CH_3,i);
          hal.console->printf("outputting: %i\n",i);
          hal.scheduler->delay(20);
      }
      hal.scheduler->delay(500);
      hal.rcout->write(CH_1,0);
      hal.rcout->write(CH_3,0);
    }
    if(value == 'q' || value == 'Q'){
       motors.armed(false);
       exit(1); 
       hal.console->println("Program Quit");
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
