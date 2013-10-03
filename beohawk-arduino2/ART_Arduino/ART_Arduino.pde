#define USE_ROS

#ifdef USE_ROS
  #include <ros.h>
  #include <ros/time.h>
  #include <tf/transform_broadcaster.h>
  ros::NodeHandle nh;
#else
  #include <FastSerial.h>
  FastSerialPort0(Serial);
#endif

#include <AP_Common.h>
#include <AP_Math.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h>
#include <APM_RC.h>

#include "ART_Arduino.h"
#include "Base.h"
#include "Comm.h"
#include "IMU.h"
#include "RC.h"

unsigned long timer_slow, timer_fast;

void setup () {
  
  base.init();
  comm.init();
  imu.init();
  imu.calibrate();
  delay(1000);
  timer_slow = timer_fast = millis();
}

void loop () {
  if (millis() - timer_fast > 10) {
    timer_fast = millis();
    
    imu.update(); // Update IMU measurements.

    //rc.get_command(); // Get RC inputs.

    //base.update_params(); // Update params based on RC control.

    // Output motor commands.
/*    if (base.motors_armed) {
      base.ledon(RED);
      rc.set_motor()
    } else {
      base.ledoff(RED);
      rc.set_motor(RC_MINTHROTTLE, RC_MINTHROTTLE, RC_MINTHROTTLE, RC_MINTHROTTLE);
    }*/
  }
  if (millis() - timer_slow > 100) {
    timer_slow = millis();

#ifdef USE_ROS
    comm.publish_tf_imu();
    nh.spinOnce();
#else
    comm.publish_euler();
    Serial.print("\n");
#endif
  }
}
