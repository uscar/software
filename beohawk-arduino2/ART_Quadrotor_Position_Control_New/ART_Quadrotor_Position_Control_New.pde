#include <Wire.h>
#include <APM_RC.h>
#include <APM_ADC.h>
#include <APM_Compass.h>
#include <APM_BMP085.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
//#include <GPS_NMEA.h>

#include "ART_Quadrotor.h"

//#define USE_SERIAL

// Initialize ROS Node
ros::NodeHandle nh;

void cb_position (const std_msgs::Float32MultiArray& msg_position) ;

float attitude_pack[3];
std_msgs::Float32MultiArray msg_attitude;
char str_feedback[40];
std_msgs::String msg_feedback;
ros::Publisher pub_attitude("/arduino/attitude", &msg_attitude);
ros::Publisher pub_feedback("/arduino/feedback", &msg_feedback);
ros::Subscriber<std_msgs::Float32MultiArray> sub_position("/arduino/position", &cb_position);
void cb_position (const std_msgs::Float32MultiArray& msg_position) {
  memset(str_feedback, 0, 40);
  char str_number[6];
  dtostrf(msg_position.data[0], 5, 2, str_number);
  sprintf(str_feedback, "Roll: %s ", str_number);
  msg_feedback.data = str_feedback;
  pub_feedback.publish(&msg_feedback);
  nh.spinOnce();
}

/* ROS Subscription
ROS_CALLBACK(pos_cb, std_msgs::String, pos_msg)
  for (int i=0; i < 12; i++) {
     pos_pack.data[i] = pos_msg.data[i]; 
  }
  
  float temp1[2001];
  for (int i=0; i < 2001; i++) {
     temp1[i] = -10.0 + (0.01*i); 
  }
  
  int temp2[361];
  for (int i=0; i < 361; i++) {
     temp2[i] = -180 + i; 
  }
  laserXPosition = temp1[pos_pack.x];
  laserYPosition = temp1[pos_pack.y];
  laserYaw = temp2[pos_pack.theta];
}


ros::Subscriber sub_pos("/arduino/pos", &pos_msg, pos_cb);
*/

void setup() {

  //Serial.begin(BAUDRATE);
  
  
//Serial.println("setup begin");

  // Initialize ROS
  nh.initNode();
  //nh.subscribe(sub_pos);
  nh.advertise(pub_attitude);
  nh.advertise(pub_feedback);
  nh.subscribe(sub_position);

  Wire.begin();

  pinMode(LEDYELLOW,OUTPUT);
  pinMode(LEDRED,OUTPUT);
  pinMode(LEDGREEN,OUTPUT);
  pinMode(SWITCH1,INPUT);
  pinMode(SWITCH2,INPUT);  

  //for sonar pwm
  pinMode(15, INPUT);  

  APM_RC.Init();
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);  

  APM_ADC.Init();
  //APM_BMP085.Init();
  //APM_Compass.Init();
  //GPS.Init();
  delay(100);  

  // Calibrate Gyros
  digitalWrite(LEDRED,HIGH);

  float aux_float[3];

  ADC_Offset[0] = gyroOffset[0];
  ADC_Offset[1] = gyroOffset[1];
  ADC_Offset[2] = gyroOffset[2];  
  ADC_Offset[3] = accelOffset[0];
  ADC_Offset[4] = accelOffset[1];
  ADC_Offset[5] = accelOffset[2];

  readADC();
  delay(20);

  long averageGyro[3] = {
    0,0,0  
  };

  for ( int i = 0 ; i < 100 ; i++ ) {
    readADC();
    for ( int j = 0 ; j < 3 ; j++ ) {
      averageGyro[j] += ADC_Ch[j];
    }
    delay(30);
  }
  for ( int i = 0 ; i < 3 ; i++ ) {
    ADC_Offset[i] = averageGyro[i]/100;    
  }

  digitalWrite(LEDRED,LOW);

  calibrateLevel();



  /* Init pressure altitude
  for ( unsigned int i = 0 ; i < 1000 ; i++ ) {
    readPressureAltitude();
  }
  
  groundPressureAltitude = pressureAltitude;
*/


  timer1 = millis();
  compassReadTimer = millis();  
  telemetryTimer = millis();
  otherTimer = millis();
  rosTimer = millis();
}

void loop() {


  
  //Sonar Sensor Data
  if ( millis() - sonarTimer > 88 ) {
    loopDt_slow = (millis() - sonarTimer)/1000.0;
    
    //sonarAltitude = (pulseIn(15,HIGH)/147.0);
    
    //sonarAltitude = analogRead(7);
    sonarTimer = millis();
    
    //if (sonarAltitude > 150 || sonarAltitude < 25)
    //  sonarAltitude = 200;
    
    
    if (abs(previousSonarAltitude - sonarAltitude) < 10)
    {
      usedAltitude = sonarAltitude;
    }
      
    previousSonarAltitude = sonarAltitude;   
    
    previousActualAltitude = actualAltitude;
    actualAltitude = 0.4*actualAltitude + (1-0.4)*usedAltitude;
     
   //Serial.println(actualAltitude);
  }  
  

  if ( millis() - timer1 > 10 ) { // timer at 100 Hz
    //Serial.println(millis() - timer1);
    loopDt = (millis()-timer1)/1000.0;
    timer1 = millis();
    timeStep++;
    
    
    getMeasurements();
    pitch = -pitch; 
    yaw = -yaw;
    
    //Serial.print(pitch);    Serial.print("  ");    Serial.print(roll);    Serial.print("  ");    Serial.println(yaw);

    // Read RC receiver
    for ( int i = 0 ; i < 4 ; i++ ) {
      RCInput[i] = radioFilter(APM_RC.InputCh(i),RCInput[i]);
    }

    for ( int i = 4 ; i < 8 ; i++ ) {
      RCInput[i] = APM_RC.InputCh(i);
      //Serial.print(i); Serial.print (": ");Serial.print(RCInput[i]);Serial.print("    ");
    }
    //Serial.println("");

    pilotRollOld = pilotRoll;
    pilotRoll = 0.175*(RCInput[0]-MIDCHANNEL);
    pilotPitchOld = pilotPitch;
    pilotPitch = -0.175*(RCInput[1]-MIDCHANNEL);
    pilotThrottle = RCInput[2];
    //Serial.println(pilotThrottle);
    pilotYawOld = pilotYaw;
    pilotYaw = 2.0*(RCInput[3]-MIDCHANNEL);
        
    if (RCInput[4] < 1550)
    {
      if (positionHold == false)
      {
        positionHold = true;
        desiredYaw = laserYaw;
        desiredXPosition = laserXPosition;
        desiredYPosition = laserYPosition;
      }
      desiredPitch = -10*(1/(1+exp(-5*(desiredXPosition-laserXPosition)))-1/2);
      desiredRoll = 10*(1/(1+exp(-5*(desiredYPosition-laserYPosition)))-1/2);
      desiredYawChange = (desiredYaw - laserYaw)*10;
      
    } else {
      positionHold = false;    
      desiredPitch = pilotPitch;
      desiredRoll = pilotRoll;
      desiredYawChange = pilotYaw;
    }
       
    
    
    if (positionHold == true)
    {
      desiredPitch = -10*(1/(1+exp(-5*desiredXPosition))-1/2);
      desiredRoll = 10*(1/(1+exp(-5*desiredYPosition))-1/2);
    } else {
      desiredPitch = pilotPitch;
      desiredRoll = pilotRoll;
    }
   

   //Throttle setting  
   if (RCInput[5] < 1550) //Altitude hold will be on
   {
     if (altHold == false) {
       holdingThrottle = pilotThrottle;
       holdingAltitude = actualAltitude;
       desiredAltitude = actualAltitude;
       
       altHold = true;
       altHoldStart = millis();
     }
     throttle = holdingThrottle;
   }
   else
   {
     altHold = false;
     throttle = pilotThrottle;
     desiredAltitude = actualAltitude;
   }
   
   //desiredAltitude = constrain(map(pilotThrottle,1100,2000, 17,100),10,70);     
 
   PIDControl();
   
   //Arm Motors 
   if ( pilotThrottle < 1200 ) {
     
       if ( RCInput[3] > 1700 && RCInput[5] > 1550) {
           motorsArmed = 1;
       }
       if ( RCInput[3] < 1200 ) {
           motorsArmed = 0;
           isLanding = false;
       }
   }
   
   //Altitude Hold

   if (motorsArmed == 0) {
     altitudeI = 0; 
     yawI = 0;
     rollI = 0;
     pitchI = 0;
     desiredAltitude = 0;
   }
   if (altHold == false || (millis()-altHoldStart)/1000 < altHoldDelay)
     altitudeI = 0;

 // DO NOT DELETE: THIS IS FOR COMPUTER CONTROLLED ALTITUDE HOLD
  // Computer controlled mode
       digitalWrite(LEDYELLOW,HIGH); 
       //Serial.println("ALT HOLD");       
  
       
    
   
   isManualControl = true;
   
   digitalWrite(LEDYELLOW,LOW);  

   //controlAltitude = 0;
   digitalWrite(LEDYELLOW,HIGH); 
          
   //xSerial.println(actualAltitude);
   digitalWrite(LEDYELLOW,LOW);
   //Serial.print(desiredAltitude); Serial.print("  "); Serial.print(holdingAltitude); Serial.print("  "); Serial.print(actualAltitude); Serial.print("  ") ;Serial.print(altitudeI); Serial.print("   "); Serial.print(controlAltitude); Serial.print("  "); Serial.println(throttle);
   //Serial.print(controlRoll); Serial.print("   "); Serial.print(controlPitch); Serial.print("   ");    Serial.println(controlYaw);
   //Serial.print(actualAltitude);Serial.print("  "); Serial.print(desiredAltitude); Serial.print("  "); Serial.print(altitudeI); Serial.print("  "); Serial.println(controlAltitude);
   //Serial.print(pitchI); Serial.print("   "); Serial.print(rollI); Serial.print("   ");    Serial.println(yawI);
   if ( motorsArmed == 1 ) {
     digitalWrite(LEDRED,HIGH);    
     if (altHold == false || (millis()-altHoldStart)/1000 < altHoldDelay)
        controlAltitude = 0;
      
      
              
      motor[0] = constrain(throttle+controlRoll+controlPitch-controlYaw+(controlAltitude)/*+controlXpose+controlYpose-controlThetapose*/,1100,2000);
      motor[1] = constrain(throttle+controlRoll-controlPitch+controlYaw+(controlAltitude)/*+controlXpose-controlYpose+controlThetapose*/,1100,2000);
      motor[2] = constrain(throttle-controlRoll-controlPitch-controlYaw+(controlAltitude)/*-controlXpose-controlYpose-controlThetapose*/,1100,2000);
      motor[3] = constrain(throttle-controlRoll+controlPitch+controlYaw+(controlAltitude)/*-controlXpose+controlYpose+controlThetapose*/,1100,2000);
      
      //Serial.println(motor[0]); Serial.print("  =  "); Serial.print(controlPitch); Serial.print(" + "); Serial.print(controlRoll);Serial.print(" + "); Serial.print(controlYaw); Serial.print(" + "); Serial.print(controlAltitude); Serial.print("*"); Serial.println(flag);

      
      APM_RC.OutputCh(0,motor[0]);
      APM_RC.OutputCh(1,motor[1]);
      APM_RC.OutputCh(2,motor[2]);
      APM_RC.OutputCh(3,motor[3]);
    } 
    else {
      digitalWrite(LEDRED,LOW);      
      APM_RC.OutputCh(0,1100);
      APM_RC.OutputCh(1,1100);
      APM_RC.OutputCh(2,1100);
      APM_RC.OutputCh(3,1100);
    }
  }
  
  
    //Ros msg data
  if ( millis() - rosTimer > 100) {   
    
    attitude_pack[0] = ToDeg(roll);
    attitude_pack[1] = ToDeg(pitch);
    attitude_pack[2] = ToDeg(yaw);
    //attitude_pack.pitch = find_attitude_index(ToDeg(pitch));
    //attitude_pack.yaw = find_yaw_index(yaw);
    msg_attitude.data_length = 3;
    msg_attitude.data = attitude_pack;
    pub_attitude.publish(&msg_attitude);
    
 
    
    nh.spinOnce();
    rosTimer = millis();
  }
 
}

void PIDControl() {
  rollError = constrain(desiredRoll - ToDeg(roll),-60,60);

  rollI += rollError*loopDt;
  rollI = constrain(rollI,-pitchRollIMax,pitchRollIMax);

  rollD = ToDeg(-Omega[0]);

  controlRoll = Kproll*rollError + Kiroll*rollI + Kdroll*rollD;

  ///////////////////////////////////////

  pitchError = constrain(desiredPitch - ToDeg(pitch),-60,60);

  pitchI += pitchError*loopDt;
  pitchI = constrain(pitchI,-pitchRollIMax,pitchRollIMax);

  pitchD = ToDeg(Omega[1]);

  controlPitch= Kppitch*pitchError + Kipitch*pitchI + Kdpitch*pitchD;

  /////////////////////////////////////////////

  yawError = constrain(desiredYawChange + .5*readADCCorrected(2),-80,80);
  //Serial.println(readADCCorrected(2));
  //Serial.println(pilotYaw);

  yawI += yawError*loopDt;
  yawI = constrain(yawI,-20,20);

  //yawD = readADCCorrected(2);

  controlYaw= Kpyaw*yawError + Kiyaw*yawI + Kdyaw*yawD;

  /////////////////////////////////////////////

  altitudeError = desiredAltitude-actualAltitude;
  
  altitudeI += altitudeError*loopDt;
  altitudeI = constrain(altitudeI,-altitudeIMax,altitudeIMax);

  altitudeD = (actualAltitude - previousActualAltitude)/loopDt_slow;
  altitudeErrorOld = altitudeError;

  controlAltitude = Kpaltitude*altitudeError + Kialtitude*altitudeI + Kdaltitude*altitudeD;  

}

// Maximum slope filter for radio inputs... (limit max differences between readings)
int radioFilter(int ch, int ch_old)
{
  int diff_ch_old;

  if (ch_old==0)      // ch_old not initialized
    return(ch);
  diff_ch_old = ch - ch_old;      // Difference with old reading
  if (diff_ch_old<0)
  {
    if (diff_ch_old<-40)
      return(ch_old-40);        // We limit the max difference between readings
  }
  else
  {
    if (diff_ch_old>40)    
      return(ch_old+40);
  }
  return(ch);
}

void getMeasurements() {
  readADC();
  if (MAGNETOMETER == 1) {
    if (millis() - compassReadTimer >= 100)  // Read compass data at 10Hz...
    {
      compassReadTimer = millis();
      APM_Compass.Read();     // Read magnetometer
      APM_Compass.Calculate(roll,pitch);  // Calculate heading
    }
  }
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();

  //  sonarAltitude = sonarAltitude*.9 + constrain(analogRead(A6),15,500)*.1;
}

void calibrateLevel() {
  digitalWrite(LEDRED,HIGH);

  // Calibrate accelOffset
  int average_x_offset = 0;
  int average_y_offset = 0;

  for ( int i = 0 ; i < 10 ; i++ ) {
    readADC();
    average_x_offset += ADC_Ch[3];
    average_y_offset += ADC_Ch[4];
    delay(20);
  }

  accelOffset[0] = average_x_offset/10;
  accelOffset[1] = average_y_offset/10;

  ADC_Offset[3] = accelOffset[0];
  ADC_Offset[4] = accelOffset[1];
  ADC_Offset[5] = accelOffset[2];

  digitalWrite(LEDRED,LOW);
}

void readPressureAltitude() {
  // Barometer read
  APM_BMP085.Read();
  float tempPressureAltitude;
  tempPressureAltitude = float(APM_BMP085.Press)/(101325.0); //101325.0;
  tempPressureAltitude = pow(tempPressureAltitude,0.190295);
  if ( pressureAltitude == 0 ) {
    pressureAltitude = (1.0 - tempPressureAltitude) * 4433000 / 2.54; // in
  } 
  else {
    pressureAltitude = pressureAltitude * 0.94 + ((1.0 - tempPressureAltitude) * 4433000 / 2.54)*0.06; // in
  }
  //Serial.println(pressureAltitude - groundPressureAltitude);
}  


int find_attitude_index(float pos) {
  float temp[2001];
  for (int i=0; i < 2001; i++) {
	temp[i] = -10.0+(0.01*i);
  }
	
  int current_index = 0;
	
  for (int i=0; i<2001; i++) {
	if (pos > temp[i]) {
		current_index = i;
	} else {
		break;
	}
  }
	
  return current_index;
}

int find_yaw_index(int yaw) {
  int temp[361];
  int current_index = 0;
  for (int i=0; i < 361; i++) {
   if (yaw > temp[i]) {
    current_index = i; 
   } else {
      break; 
    }
  } 
  return current_index;
}



