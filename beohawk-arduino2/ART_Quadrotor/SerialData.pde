#define MSG_POSITIONAL_ERROR 1
#define MSG_ARM_MOTORS 2
#define MSG_DESIRED_ALTITUDE 3
#define MSG_PID_ROLL 'r'
#define MSG_PID_PITCH 'p'
#define MSG_PID_YAW 'y'
#define MSG_PID_ALTITUDE 'a'

void processPicoITXSerial() {
  // Create and send serial message containing Attitude Info to PicoITX
  ros_rot.x = roll;
  ros_rot.y = pitch;
  ros_rot.z = yaw;
  ros_alt.data = sonarAltitude;
  
  rotation.publish( &ros_rot );
  altitude.publish( &ros_alt );
  nh.spinOnce();
  
  //packet.checkSum = 0;

  // Read serial message from PicoITX
  /*num = Serial.available();
  timer = millis();
  while (num >= 6 && millis() - timer < 12) {
    id = Serial.read();
    data[0] = Serial.read();
    data[1] = Serial.read();
    data[2] = Serial.read();
    data[3] = Serial.read();
    checkSum = Serial.read();
    Serial.print("Read: ");
    Serial.print(id,DEC);Serial.print('\t');
    Serial.print(data[0]);Serial.print('\t');
    Serial.print(data[1]);Serial.print('\t');
    Serial.print(data[2]);Serial.print('\t');
    Serial.print(data[3]);Serial.print('\t');
    Serial.print(checkSum);Serial.println('\t');
    if (true) { //(id ^ data[0] ^ data[1] ^ data[2] ^ data[3]) == checkSum) {
      switch (id) {
      case MSG_POSITIONAL_ERROR: 
        xError = data[0];     // Positional Error
        yError = data[1];
        // Reset serial message handlers
        id = 0;
        checkSum = 0;
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;        
        break;
      case MSG_ARM_MOTORS: 
        motorsArmed = data[0];     // Arm Motors
        // Reset serial message handlers
        id = 0;
        checkSum = 0;
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;        
        break;
      case MSG_DESIRED_ALTITUDE: 
        desiredAltitude = data[0];     // Desired Altitude
        Serial.print("Desired altitude: "); Serial.println(desiredAltitude);
        if (desiredAltitude <= 0) {
          if ( isManualControl == false ) {
            isLanding = true;
            landingAltitude = sonarAltitude;
            landingTime = millis();
          }
        } 
        else {
          isManualControl = false;
          throttle = 1400;                      
        }
        holdingAltitude = true;
        // Reset serial message handlers
        id = 0;
        checkSum = 0;
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;        
        break;   
        
      }
    }
    num = Serial.available();
  }*/
}  

float readFloat() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";
  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}


