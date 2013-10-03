#include <APM_ADC.h>
#include <APM_RC.h>

#include "quad_consts.h"

int cache[7];
int cur[7];

void setup(){
  APM_ADC.Init();
  APM_RC.Init();

  Serial.begin(57600);
  Serial.println("test control start");
  for(int i = 0;i<4;i++){
    APM_RC.OutputCh(i,motor_off);
  }
  delay(1000);
  for(int i = 0;i<4;i++){
    APM_RC.OutputCh(i,motor_off); 
  }
}

void loop(){
  int redraw = 0; 
  for(int i = 0;i<7;i++){
    cur[i] = APM_ADC.Ch(i) - IMU_offsets[i];
    if(abs(cur[i] - cache[i])>3){
      cache[i] = cur[i];
      redraw = 1; 
    }
  }

  if(redraw){
    for(int i = 0;i<7;i++){
      Serial.print(cur[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

  int z_g = APM_ADC.Ch(0);
  int y_g = APM_ADC.Ch(1);
  int x_g = APM_ADC.Ch(2);

  int tmp = APM_ADC.Ch(3);

  int x_a = APM_ADC.Ch(4);
  int y_a = APM_ADC.Ch(5);
  int z_a = APM_ADC.Ch(6);


  delay(10);
}

void readIMU(){
  //read in the raw data and zero it
  for(int i = 0;i<7;i++){
    raw_imu[i] = APM_ADC.Ch(i) - IMU_offsets[i];
  }
  //normalize gyro data
  for(int i = 0;i<3;i++){
    cur_gyr[i] = (float)raw_imu[i]/(float)GYR_MAG;
  }
  //normalize accel data
  for(int i = 0;i<3;i++){
    cur_acc[i] = (float)raw_imu[i+3]/(float)ACC_MAG;
  }
}
void readCTRL(){
  //read in the raw data from the controller 
  for(int i = 0;i<8;i++){
    //raw ctrl 0,1,3 can be negative (norm to -1->1)
    raw_ctrl[i] = APM_RC.InputCh(i)-CTRL_offsets[i]; 
    ctrl_norm[i] = (float)raw_ctrl[i]/(float)ctrl_rng;
  }
  //throttle is not past "on" threshold
  if(raw_ctrl[2]<=3){
    throttle = motor_off;
  }
  else{
     throttle = motor_min+(motor_max-motor_min)*ctrl_norm[2];
  }
  //x acc
  target_acc[0] = scale_ctrl(ctrl_norm[0]); // more trig...
  //y acc
  target_acc[1] = scale_ctrl(ctrl_norm[1]); // more trig...
  //z acc
  target_acc[2] = 1; // some function of ctrl[2] == throttle.
  //z gyr
  target_gyr[0] = scale_ctrl(ctrl_norm[3]);
  //y,x gyr
  target_gyr[1] = target_gyr[2] = 0;
  
  pid_to_motor(motor,target_acc,target_gyr,cur_acc,cur_gyr,throttle);
  
}

float scale_ctrl(float in){
  return in/4; 
}

void printIMU(){
  Serial.println();
  Serial.print("acc: x:");
  Serial.print(cur_acc[0]);
  Serial.print(" y:");
  Serial.print(cur_acc[1]);
  Serial.print(" z:");
  Serial.print(cur_acc[2]);
  Serial.println();
  Serial.print("gyr: x:");
  Serial.print(cur_gyr[2]);
  Serial.print(" y:");
  Serial.print(cur_gyr[1]);
  Serial.print(" z:");
  Serial.print(cur_gyr[0]);
  Serial.println();
}


