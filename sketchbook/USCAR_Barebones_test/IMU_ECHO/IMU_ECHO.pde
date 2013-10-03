#include <APM_ADC.h>
#include "quad_consts.h"

int cache[7];
int cur[7];

void setup(){
  APM_ADC.Init();
  Serial.begin(57600);
  Serial.println("ADC (IMU) TESTING BEGIN");
  delay(1000);
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
    raw_read[i] = APM_ADC.Ch(i) - IMU_offsets[i];
  }
  //normalize gyro data
  for(int i = 0;i<3;i++){
    cur_gyr[i] = (float)raw_read[i]/(float)GYR_MAG;
  }
  //normalize accel data
  for(int i = 0;i<3;i++){
    cur_acc[i] = (float)raw_read[i+3]/(float)ACC_MAG;
  }
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

