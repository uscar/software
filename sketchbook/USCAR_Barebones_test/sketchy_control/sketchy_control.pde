#include <APM_RC.h> // ArduPilot Mega RC Library

int vals[8];
int cache[8];

void setup()
{
    APM_RC.Init();     // APM Radio initialization
    Serial.begin(57600);
    Serial.println("ArduPilot Mega RC library test");
    delay(1000);
}

void loop()
{
    // New radio frame? (we could use also if((millis()- timer) > 20)
//    if (APM_RC.GetState() == 1){

        for(int i = 0; i < 8; i++){
          vals[i] = APM_RC.InputCh(i);
        }
        for(int i = 0;i<8;i++){
          if(abs(vals[i]-cache[i])>5){
            Serial.print("CH:");
            for(int i =0;i<8;i++){
              cache[i] = vals[i];
              Serial.print(vals[i]);    // Print channel values
              Serial.print(",");
              int outs[4];
              for(int i = 0;i<4;i++){
                 outs[i] = vals[i]; 
              }
            //APM_RC.OutputCh(i, APM_RC.InputCh(i)); // Copy input to Servos 
          }
            Serial.println();
            break;
          }
        }
       
//    }
}

void writeOutputs(int controls[4]){
  int outs[4];
  for(int i = 0;i<4;i++){
    outs[i] = (controls[2]-1000)*3;
  }
  
  
  
  int rot = controls[3] - 1498;
  outs[1] += rot;
  outs[3] += rot;
  outs[0] -= rot;
  outs[2] -= rot;
  
  int left = controls[0]-1498;
  outs[0] += left;
  outs[1] += left;
  outs[2] -= left;
  outs[3] -= left;
  
  int forward = controls[1]-1498;
  outs[0] += forward;
  outs[2] += forward;
  outs[1] -= forward;
  outs[3] -= forward;
  Serial.println();
  for(int i = 0;i<4;i++){
     Serial.print(outs[i]);
     Serial.print(" "); 
  }
  Serial.println();
  for(int i = 0;i<4;i++){
    APM_RC.OutputCh(i, outs[i]); // Copy input to Servos            
  }
}

