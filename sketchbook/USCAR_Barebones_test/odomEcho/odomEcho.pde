/*
    Example of APM_RC library.
    Code by Jordi MuÒoz and Jose Julio. DIYDrones.com

    Print Input values and send Output to the servos
    (Works with last PPM_encoder firmware)
*/

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


