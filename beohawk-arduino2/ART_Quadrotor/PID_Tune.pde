void processPIDConstants()
{
  if(Serial.available() != 0)
  {
    PIDPacket p;
    for(int i = 0; i < 48; i++)
      p.data[i] = Serial.read();  
    
    Kproll = p.roll[0];
    Kiroll = p.roll[1];
    Kdroll = p.roll[2];
    Kppitch = p.pitch[0];
    Kipitch = p.pitch[1];
    Kdpitch = p.pitch[2];
    Kpyaw = p.yaw[0];
    Kiyaw = p.yaw[1];
    Kdyaw = p.yaw[2];
    Kpaltitude = p.altitude[0];
    Kialtitude = p.altitude[1];
    Kdaltitude = p.altitude[2];
    
    Serial.print("\nroll\t");
    Serial.print(Kproll); Serial.print("\t");
    Serial.print(Kiroll); Serial.print("\t");
    Serial.print(Kdroll); Serial.print("\t");   
    Serial.print("\npitch\t");
    Serial.print(Kppitch); Serial.print("\t");
    Serial.print(Kipitch); Serial.print("\t");
    Serial.print(Kdpitch); Serial.print("\t");   
    Serial.print("\nyaw\t");
    Serial.print(Kpyaw); Serial.print("\t");
    Serial.print(Kiyaw); Serial.print("\t");
    Serial.print(Kdyaw); Serial.print("\t");   
    Serial.print("\nalt\t");
    Serial.print(Kpaltitude); Serial.print("\t");
    Serial.print(Kialtitude); Serial.print("\t");
    Serial.print(Kdaltitude); Serial.print("\t");   
  }
}