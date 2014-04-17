struct kPID{
    float P;
    float I;
    float D;
    float Imax;
};

void setRollPID(kPID rPid);
kPID getRollPID();
void setPitchPID(kPID pPid);
kPID getPitchPID();
void setYawPID(kPID yPid);
kPID getYawPID();

void setGyrFactor(float f);
float getGyrFactor();


