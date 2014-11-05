
//Anthony was working out how to approach the problems of aggressive landing and takeoff
//last updated 11/4/14 at 10:30 pm

//TODO inherit from the routines.h


//aggressive landing would take a destination vector in space
//figure out normalized vector path to get there (check gyroscope, barometer, optical flow sensor/laser range finder), 
//adjust throttle, roll, pitch, yaw accordingly
//use execute method in flight_control class
void aggressiveLand(Vector3f destinationVector){
    Vector3f routeVector;
    float cntrl_throttleF;
    float cntrl_yawF;
    
    destinationVector = destinationVector.normalized();
    
    //TODO calculate roll, pitch into route vector 
     
    execute(routeVector, cntrl_throttleF, cntrl_yawF);
}  


//aggressive take-off would take a destination vector in space
//figure out normalized vector path to get there (check gyroscope, barometer, optical flow sensor/laser range finder), 
//adjust throttle, roll, pitch, yaw accordingly
//use execute method in flight_control class
void aggressiveTakeOff(Vector3f destinationVector){
    Vector3f routeVector;
    float cntrl_throttleF;
    float cntrl_yawF;
    
    destinationVector = destinationVector.normalized();
    
    //TODO calculate roll, pitch into route vector
     
    execute(routeVector, cntrl_throttleF, cntrl_yawF);
}
