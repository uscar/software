
//<---------declarations-------->
void printv3f(Vector3f & v, const AP_HAL::HAL & h);

//<-------implementations------->
void printv3f(Vector3f & v, const AP_HAL::HAL & h){
  h.console->printf("<%f,%f,%f>",v.x,v.y,v.z); 
}

