extern const AP_HAL::HAL& hal;

//<---------declarations-------->
void printv3f(Vector3f & v);

//<-------implementations------->
void printv3f(Vector3f & v){
  hal.console->printf("<%f,%f,%f>\n",v.x,v.y,v.z); 
}

