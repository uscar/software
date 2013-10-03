//first 3 = gyro, 4th = temp, last 3 = accel.
//these should be adjusted for each new setup. (slight angle dif.)
int IMU_offsets[7] = {1683,1748,1632,1500,2025,2025,2088};
//                    J1    J2   J3   J4
//                    Rh    Rv   Lv   Lh ???
int CTRL_offsets[8] = {1518,1518,1118,1518,1118,1118,1118,1118};

float ctrl_norm[8];

int raw_ctrl[8];

int raw_imu[7];

int GYR_MAG = 100;
int ACC_MAG = 408;

int throttle = 0;

int motor[4] = {0,0,0,0};

int motor_max = 2000;

int motor_min = 1100;

int motor_off = 0;

int ctrl_min = 1118;

int ctrl_max = 1918;

int ctrl_neu = 1518;

int ctrl_rng = ctrl_max-ctrl_min;

//these are normalized.

//                   x y z
float target_acc[3] = {0,0,1};
//                   z y x
float target_gyr[3] = {0,0,0};

//                x y z
float cur_acc[3] = {0,0,1};
//                z y x
float cur_gyr[3] = {0,0,0};


//ultrasonic transducer hight (cm)
float target_hgt = 100;
float cur_hgt = 0;

//vector types
typedef float fv3[3];
typedef float fv4[4];
typedef int iv3[3];
typedef int iv4[4];
