float i_decay = .7;

float i_factor = .4;
float p_factor = .6;
float d_factor = .4;

float acc_err_hist[3][10];
float gyr_err_hist[3][10];

float mutual_factor = 5;

float i_a[3] = {
  0,0,0};
float p_a[3] = {
  0,0,0};
float d_a[3] = {
  0,0,0};

float i_g[3] = {
  0,0,0};
float p_g[3] = {
  0,0,0};
float d_g[3] = {
  0,0,0};

float net_g[3];
float net_a[3];


void pid_to_motor(int motor_out[4],float targ_acc[3],float targ_gyr[3],float in_acc[3],float in_gyr[3],int thrust){
  for(int i = 0;i<3;i++){
    for(int ii = 8;ii>=0;ii--){
      acc_err_hist[i][ii+1] = acc_err_hist[i][ii];
      gyr_err_hist[i][ii+1] = gyr_err_hist[i][ii];
    }
    acc_err_hist[i][0] = targ_acc[i] - cur_acc[i];
    gyr_err_hist[i][0] = targ_gyr[i] - cur_gyr[i];

    d_g[i] = gyr_err_hist[i][1] - gyr_err_hist[i][0];
    d_a[i] = acc_err_hist[i][1] - acc_err_hist[i][0];

    p_g[i] = gyr_err_hist[i][0];
    p_a[i] = gyr_err_hist[i][0];

    i_g[i] *= i_decay;
    i_g[i] += p_g[i];

    i_a[i] *= i_decay;
    i_g[i] += p_a[i];

    net_g[i] = p_g[i] * p_factor + i_g[i] * i_factor + d_g[i] * d_factor;
    net_a[i] = p_a[i] * p_factor + i_a[i] * i_factor + d_a[i] * d_factor;
  }

  float nmotor[4];
  
  nmotor[0] = (-1)*net_g[0] + (-1)*net_g[1] + ( 1)*net_g[2] + net_a[0] + net_a[1] + net_a[2];
  nmotor[1] = ( 1)*net_g[0] + (-1)*net_g[1] + (-1)*net_g[2] + net_a[0] + net_a[1] + net_a[2];
  nmotor[2] = ( 1)*net_g[0] + ( 1)*net_g[1] + ( 1)*net_g[2] + net_a[0] + net_a[1] + net_a[2];
  nmotor[3] = (-1)*net_g[0] + ( 1)*net_g[1] + (-1)*net_g[2] + net_a[0] + net_a[1] + net_a[2];
  

  float throttle = (motor_out[0]+motor_out[1]+motor_out[2]+motor_out[3])/4;

  int mmotor = motor_out[0];
  for(int i = 1;i<4;i++){
    mmotor = (mmotor>motor_out[i])?mmotor:motor_out[i];
  }

  for(int i = 0;i<4;i++){
    nmotor[i] = (float)motor_out[i] / (float)mmotor; 
    motor_out[i] = nmotor[i]*thrust;
  }
  
  return;
}
