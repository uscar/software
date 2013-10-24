float gyro_sum[3]={0.0f,0.0f,0.0f};
unsigned long gyro_time[2]={0,0};

inline void add_sample(norm3 n){
	gyro_time[0]=gyro_time[1];
	gyro_time[1]=millis();
	sum[0]+=n.normal.x*n.magnitude*(float)(gyro_time[1]-gyro_time[0])*0.001f;
	sum[1]+=n.normal.y*n.magnitude*(float)(gyro_time[1]-gyro_time[0])*0.001f;
	sum[2]+=n.normal.z*n.magnitude*(float)(gyro_time[1]-gyro_time[0])*0.001f;
}

inline norm3 get_sample(){
	norm3 result;
	result.magnitude=gyro_sum[0]*gyro_sum[0]+gyro_sum[1]*gyro_sum[1]+gyro_sum[2]*gyro_sum[2];
	float inverse=1.0f/result.magnitude;
	result.normal.x=gyro_sum[0]*inverse;
	result.nomral.y=gyro_sum[1]*inverse;
	result.normal.z=gyro_sum[2]*inverse;
	return result;
}

