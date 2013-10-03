#include "Comm.h"

#ifdef USE_ROS

char Comm::header_frame_tf_euler[] = "/imu",
     Comm::child_frame_tf_euler[] = "/world";
  

void Comm::init () {
  nh.initNode();
  pub_tf.init(nh);
}

void Comm::publish_tf_imu () {
  tf_imu.header.stamp = nh.now();
  tf_imu.header.frame_id = header_frame_tf_euler;
  tf_imu.child_frame_id = child_frame_tf_euler;
  Quaternion q;
  q.from_euler(imu.roll, imu.pitch, imu.yaw);
  tf_imu.transform.translation.x = 0;
  tf_imu.transform.translation.y = 0;
  tf_imu.transform.translation.z = 0;
  tf_imu.transform.rotation.w = q.q1;
  tf_imu.transform.rotation.x = q.q2;
  tf_imu.transform.rotation.y = q.q3;
  tf_imu.transform.rotation.z = q.q4;
  pub_tf.sendTransform(tf_imu);
}

#else
void Comm::init() {
  Serial.begin(115200);
}

void Comm::publish_euler () {
  Serial.printf("%f\t%f\t%f\t", ToDeg(imu.roll),
      ToDeg(imu.pitch), ToDeg(imu.yaw));
}

void Comm::publish_imu_raw () {
  Serial.printf("g: %f\t%f\t%f | a: %f\t%f\t%f\t",
      imu.vec_gyro.x, imu.vec_gyro.y, imu.vec_gyro.z,
      imu.vec_accel.x, imu.vec_accel.y, imu.vec_accel.z);
  Serial.print("\n");
}
#endif
