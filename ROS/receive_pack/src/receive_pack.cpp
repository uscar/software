#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;

ros::Subscriber sub_arduino;

float find_index(int param) {
	float value[2001];
	for (int i = 0; i < sizeof(value); i++) {
		value[i] = -10.0 + (0.01*i);
	}
	return value[param];
}

int find_yaw(int param) {
	int value[361];
	for (int i = 0; i < sizeof(value); i++) {
		value[i] = -180 + i;
	}
	return value[param];
}


/*
void arduino_cb(const std_msgs::String::ConstPtr& msg_arduino) {
	union DataPacket {
		struct {
			float roll;
			float pitch;
			float yaw;
		};
		char data[12];
	} packet;

	for (int i = 0; i < 12; i++) {
		packet.data[i] = msg_arduino->data[i];
	}
	
	cout << endl << "   " << "Roll:  " << (packet.roll) << "   Pitch:  " << (packet.pitch) << "   Yaw:  " << (packet.yaw) 		<< endl;

}

*/

void arduino_cb (const std_msgs::Float32MultiArray::ConstPtr& msg_attitude) {
	cout << endl << "   " << "Roll:  " << msg_attitude->data[0] << "   Pitch:  " << msg_attitude->data[1] << "   Yaw:  " << msg_attitude->data[2]  << endl;

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "receive_pack");
	ros::NodeHandle nh;
	
	/*sub_arduino = nh.subscribe("/arduino/attitude", 1, arduino_cb);
	
	ros::Rate loop_rate(10);
	while(ros::ok()) {
  		ros::spinOnce();
		loop_rate.sleep();
	}
*/

       ros::Publisher pub_position = nh.advertise<std_msgs::Float32MultiArray>("/arduino/position", 10);
       std_msgs::Float32MultiArray msg_position;
       /*msg_position.layout.dim.push_back(std_msgs::MultiArrayDimension());
       msg_position.layout.dim[0].label = "whatever";
       msg_position.layout.dim[0].size = 3;
       msg_position.layout.dim[0].stride = 3;*/

  msg_position.data.push_back(5.6);
       msg_position.data.push_back(-3.4);
       msg_position.data.push_back(-1.2);
      
     while(ros::ok()) {
      pub_position.publish(msg_position);
       
       ros::spinOnce();
       sleep(1);

     }
       return 0;
}
