#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	ROS_INFO("I heard:");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_test");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);
	ros::spin();

	return 0;
}
