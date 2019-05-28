#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32MultiArray.h"

std_msgs::Float32MultiArray cmd_motor;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    cmd_motor.data[0] = msg->axes[1];
    cmd_motor.data[1] = msg->axes[3];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_test");
	ros::NodeHandle n;
    cmd_motor.data.resize(2);
    
    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("cmd_motor", 1000);
	ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        pub.publish(cmd_motor);
        ros::spinOnce();
        loop_rate.sleep();

    }

	return 0;
}
