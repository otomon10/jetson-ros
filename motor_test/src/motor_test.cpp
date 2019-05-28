#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "motor.h"

Motor *motorL;
Motor *motorR;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    motorL->run(msg->axes[1]);
    motorR->run(msg->axes[3]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);

    motorL = new Motor(89, 202);
    motorR = new Motor(187, 186);

    ros::spin();

	return 0;
}
