#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "motor.h"

std::unique_ptr<Motor> motorL;
std::unique_ptr<Motor> motorR;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    motorL->run(joy->axes[1]);
    motorR->run(joy->axes[3]);
}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    // only applies a given velocity for 1 second for safety reasons.
    ros::Rate loop(1);
    double linearSpeed = 0;                 /* -1.0 ~ 1.0 */
    double angularSpeed = 0;
    const double maxLinearSpeed = 0.13729;  /* m/s */
    const double radius = 8.0;
    const double maxAngularSpeed = maxLinearSpeed / radius; /* rad/s */

    linearSpeed = vel->linear.x / maxLinearSpeed;
    angularSpeed = vel->angular.z / maxAngularSpeed;

    ROS_INFO("linearSpeed = %lf", linearSpeed);
    ROS_INFO("angularSpeed = %lf", angularSpeed);

    motorL->run(linearSpeed + angularSpeed);
    motorR->run(linearSpeed - angularSpeed);
    loop.sleep();
    motorL->run(0);
    motorR->run(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle n;
    ros::Subscriber joySub = n.subscribe("joy", 10, joyCallback);
    ros::Subscriber velSub = n.subscribe("cmd_vel", 10, velCallback);

    motorL.reset(new Motor(89, 202));
    motorR.reset(new Motor(187, 186));

    ros::spin();

    return 0;
}
