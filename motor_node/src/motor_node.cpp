#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "motor.h"

static std::unique_ptr<Motor> motorL;
static std::unique_ptr<Motor> motorR;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    motorL->run(joy->axes[1]);
    motorR->run(joy->axes[3]);
}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    // only applies a given velocity for 1 second for safety reasons.
    ros::Rate loop(1);
    float linearSpeed = 0;
    float angularSpeed = 0;
    const float radius = 0.08f;   // 8[cm]
    const float maxLinearSpeed = 0.13729f;  // [m/s]
    const float maxAngularSpeed = maxLinearSpeed / radius;  // [rad/s]

    linearSpeed = static_cast<float>(vel->linear.x) / maxLinearSpeed;
    angularSpeed = static_cast<float>(vel->angular.z) / maxAngularSpeed;

    ROS_INFO("linearSpeed = %f", linearSpeed);
    ROS_INFO("angularSpeed = %f", angularSpeed);

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
