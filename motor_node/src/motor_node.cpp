#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "motor.h"

static std::unique_ptr<Motor> motorL;
static std::unique_ptr<Motor> motorR;
static ros::Publisher odom_pub;
static ros::Publisher joint_pub;

const double T = 0.165f;               // separation of wheels [m]
const double wheelRadius = 0.0345f;    // wheel radius [m]
const double maxLinearSpeed = 0.127f;  // wheeRaduus * motor angularVel = 0.0345 [m] * 3.97935 [rad/s] * 0.926(adj) = 0.124 [m/s]
const double maxAngularSpeed = 1.54f;  // maxLinearSpeed * 2 / T = 0.137 [m/s] * 2 / 0.165[m] * 0.926(adj) = 1.54[rad/s]
const double Ts = 0.1f;                // moving duration [sec]

void broadcastOdom(double vx, double vy, double vth)
{
    ros::Time publish_time = ros::Time::now();

    // calc odom
    static double x = 0;
    static double y = 0;
    static double th = 0;
    th += vth * Ts;
    x += (vx * cos(th) - vy * sin(th)) * Ts;
    y += (vx * sin(th) + vy * cos(th)) * Ts;

    // make tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = publish_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // broadcast odom tf frame
    static tf::TransformBroadcaster tf_br;
    tf_br.sendTransform(odom_trans);

    // publish odom msg
    nav_msgs::Odometry odom;
    odom.header.stamp = publish_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
}

void publishJointState(double leftWheelSpeed, double rightWheelSpeed)
{
    // calc theta
    static double thetaL;
    static double thetaR;
    thetaL += (leftWheelSpeed  * maxLinearSpeed) * Ts / wheelRadius;
    thetaR += (rightWheelSpeed * maxLinearSpeed) * Ts / wheelRadius;
    ROS_DEBUG("posL = %f", thetaL);
    ROS_DEBUG("posR = %f", thetaR);

    // publish joint satate
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.name.resize(2);
    js.name[0] = "left_wheel_joint";
    js.name[1] = "right_wheel_joint";
    js.position.resize(2);
    js.position[0] = thetaL;
    js.position[1] = thetaR;
    joint_pub.publish(js);

    // calc velocity
    double vth = (rightWheelSpeed - leftWheelSpeed) * maxLinearSpeed / T;
    double vx = (rightWheelSpeed + leftWheelSpeed) * maxLinearSpeed / 2;

    broadcastOdom(vx, 0, vth);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    motorL->run(joy->axes[1]);
    motorR->run(joy->axes[3]);
    publishJointState(joy->axes[1], joy->axes[3]);
}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    ROS_DEBUG("linear.x  = %f (max speed: %f)", vel->linear.x, maxLinearSpeed);
    ROS_DEBUG("angular.z = %f (max speed: %f)", vel->angular.z, maxAngularSpeed);

    // normalize
    double linearSpeed = vel->linear.x / maxLinearSpeed;
    double angularSpeed = vel->angular.z / maxAngularSpeed;
    if(std::fabs(linearSpeed) > 1.0){
        linearSpeed = linearSpeed / std::fabs(linearSpeed);
    }
    if(std::fabs(angularSpeed) > 1.0){
        angularSpeed = angularSpeed / std::fabs(angularSpeed);
    }
    ROS_DEBUG("linearSpeed(-1.0~1.0)  = %f", linearSpeed);
    ROS_DEBUG("angularSpeed(-1.0~1.0) = %f", angularSpeed);

    // motor speed (-1.0~1.0)
    double motorLSpeed;
    double motorRSpeed;
    motorLSpeed = linearSpeed - angularSpeed;
    if(std::fabs(motorLSpeed) > 1.0){
        motorLSpeed = motorLSpeed / std::fabs(motorLSpeed);
    }
    motorRSpeed = linearSpeed + angularSpeed;
    if(std::fabs(motorRSpeed) > 1.0){
        motorRSpeed = motorRSpeed / std::fabs(motorRSpeed);
    }
    ROS_DEBUG("leftWheelSpeed  = %f", motorLSpeed);
    ROS_DEBUG("rightWheelSpeed = %f", motorRSpeed);

    // only applies a given velocity for 0.1 second for safety reasons.
    ros::Rate loop(static_cast<int>(1/Ts));

    motorL->run(static_cast<float>(motorLSpeed));
    motorR->run(static_cast<float>(motorRSpeed));
    publishJointState(motorLSpeed, motorRSpeed);
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
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

    motorL.reset(new Motor(89, 202));
    motorR.reset(new Motor(187, 186));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(10);
    while(ros::ok())
    {
        {
            publishJointState(0, 0);
        }
        rate.sleep();
    }

    spinner.stop();

    return 0;
}
