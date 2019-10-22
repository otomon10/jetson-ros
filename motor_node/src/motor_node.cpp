#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "motor.h"

static std::unique_ptr<Motor> motorL;
static std::unique_ptr<Motor> motorR;
static ros::Publisher joint_pub;

const float T = 0.165f;               // separation of wheels [m]
const float wheelRadius = 0.0345f;    // wheel radius [m]
const float maxLinearSpeed = 0.127f;  // wheeRaduus * motor angularVel = 0.0345 [m] * 3.97935 [rad/s] * 0.926(adj) = 0.124 [m/s]
const float maxAngularSpeed = 1.54f;  // maxLinearSpeed * 2 / T = 0.137 [m/s] * 2 / 0.165[m] * 0.926(adj) = 1.54[rad/s]
const float Ts = 0.1f;                // moving duration [sec]

void broadcastOdom(float x, float y, float yaw)
{
    ROS_DEBUG("x= %f y=%f yaw=%f", x, y, yaw);

    std::string source_frame = "odom";
    std::string target_frame = "base_link";

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);

    geometry_msgs::Pose t_pose;
    static double xy[2] = {0};
    xy[0] += x;
    xy[1] += y;
    t_pose.position.x = xy[0];
    t_pose.position.y = xy[1];
    t_pose.orientation.x = quaternion.x();
    t_pose.orientation.y = quaternion.y();
    t_pose.orientation.z = quaternion.z();
    t_pose.orientation.w = quaternion.w();

    static tf::TransformBroadcaster tf_br;
    tf::Transform transform;
    poseMsgToTF(t_pose, transform);
    tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));
}

void publishJointState(float leftWheelSpeed, float rightWheelSpeed)
{
    // calc theta
    static float thetaL;
    static float thetaR;
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

    // calc odom
    static float robot_angle = 0;
    robot_angle += ( (rightWheelSpeed - leftWheelSpeed) * maxLinearSpeed / T ) * Ts;
    float vel    = (rightWheelSpeed + leftWheelSpeed) * maxLinearSpeed / 2;
    float odomX  = vel * cosf(robot_angle) * Ts;
    float odomY  = vel * sinf(robot_angle) * Ts;

    broadcastOdom(odomX, odomY, robot_angle);
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
    float linearSpeed = static_cast<float>(vel->linear.x) / maxLinearSpeed;
    float angularSpeed = static_cast<float>(vel->angular.z) / maxAngularSpeed;
    if(std::fabs(linearSpeed) > 1.0f){
        linearSpeed = linearSpeed / std::fabs(linearSpeed);
    }
    if(std::fabs(angularSpeed) > 1.0f){
        angularSpeed = angularSpeed / std::fabs(angularSpeed);
    }
    ROS_DEBUG("linearSpeed(-1.0~1.0)  = %f", linearSpeed);
    ROS_DEBUG("angularSpeed(-1.0~1.0) = %f", angularSpeed);

    // motor speed (-1.0~1.0)
    float motorLSpeed;
    float motorRSpeed;
    motorLSpeed = linearSpeed - angularSpeed;
    if(std::fabs(motorLSpeed) > 1.0f){
        motorLSpeed = motorLSpeed / std::fabs(motorLSpeed);
    }
    motorRSpeed = linearSpeed + angularSpeed;
    if(std::fabs(motorRSpeed) > 1.0f){
        motorRSpeed = motorRSpeed / std::fabs(motorRSpeed);
    }
    ROS_DEBUG("leftWheelSpeed  = %f", motorLSpeed);
    ROS_DEBUG("rightWheelSpeed = %f", motorRSpeed);

    // only applies a given velocity for 0.1 second for safety reasons.
    ros::Rate loop(static_cast<int>(1/Ts));

    motorL->run(motorLSpeed);
    motorR->run(motorRSpeed);
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
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

    motorL.reset(new Motor(89, 202));
    motorR.reset(new Motor(187, 186));
    publishJointState(0, 0);

    ros::spin();

    return 0;
}
