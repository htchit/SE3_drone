#include <ros/ros.h>
#include "controller.h"

#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "std_msgs/Float32.h"

Desired_State_t des;
Odom_Data_t odom_data;

void odom_sub_cb(const geometry_msgs::PoseStamped &msg)
{
    odom_data.p(0) = msg.pose.position.x;
    odom_data.p(1) = msg.pose.position.y;
    odom_data.p(2) = msg.pose.position.z;
    odom_data.q(0) = msg.pose.orientation.x;
    odom_data.q(1) = msg.pose.orientation.y;
    odom_data.q(2) = msg.pose.orientation.z;
    odom_data.q(3) = msg.pose.orientation.w;
}

void velocity_sub_cb(const geometry_msgs::TwistStamped &msg)
{
    odom_data.v(0) = msg.twist.linear.x;
    odom_data.v(1) = msg.twist.linear.y;
    odom_data.v(2) = msg.twist.linear.z;
    odom_data.w(0) = msg.twist.angular.x;
    odom_data.w(1) = msg.twist.angular.y;
    odom_data.w(2) = msg.twist.angular.z;
}

void Input_sub_cb(const std_msgs::String::ConstPtr &msg)
{
    des.p(0) = msg.p.x;
    des.p(1) = msg.p.y;
    des.p(2) = msg.p.z;
    des.v(0) = msg.v.x;
    des.v(1) = msg.v.y;
    des.v(2) = msg.v.z;
    des.a(0) = msg.a.x;
    des.a(1) = msg.a.y;
    des.a(2) = msg.a.z;
    des.jerk(0) = msg.jerk.x;
    des.jerk(1) = msg.jerk.y;
    des.jerk(2) = msg.jerk.z;
    des.yaw = msg.yaw;
    des.head_rate = msg.head_rate;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Ctrl");
    ros::NodeHandle nh;

    Parameter_t param(nh);

    Controller controller(param);

    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1000, pose_sub_cb);
    ros::Subscriber vel_sub = n.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1000, velocity_sub_cb);

    ros::Subscriber Input_sub = n.subscribe("/PX4_geometric_controller/target_cmd", 1000, Input_sub_cb);

    ros::Publisher ctrl_pub = n.advertise<mavros_msgs::AttitudeTarget>("/setpoint_raw/attitude", 10);

    ros::Rate r(param.ctrl_rate);

    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        ros::Time now_time = ros::Time::now();

        controller.config_gain(param);

        controller.update(des, odom_data, u);

        controller.publish_ctrl(u, now_time);
    }

    return 0;
}
