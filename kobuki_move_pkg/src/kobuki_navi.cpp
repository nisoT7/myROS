#include<iostream>
#include<math.h>
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
#include<tf/transform_datatypes.h>

using namespace std;

namespace{
    double poseX;
    double poseY;
    double poseT;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    geometry_msgs::Quaternion quat;
    poseX = odom->pose.pose.position.x;
    poseY = odom->pose.pose.position.y;

    quat.x = odom->pose.pose.orientation.x;
    quat.y = odom->pose.pose.orientation.y;
    quat.z = odom->pose.pose.orientation.z;
    quat.w = odom->pose.pose.orientation.w;

    poseT = tf::getYaw(quat);

    cout<<"x = "<<poseX<<"[m]"<<"\t";
    cout<<"y = "<<poseY<<"[m]"<<"\t";
    cout<<"theta = "<<poseT*180/M_PI<<"[deg]"<<endl;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"kobuki_navi");
    ros::NodeHandle node;

    ros::Subscriber odom_sub = node.subscribe("odom",1,odomCallback);
    ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel",1);

    geometry_msgs::Twist cmd_vel;

    while(ros::ok())
    {
        if(poseT > 90 * 180/M_PI)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.2;
        }
        else
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        vel_pub.publish(cmd_vel);
        ros::spinOnce();
    }
    return 0;
}
