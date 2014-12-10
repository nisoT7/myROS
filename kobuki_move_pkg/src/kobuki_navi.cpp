#include<iostream>
#include<math.h>
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>

using namespace std;

namespace{
    double poseX;
    double poseY;
    double poseT;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    poseX = odom->pose.pose.position.x;
    poseY = odom->pose.pose.position.y;
    poseT = odom->pose.pose.orientation.w;

    cout<<"x = "<<poseX<<"[m]"<<"\t";
    cout<<"y = "<<poseY<<"[m]"<<"\t";
    cout<<"theta = "<<((poseT*180/M_PI)/ 57.2949)*180<<"[deg]"<<endl;


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
        if(poseT * 180/M_PI < 90.0)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.5;
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
