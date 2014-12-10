/*
 * kobukiのモータをONにするパッケージ
 * 参考 : https://github.com/yujinrobot/kobuki/blob/hydro/kobuki_keyop/src/keyop_core.cpp
 */

#include<ros/ros.h>
#include<ecl/time.hpp>
#include<ecl/exceptions.hpp>
#include<kobuki_msgs/MotorPower.h>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"kobuki_motor");
    ros::NodeHandle node;

    ros::Publisher motor_power_pub = node.advertise<kobuki_msgs::MotorPower>("motor_power",1);

    /** kobuki connection **/
    bool wait_for_connection_ = true;
    if(!wait_for_connection_) return true;

    ecl::MilliSleep millisleep;
    int count = 0;
    bool connected = false;
    bool power_status = false;

    while(!connected)
    {
        if(motor_power_pub.getNumSubscribers() > 0){
            connected = true;
            break;
        }
        if(count == 6)
        {
            connected = false;
            break;
        }
        else
        {
            ROS_WARN_STREAM("could not connect, trying again after 500ms");
            try
            {
                millisleep(500);
            }
            catch(ecl::StandardException e)
            {
                ROS_ERROR_STREAM("Waiting has been interrupted.");
                ROS_DEBUG_STREAM(e.what());
                return false;
            }
            ++count;
        }
    }
    if(!connected)
    {
        ROS_ERROR("Could not connect.");
    }
    else
    {
        kobuki_msgs::MotorPower power_cmd;
        power_cmd.state = kobuki_msgs::MotorPower::ON;
        ROS_INFO("connected.");
        power_status = true;
    }
    while(ros::ok())
    {
        ros::spinOnce();
    }
}
