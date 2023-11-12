#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");//generate node
    ros::NodeHandle n;//handler for our node

    //creation of a topic and get a publisher in return
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    //set frequency that the node loops at
    ros::Rate loop_rate(10);
    int count = 0;
    
    while (ros::ok())
    {
        //creation of msg and publishing
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());//print on screen
        chatter_pub.publish(msg);

        ros::spinOnce();//usefull when having another node that subscribes to the topic
        loop_rate.sleep();
        ++count;
    }

    return 0;
}