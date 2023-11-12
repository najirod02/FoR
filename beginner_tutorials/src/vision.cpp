/*
simulates the reading of position of an object though
a camera and sends the data through a service
*/

//TODO: the comunication is done with topics, the best way
//is to use services, look here: 
//https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

#include "ros/ros.h"
#include "beginner_tutorials/TargetPose.h"
#include "beginner_tutorials/GraspPose.h"
#include "beginner_tutorials/GraspOrientation.h"
#include "beginner_tutorials/Quaternion.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");//generate node
    ros::NodeHandle n;//handler for our node

    //creation of a topic and get a publisher in return
    ros::Publisher chatter_pub = n.advertise<beginner_tutorials::TargetPose>("chatter", 1000);

    //set frequency that the node loops at
    ros::Rate loop_rate(10);
    //in this case we write the same values each time
    float x = 100, y=50, z=34, w=13, alpha=23, betha=34.5, gamma=-67.3;
    
    while (ros::ok())
    {
        //creation of msg and publishing
        beginner_tutorials::TargetPose msg;
        beginner_tutorials::GraspPose pose;
        beginner_tutorials::GraspOrientation orientation;
        beginner_tutorials::Quaternion quaternion;

        pose.x=x;
        pose.y=y;
        pose.z=z;

        orientation.alpha=alpha;
        orientation.betha=betha;
        orientation.gamma=gamma;

        quaternion.x=x;
        quaternion.y=y;
        quaternion.z=z;
        quaternion.w=w;

        msg.graspPose = pose;
        msg.graspOrientation = orientation;
        msg.quaternion = quaternion;

        ROS_INFO("w value of quaternion: %f", msg.quaternion.w);//print on screen
        chatter_pub.publish(msg);

        ros::spinOnce();//usefull when having another node that subscribes to the topic
        loop_rate.sleep();
    }

    return 0;
}