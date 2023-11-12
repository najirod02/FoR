/*
simulates the process where we ask the vision node for the
position of an object
*/

#include "ros/ros.h"
#include "beginner_tutorials/TargetPose.h"
#include "beginner_tutorials/GraspPose.h"
#include "beginner_tutorials/GraspOrientation.h"
#include "beginner_tutorials/Quaternion.h"
#include <sstream>


//the callback function, like the handler for signals
void chatterCallback(const beginner_tutorials::TargetPose::ConstPtr& msg)
{
    beginner_tutorials::Quaternion quaternion = msg->quaternion;
    ROS_INFO("I heard w of quaternion: [%f]", quaternion.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}