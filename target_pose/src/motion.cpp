/*
simulates the process where we ask the vision node for the
position of an object
*/

#include "ros/ros.h"
#include "target_pose/TargetPose.h"
#include "target_pose/GraspPose.h"
#include "target_pose/GraspOrientation.h"
#include "target_pose/Quaternion.h"
#include <sstream>


//the callback function, like the handler for signals
void chatterCallback(const target_pose::TargetPose::ConstPtr& msg)
{
    target_pose::Quaternion quaternion = msg->quaternion;
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