/*
simulates the process where we ask the vision node for the
position of an object using the topic chatter
*/

#include "ros/ros.h"
#include "target_pose/TargetPose.h"
#include "target_pose/GraspPosition.h"
#include "target_pose/GraspOrientation.h"
#include "target_pose/Quaternion.h"
#include <sstream>

//the callback function, like the handler for signals
void chatterCallback(const target_pose::TargetPose::ConstPtr& msg)
{
    target_pose::GraspPosition position = msg->position;
    target_pose::GraspOrientation orientation = msg->orientation;
    target_pose::Quaternion quaternion = msg->quaternion;

    ROS_INFO("---- GRASP POSITION heard -------------------------------------------");
    ROS_INFO("x: %f", position.position.at(0));
    ROS_INFO("y: %f", position.position.at(1));
    ROS_INFO("z: %f", position.position.at(2));

    ROS_INFO("---- GRASP ORIENTATION heard -------------------------------------------");
    ROS_INFO("alpha: %f", orientation.orientation.at(0));
    ROS_INFO("betha: %f", orientation.orientation.at(1));
    ROS_INFO("gamma: %f", orientation.orientation.at(2));

    ROS_INFO("---- QUATERNION heard -------------------------------------------");
    ROS_INFO("x: %f", quaternion.quaternion.x);
    ROS_INFO("y: %f", quaternion.quaternion.y);
    ROS_INFO("z: %f", quaternion.quaternion.z);
    ROS_INFO("w: %f", quaternion.quaternion.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion");
    ros::NodeHandle n;
    //set the subscriber indicating the topic, the frequency and the callback function
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}