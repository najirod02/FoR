/*
obtaining the actual joint configuration
subscribing at /ur5/joint_states topic published by gazebo

topic type 
    sensor_msgs/JointState

structure
    header: 
    seq: <int>
    stamp: 
        secs: <int>
        nsecs: <long?>
    frame_id: ''
    name: 
    - elbow_joint
    - shoulder_lift_joint
    - shoulder_pan_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
    position: [ 6 x float64 values]
    velocity: [6 x float64 values]
    effort: [6 x float64 values]
*/

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <algorithm>
#include <iterator>
#include <sstream>
#include <string.h>

std::string TOPIC("/ur5/joint_states");
int RATE = 1000; //Hz

void chatterCallback(const sensor_msgs::JointState::ConstPtr& joints)
{
    //instead of using for loops we copy using the iterator, the values on the output stream
    ROS_INFO("JOINT STATE\n");

    ROS_INFO("Names: ");
    std::copy(joints->name.begin(), joints->name.end(), std::ostream_iterator<std::string>(std::cout, ", "));
    ROS_INFO("\n");

    ROS_INFO("Position: ");
    std::copy(joints->position.begin(), joints->position.end(), std::ostream_iterator<double>(std::cout, ", "));
    ROS_INFO("\n");

    ROS_INFO("Velocity: ");
    std::copy(joints->velocity.begin(), joints->velocity.end(), std::ostream_iterator<double>(std::cout, ", "));
    ROS_INFO("\n");

    ROS_INFO("Effort: ");
    std::copy(joints->effort.begin(), joints->effort.end(), std::ostream_iterator<double>(std::cout, ", "));
    ROS_INFO("\n");
    ROS_INFO("\n------------------------------------------------------------------------------------");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(TOPIC, RATE, chatterCallback);
    ros::spin();
    return 0;
}