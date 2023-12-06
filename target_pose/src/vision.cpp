/*
simulates the reading of pose of an object through
a camera and sends the data through a topic
*/

#include "ros/ros.h"
#include "target_pose/TargetPose.h"
#include "target_pose/GraspPosition.h"
#include "target_pose/GraspOrientation.h"
#include "target_pose/Quaternion.h"
#include <sstream>
#include <random>

int main(int argc, char **argv)
{
    std::random_device rd;//obtain a random number from hardware
    std::mt19937 gen(rd());//seed the generator
    std::uniform_real_distribution<> distr(-40.0, 300.0);//define the range

    ros::init(argc, argv, "vision");//generate node
    ros::NodeHandle n;//handler for our node

    //creation of a topic and get a publisher in return
    ros::Publisher chatter_pub = n.advertise<target_pose::TargetPose>("chatter", 1000);

    //set frequency that the node loops at
    ros::Rate loop_rate(10);
    int counter = 0;
    
    while (ros::ok())
    {
        //creation of msg and publishing
        ++counter;
        target_pose::TargetPose msg;
        target_pose::GraspPosition position;
        target_pose::GraspOrientation orientation;
        target_pose::Quaternion quaternion;

        //grasp position
        position.position.push_back(distr(gen));
        position.position.push_back(distr(gen));
        position.position.push_back(distr(gen));
    
        //graspo orientation
        orientation.orientation.push_back(distr(gen));
        orientation.orientation.push_back(distr(gen));
        orientation.orientation.push_back(distr(gen));

        //quaternion values
        quaternion.quaternion.x = (distr(gen));
        quaternion.quaternion.y = (distr(gen));
        quaternion.quaternion.z = (distr(gen));
        quaternion.quaternion.w = (distr(gen));
        
        msg.position = position;
        msg.orientation = orientation;
        msg.quaternion = quaternion;

        //print on screen
        ROS_INFO("---- GENERATION N %d -------------------------------------------", counter);
        ROS_INFO("---- GRASP POSITION -------------------------------------------");
        ROS_INFO("x: %f", msg.position.position.at(0));
        ROS_INFO("y: %f", msg.position.position.at(1));
        ROS_INFO("z: %f", msg.position.position.at(2));

        ROS_INFO("---- GRASP ORIENTATION -------------------------------------------");
        ROS_INFO("alpha: %f", msg.orientation.orientation.at(0));
        ROS_INFO("betha: %f", msg.orientation.orientation.at(1));
        ROS_INFO("gamma: %f", msg.orientation.orientation.at(2));

        ROS_INFO("---- QUATERNION -------------------------------------------");
        ROS_INFO("x: %f", msg.quaternion.quaternion.x);
        ROS_INFO("y: %f", msg.quaternion.quaternion.y);
        ROS_INFO("z: %f", msg.quaternion.quaternion.z);
        ROS_INFO("w: %f", msg.quaternion.quaternion.w);

        chatter_pub.publish(msg);
        ros::spinOnce();//usefull when having another node that subscribes to the topic
        loop_rate.sleep();//to sync to the loop rate
    }

    return 0;
}