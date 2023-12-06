/*
simulates the reading of pose of an object through
a camera and sends the data through a service
*/

#include "ros/ros.h"
#include "target_pose/Target.h"
#include "target_pose/TargetPose.h"
#include "target_pose/GraspPosition.h"
#include "target_pose/GraspOrientation.h"
#include "target_pose/Quaternion.h"
#include <random>
#include <sstream>

int counter = 0;

std::random_device rd; //obtain a random number from hardware
std::mt19937 gen(rd()); //seed the generator
std::uniform_real_distribution<> distr(-40.0, 300.0); //define the range

/*
this is the callback function, in this case there is no input for the service
so req will be null
*/
bool target(target_pose::Target::Request &req, target_pose::Target::Response &res){
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
    //only for debugging
    ROS_INFO("---- REQUEST N %d -------------------------------------------", counter);
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

    //set response and return true for no error
    res.pose = msg;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_service");
    ros::NodeHandle n;
    //set service "name" and the callback function
    ros::ServiceServer service = n.advertiseService("setTarget", target);
    ros::spin();//to remain in listening

    return 0;
}