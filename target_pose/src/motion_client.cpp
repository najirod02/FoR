/*
simulates the process where we ask the vision node for the
pose of an object through a service
*/

#include "ros/ros.h"
#include "target_pose/Target.h"
#include "target_pose/TargetPose.h"
#include "target_pose/GraspPosition.h"
#include "target_pose/GraspOrientation.h"
#include "target_pose/Quaternion.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_client");
    ros::NodeHandle n;
    //set type and name of service
    ros::ServiceClient client = n.serviceClient<target_pose::Target>("setTarget");

    target_pose::Target srv;
    //no input for this request
    //srv.request.<param> = <value>

    //NOTE: uncommment the following lines if you want to send multiple requests from the client
    //ros::Rate loop_rate(1); //1 Hz

    //while(ros::ok()){
    if(client.call(srv)){
        
        ROS_INFO("---- GRASP POSITION heard -------------------------------------------");
        ROS_INFO("x: %f", srv.response.pose.position.position.at(0));
        ROS_INFO("y: %f", srv.response.pose.position.position.at(1));
        ROS_INFO("z: %f", srv.response.pose.position.position.at(2));

        ROS_INFO("---- GRASP ORIENTATION heard -------------------------------------------");
        ROS_INFO("alpha: %f", srv.response.pose.orientation.orientation.at(0));
        ROS_INFO("betha: %f", srv.response.pose.orientation.orientation.at(1));
        ROS_INFO("gamma: %f", srv.response.pose.orientation.orientation.at(2));

        ROS_INFO("---- QUATERNION heard -------------------------------------------");
        ROS_INFO("x: %f", srv.response.pose.quaternion.quaternion.x);
        ROS_INFO("y: %f", srv.response.pose.quaternion.quaternion.y);
        ROS_INFO("z: %f", srv.response.pose.quaternion.quaternion.z);
        ROS_INFO("w: %f", srv.response.pose.quaternion.quaternion.w);
    }
    else {
        ROS_INFO("Failed to call the service");
        return 1;
    }

        //loop_rate.sleep();
        //ros::spinOnce();
    //}

    return 0;
}