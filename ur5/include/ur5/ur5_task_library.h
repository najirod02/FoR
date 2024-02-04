/**
 * the file contains all the functions
 * used in the task planning
 */

#ifndef UR5_TASK_LIBRARY_H
#define UR5_TASK_LIBRARY_H

#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ur5/ServiceMessage.h"
#include <string.h>
#include <math.h>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <list>

using namespace std;
using namespace Eigen;

/** 
 * define all the possible states in which the robot can be
*/
enum stato{
    start,//ask vision node to scan for blocks
    block_take, //get a block
    high_block_take,//going to take the block in elevated z
    high_block_release,//block released and moving away on z
    high_class_take,//block took and moving away on z
    high_class_release,//going to release point but on higher z
    class_release,//point of release of the block
    motion_error,//in case of error, use a intermediate safe point
    block_request,//ask next block
    no_more_blocks//no more blocks, terminate
};

const double GRIPPER_CLOSURE=-0.0639;//value of closed gripper
const double GRIPPER_OPEN = 0.3;//value of open gripper
const double GRASPING_HEIGHT=1.02;//safe height to take the block
const double SAFE_Z_MOTION = 1.3;//used for the intermediate movement between initial and final position

Vector3d euler,position;       
Vector3d pos[5];
Vector3d phi[5];
double xef[3];//position end effector to give to the motion
Vector3d xef_class;//position vector used for the class
double phief[3];//phi end effector to give to the motion
Vector3d phief_class;//phi vector used for the class
double gripper;//actual value of the gripper

int n_classes;//number of blocks to grab
int class_of_block[5];//6 values for 6 classes of blocks [1-6]
int k=0;//variable used in iteration
int error;//0:successfull motion  -- 1:unreachable block  -- 2:error in motion

const int ack=0;//variable used for service synchronization
int actual_ack=0;//variable used for service synchronization
int iteration;//variable used for service synchronization
int actual_iteration;//variable used for service synchronization

stato state;//current state
stato next_state;//next state used after the wait ackstate to proceed to the next step
int final_end=0;

/**
 * the function implements a state machine where we get the pose of 
 * a block and make requests to the motion in order to grab and move the block
 * in another known position
 * 
 * @state the state in which the machine is at the moment
 * @n the node handler to make request to the motion service
 */
void gestisciStato(stato &state,ros::NodeHandle n);


#endif