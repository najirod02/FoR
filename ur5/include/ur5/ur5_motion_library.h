/**
 * the file contains all the functions
 * that are used in order to compute the trajectory of the robot
 * and also make some checks in order to avoid singularities and go outside
 * the designed workin area.
 */

#ifndef UR5_INVDIFF_LIBRARY_H
#define UR5_INVDIFF_LIBRARY_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "ur5/ServiceMessage.h"    
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <string.h>
#include <math.h>
#include <list>
#include <iostream>

using namespace std;
using namespace Eigen;

class InverseDifferential   
{
    const double Tf=10.0;
    const double Tb=0;
    const double deltaT=0.1;//the smaller, the more precise but more computation

    //the topic to send the new joints configurations
    const std::string TOPIC = std::string("/ur5/joint_group_pos_controller/command");
    //the topic to read the joints states
    const std::string TOPIC_SUB = std::string("/ur5/joint_states");

    int iteration=0; //number of iteration
    int intermediate_point_trajectory=0;

    const static int JOINT_NAMES = 6;
    const double SCALAR_FACTOR = 1.0;
    //used in the damped pseudoinverse matrix
    //const double DAMPING_FACTOR = pow(10, -1.15);//determinant method
    //TODO: diminuire valore [-1.001; -1]
    const double DAMPING_FACTOR = pow(10, -1.001);//using eigenvalues method
    const double ALMOST_ZERO = 1e-7;//threshold when a values is recognized as zero
    const int RATE = 1000;//set to 100 to slow down

    //range for working area    
    //in the robot frame
    const double MAX_X = 0.5;
    const double MIN_X = -0.5;
    const double MAX_Y = 0.12;
    const double MIN_Y = -0.45;
    const double MAX_Z = 0.733;
    const double MIN_Z = 0.15;

    //transformation from world to robot frame
    const double WORLD_TO_ROBOT_X = -0.5;
    const double WORLD_TO_ROBOT_Y = 0.35;
    const double WORLD_TO_ROBOT_Z = 1.75;
    Matrix4d WORLD_TO_ROBOT;

    //global values
    Quaterniond q0,qf;
    Vector3d xe0,xef,phie0,phief,xe_intermediate,phie_intermediate;
    Vector2d gripper;
    VectorXd q, q_des;
    VectorXd A, D, ALPHA;

    ros::Publisher joint_pub;
    ros::Subscriber sub;
    std::string joint_names [JOINT_NAMES];

    char **argv;
    int argc;

    //ack/ack2 are used to syncronize the service 
    const int ack=0;
    int ack2=1;
    int error=0;//no error at start
    int final_end=0;//!=0 if there are no other blocks

    public:
        InverseDifferential(int argc_, char** argv_);

        //ROS functions

        /**
         * a service that receives the pose in which the robot must arrive
         * and responds if the position has been reached or not
         */
        bool motionPlannerToTaskPlannerServiceResponse(ur5::ServiceMessage::Request &req,ur5::ServiceMessage::Response &res);

        /**
         * a simple functions that cretes the message and publish it through the 
         * topic.
         */
        void send_des_jstate(ros::Publisher joint_pub, Eigen::VectorXd q_des);

        /**
         * receive the joints states from the topic
         * need to check the correspondence of q index and position index
         * to make shure that we are reading the correct joint (names)
         */
        void receive_jstate(const sensor_msgs::JointState::ConstPtr& msg);

        /**
         * main function,
         * initialize nodes, reading and sending the new joint states
         */
        int talker();

        /**
         * main function that starts the computation of the trajectory
         */
        bool invDiff();


        //Convertion functions

        /**
         * function that convert the euler angles in a quaternion
         * using the XYZ convention
         */
        Eigen::Matrix3d eulerAnglesToRotationMatrix(const Eigen::Vector3d& euler);

        /**
         * function that gives the euler angles XYZ from the rotation
         * matrix
         */
        Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d& rotationMatrix);

        /**
         * function that convert the euler angles XYZ in a quaternion
         */
        Eigen::Quaterniond eulerAnglesToQuaternion(const Eigen::Vector3d& euler);

        Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& rotationMatrix);

        /**
         * given the pose of a block in the world frame, return the pose of the same
         * block in the robot frame.
         * 
         * The coordinates and orientation has the same unit of gazebo (in meters and radiants)
         * 
         * Useful when specifing the block pose w.r.t world frame
         */
        void worldToRobotFrame(Eigen::Vector3d& coords, Eigen::Vector3d& euler);


        //Inverse differential functions

        /**
         * given the joints values, return the end effector position
         * and the rotation matrix
         */
        void ur5Direct(Eigen::Vector3d &xe, Eigen::Matrix3d &Re,const Eigen::VectorXd q_des);

        /**
         * given the joints values, returns the computet jacobian
         * for the ur5
         */
        Eigen::MatrixXd ur5Jac(Eigen::VectorXd v);

        /**
         * first of the two functions that generates the trajectory for the robot
         */
        list<Eigen::VectorXd> invDiffKinematicControlSimCompleteQuaternion(Eigen::VectorXd TH0,Eigen::VectorXd T,Eigen::Matrix3d Kp,Eigen::Matrix3d Kphi);

        /**
         * second of the two functions that generates the trajectory for the robot
         */
        Eigen::VectorXd invDiffKinematicControlCompleteQuaternion(Eigen::VectorXd qk,Eigen::Vector3d xe,Eigen::Vector3d xd,Eigen::Vector3d vd,Eigen::Vector3d omegad,Eigen::Quaterniond qe,Eigen::Quaterniond qd,Eigen::Matrix3d Kp,Eigen::Matrix3d Kphi);

        /**
         * position as a function over time
         * using linear interpolation between
         * 2 points
         */
        Eigen::Vector3d xd(double ti);

        /**
         * implement the slerp method
         */
        Eigen::Quaterniond qd(double ti);

        /**
         * given position and orientation, returns the possible
         * joints' configurations
         */
        Eigen::MatrixXd ur5Inverse(Eigen::Vector3d &p60, Eigen::Matrix3d &Re);

        /**
         * compute the homogeneous matrix for the computation of the direct
         * kinematic
         */
        Eigen::Matrix4d getRotationMatrix(double th, double alpha, double d, double a);

        /**
         * remove all the NON possible solutions from the inverse problem
         * checking if each columns contains at least a NaN value
         */
        Eigen::MatrixXd purgeNanColumn(Eigen::MatrixXd matrix);


        //Other functions

        /**
         * function that controls if the end effector is inside the working area
         */
        bool checkWorkArea(const Eigen::Vector3d& position);

        /**
         * function that controls if the given joint configuration, the end
         * effector in inside the working area
         */
        bool checkWorkArea(const Eigen::VectorXd& joints);

        /**
         * print the values of a generic vector
         */
        void printVector(Eigen::VectorXd v);

        /**
         * function that checks if a value is close to zero
         */
        bool almostZero(double value);

        /**
         * function that checks if the angle is close to zero.
         * if so, set the value to zero
         */
        void angleCorrection(double & angle);

        /**
         * the wirst's joint can reach its limits, given the desired configuration
         * bring back in the valid range the joint value -2pi;2pi
         */
        void fixWirstJointLimits(Eigen::VectorXd& joints);
};

#endif