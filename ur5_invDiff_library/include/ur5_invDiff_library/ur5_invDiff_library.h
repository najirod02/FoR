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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <string.h>
#include <math.h>

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
    const double DAMPING_FACTOR = 1e-6;//used in the damped pseudoinverse matrix
    const double ALMOST_ZERO = 1e-7;//threshold when a values is recognized as zero
    const int RATE = 100;//default: 1 kHz

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
    bool gripper_sim;

    public:
        InverseDifferential(int argc_, char** argv_);

        //ROS functions
        void send_des_jstate(ros::Publisher joint_pub, bool gripper_sim, Eigen::VectorXd q_des);

        void receive_jstate(const sensor_msgs::JointState::ConstPtr& msg);

        void talker();


        //Convertion functions
        Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d& rotationMatrix);

        Eigen::Matrix3d eulerAnglesToRotationMatrix(const Eigen::Vector3d& euler);

        Eigen::Quaterniond eulerAnglesToQuaternion(const Eigen::Vector3d& euler);

        Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& rotationMatrix);

        void worldToRobotFrame(Eigen::Vector3d& coords, Eigen::Vector3d& euler);


        //Inverse differential functions
        void ur5Direct(Eigen::Vector3d &xe, Eigen::Matrix3d &Re,const Eigen::VectorXd q_des);

        Eigen::MatrixXd ur5Jac(Eigen::VectorXd v    );

        list<Eigen::VectorXd> invDiffKinematicControlSimCompleteQuaternion(Eigen::VectorXd TH0,Eigen::VectorXd T,Eigen::Matrix3d Kp,Eigen::Matrix3d Kphi);

        Eigen::VectorXd invDiffKinematicControlCompleteQuaternion(Eigen::VectorXd qk,Eigen::Vector3d xe,Eigen::Vector3d xd,Eigen::Vector3d vd,Eigen::Vector3d omegad,Eigen::Quaterniond qe,Eigen::Quaterniond qd,Eigen::Matrix3d Kp,Eigen::Matrix3d Kphi);

        Eigen::Vector3d xd(double ti);

        Eigen::Quaterniond qd(double ti);

        Eigen::MatrixXd ur5Inverse(Eigen::Vector3d &p60, Eigen::Matrix3d &Re);

        Eigen::Matrix4d getRotationMatrix(double th, double alpha, double d, double a);

        Eigen::MatrixXd purgeNanColumn(Eigen::MatrixXd matrix);


        //Other functions
        bool checkWorkArea(const Eigen::Vector3d& position);

        bool checkWorkArea(const Eigen::VectorXd& joints);

        void printVector(Eigen::VectorXd v);

        bool almostZero(double value);

        void angleCorrection(double & angle);

        void fixWirstJointLimits(Eigen::VectorXd& joints);
};

#endif