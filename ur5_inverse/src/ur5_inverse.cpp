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

/**
 * translation of the matlab script ur5Inverse.m 
 * that permits to indicate the end effector pose (position + rotation)
 * 
 * NOTE
 * the inverse kinematic problem gives more than 1 solution and the one choosed
 * in arbitrarly
 * 
 * in case of singular configuration, an arbitrary value of the th6 will be choosed
 * 
 * in case of a point out of the work space, Nan values will be used
 * 
 * all the joint have a 360° mobility and the gripper has (only testes) 45° of opening
 * for each side 
 * 
 * example of call:
 * joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
 * 
 */
class InversePublisher{

    //the topic to send the new joints configurations
    const std::string TOPIC = std::string("/ur5/joint_group_pos_controller/command");
    //the topic to read the joints states
    const std::string TOPIC_SUB = std::string("/ur5/joint_states");

    const static int JOINT_NAMES = 6;
    const double SCALAR_FACTOR = 1.0;

    ros::Publisher joint_pub;
    ros::Subscriber sub;
    std::string joint_names [JOINT_NAMES];
    Eigen::ArrayXd A, D, ALPHA;

    //used to read the joint states
    Eigen::ArrayXd q;
    Eigen::Array3d pe;//the destination point
    Eigen::Array3d euler;//the rotation of e.e
    Eigen::Matrix4d T60;
    

    char **argv;
    int argc;
    bool gripper_sim;


    public:
        
        /** 
         * initialize all the data needed to read and generate the new joints configurations
        */
        InversePublisher(int argc_, char **argv_) :
        joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
        {
            //just copy the argc and argv of main to initialize the nodes in talker function
            argc = argc_;
            argv = argv_;

            //initialize all the vector
            A.resize(JOINT_NAMES);
            D.resize(JOINT_NAMES);
            ALPHA.resize(JOINT_NAMES);
            q.resize(JOINT_NAMES);

            //set values of the D-H parameters
            //A - D -> distance between joints
            //ALPHA -> rotation about x
            A << 0, -0.425, -0.3922, 0, 0, 0;
            D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
            ALPHA << M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0;
            A *= SCALAR_FACTOR;
            D *= SCALAR_FACTOR;

            //read the position and euler values
            pe << std::stod(argv_[1]), std::stod(argv_[2]), std::stod(argv_[3]);
            euler << std::stod(argv_[4]), std::stod(argv_[5]), std::stod(argv_[6]);
            
            //TODO: initialize T60 --> see slides or doc
            //T60 = [R60 p60; zeros(1,3) 1];
            //need to use R60 = euler BUT in a matrix
            //p60 = pe

            // from euler angles (ZYX convention - subsequent rotations about moving axes) to rotation matrix w_R_b
            Eigen::Matrix3d R60;
            R60 = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ());

            //start talker
            talker();    
        }

    private: 

        void ur5Inverse(Eigen::ArrayXd &pe, Eigen::MatrixXd &Re){}

        /**
         * returns the homogEigen::Matrix4d getRotationMatrix(double th, double alpha, double d, double a){eneus matrix
         */
        Eigen::Matrix4d getRotationMatrix(double th, double alpha, double d, double a){
            Eigen::Matrix4d rotation;
            rotation << cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha), a*cos(th);
                        sin(th), cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
                        0, sin(alpha), cos(alpha), d;
                        0, 0, 0, 1;

            return rotation;                                        
        }

        bool almostZero(double value){
            return abs(value)<1e-7;
        }

        void talker(){
            std::cout << "pe " << pe << std::endl;
            std::cout << "euler " << euler << std::endl;
        }

};

int main(int argc, char** argv){
    //in input we need 3 (position) + 3 (angles) values
    if(argc != 7){
        std::cout << "Insert exactly [x, y, z] for position and [A, B, Γ] for rotation (in grads)" << std::endl;
        return 1;
    }
    
    InversePublisher myPub(argc, argv);

    return 0;
}