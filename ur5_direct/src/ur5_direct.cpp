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
 * translation of the matlab script ur5Direct.m 
 * that permits to indicate the joint values and move the manipulator at the given
 * configuration
 * 
 * NOTE
 * there is NO CONTROL on possible SINGULARITIES and COLLISIONS so, use the script 
 * with precautions!!!!!
 * 
 * all the joint have a 360° mobility and the gripper has (only testes) 45° of opening
 * for each side 
 * 
 * example of call:
 * joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
 * 
 * gripper_sim False -> specify only the joint values
 * rosrun ur5_direct ur5_direct -90 -45 -120 90 -90 0
 * 
 * NOTE
 * the gripper is about millimeters and NOT angles (max opening = 100mm)
 * gripper_sim True -> specify joint values + gripper (2 values, left + right)
 * rosrun ur5_direct ur5_direct -90 -45 -120 90 -90 45 45
 */
class DirectPublisher{

    //the topic to send the new joints configurations
    const std::string TOPIC = std::string("/ur5/joint_group_pos_controller/command");
    //the topic to read the joints states
    const std::string TOPIC_SUB = std::string("/ur5/joint_states");

    const static int JOINT_NAMES = 6;
    const double SCALAR_FACTOR = 10.0;

    ros::Publisher joint_pub;
    ros::Subscriber sub;
    std::string joint_names [JOINT_NAMES];
    Eigen::ArrayXd A, D, ALPHA;

    //initial and final joint states
    Eigen::ArrayXd q;
    Eigen::ArrayXd q_des;

    char **argv;
    int argc;
    bool gripper_sim;

    public:
        
        /** 
         * initialize all the data needed to read and generate the new joints configurations
        */
        DirectPublisher(int argc_, char **argv_) :
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
            q_des.resize(argc-1);

            //set values of the D-H parameters
            //A - D -> distance between joints
            //ALPHA -> rotation about x
            A << 0, -0.425, -0.3922, 0, 0, 0;
            D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
            ALPHA << M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0;
            A *= SCALAR_FACTOR;
            D *= SCALAR_FACTOR;

            //read the destination values given in input from argv
            //convert from grads to rads
            for(int i=1; i<=argc-1; ++i){
                q_des(i-1) = std::stod(argv_[i]) * (M_PI / 180.0);
            }

            //start talker
            talker();    
        }

    private:

        /** 
         * Direct Kinematics of the UR5
         * Parameters:
         * Th: six joint angles
         * Returns
         * pe: cartesian position of the end effector
         * Re: Rotation matrix of the end effector
        */
        void ur5Direct(Eigen::ArrayXd &pe, Eigen::MatrixXd &Re){
            Eigen::Matrix4d t10 = getRotationMatrix(q_des(0), ALPHA(0), D(0), A(0));
            Eigen::Matrix4d t21 = getRotationMatrix(q_des(1), ALPHA(1), D(1), A(1));
            Eigen::Matrix4d t32 = getRotationMatrix(q_des(2), ALPHA(2), D(2), A(2));
            Eigen::Matrix4d t43 = getRotationMatrix(q_des(3), ALPHA(3), D(3), A(3));
            Eigen::Matrix4d t54 = getRotationMatrix(q_des(4), ALPHA(4), D(4), A(4));
            Eigen::Matrix4d t65 = getRotationMatrix(q_des(5), ALPHA(5), D(5), A(5));

            Eigen::Matrix4d t60 = t10*t21*t32*t43*t54*t65;

            pe = t60.block(0, 3, 3, 3);
            Re = t60.block(0, 3, 3, 3);
        }

        /**
         * returns the homogeneus matrix
         */
        Eigen::Matrix4d getRotationMatrix(double th, double alpha, double d, double a){
            Eigen::Matrix4d rotation;
            rotation << cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha), a*cos(th);
                        sin(th), cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
                        0, sin(alpha), cos(alpha), d;
                        0, 0, 0, 1;

            return rotation;                                        
        }

        /**
         * a simple functions that cretes the message and publish it through the 
         * topic.
         * note that the angles must be of RADIANT type !!!
         */
        void send_des_jstate(){
            std_msgs::Float64MultiArray msg;
            //in case the gripper_sim in params.py is True we need to specify 
            //2 other values to control the gap between the gripper
            //the first value is for the left side
            //the second value is for the right side
            std::vector<double> v3(&q_des[0], q_des.data() + q_des.cols() * q_des.rows());
            msg.data = v3;

            joint_pub.publish(msg);
        }
    

        /**
         * receive the joints states from the topic
         * need to check the correspondence of q index and position index
         * to make shure that we are reading the correct joint (names)
         */
        void receive_jstate(const sensor_msgs::JointState::ConstPtr& msg){
            for(int msg_idx=0; msg_idx<msg->name.size(); ++msg_idx){
                for(int joint_idx=0; joint_idx<JOINT_NAMES; ++joint_idx){
                    if(joint_names[joint_idx].compare(msg->name[msg_idx]) == 0){
                        q[joint_idx] = msg->position[msg_idx];
                    }
                }
            }
        }

        /**
         * main function,
         * initialize nodes, reading and sending the new joint states
         */
        void talker(){
            //AnonymousName adds a random suffix to the node name
            ros::init(argc, argv, "ur5_direct", ros::init_options::AnonymousName);
            ros::NodeHandle n;

            //check if robot is real
            bool real_robot;
            n.getParam("real_robot", real_robot);

            if(real_robot)
                std::cout << "Robot is REAL" << std::endl;

            //create and subscribe to topics
            joint_pub = n.advertise<std_msgs::Float64MultiArray>(TOPIC, 1000);
            sub = n.subscribe(TOPIC_SUB, 1000, &DirectPublisher::receive_jstate, this);
 
            ros::Duration(2).sleep();//sleep for 2 seconds
            ros::spinOnce();//IMPORTANT!!! to make sure that the initial configuration is read from the subscriber

            //print initial q state
            std::cout << "initial q [ ";
            for(int i=0; i<q.size(); ++i){
                std::cout << q[i] << " ";
            }
            std::cout << "]" << std::endl;

            //check if gripper is actuated
            gripper_sim;
            n.getParam("/gripper_sim", gripper_sim);
            //gripper_sim = 0 if false (treated as  a rigid body)

            //calculate final position of end effector
            Eigen::ArrayXd pe;
            Eigen::MatrixXd Re;
            pe.resize(3);
            Re.resize(3,3);

            ur5Direct(pe, Re);

            //print cartesian position of the end effector
            std::cout << "pe [ ";
            for(int i=0; i<pe.size(); ++i){
                std::cout << pe[i] << " ";
            }
            std::cout << "]" << std::endl;
            
            //send through topic the new joint values
            send_des_jstate();
 
            //print final q state
            ros::spinOnce();
            std::cout << "final q [ ";
            for(int i=0; i<q.size(); ++i){
                std::cout << q[i] << " ";
            }
            std::cout << "]" << std::endl;
        }

};

int main(int argc, char **argv){
    if(argc < 7) {
        std::cerr << "Insert 6 angles of the joints (in grads) " << std::endl << "[if gripper_sim is set on True add other 2 values for the gripper]" << std::endl;
        return 1;
    }

    DirectPublisher myPub(argc, argv);
    return 0;
}