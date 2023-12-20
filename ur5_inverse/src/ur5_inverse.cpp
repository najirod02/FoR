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
 * all the joint have a 360° mobility and the gripper has an (symetric) opening of 100mm
 * 45mm for each side
 * 
 * NOTE
 * remember to set on params.py the gripper_sim flag as TRUE
 * 
 * example of call:
 * joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
 * 
 * rosrun ur5_inverse ur5_inverse 0.1 -0.65 0.2 -0.4 -0.2 0.3
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
    Eigen::Matrix3d R60;//rotation matrix of the euler angles
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
            
            //start talker
            talker();    
        }

    private: 

        /**
         * given the final point and the rotation of the end effector
         * caculate the different solutions
         */
        Eigen::MatrixXd ur5Inverse(Eigen::Array3d &p60, Eigen::Matrix3d &Re){
            Eigen::MatrixXd Th(6, 8);
            
            // from euler angles to rotation matrix R60
            Eigen::AngleAxisd rollAngle(euler(0), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(euler(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(euler(2), Eigen::Vector3d::UnitZ());
            
            Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
            R60 = q.matrix();
            //std::cout << "R60\n " << R60 << std::endl;

            Eigen::Affine3d hmTransf = Eigen::Affine3d::Identity();
            hmTransf.translation() = p60;
            hmTransf.linear() = R60;
            T60 = hmTransf.matrix();
            //std::cout << "T60\n" << T60 << std::endl;
            
            //finding th1
            Eigen::Vector4d data;
            data << 0, 0, -D(5), 1;
            //trapose result to save in p50
            Eigen::ArrayXd p50 = (T60 * data).topRows(4);

            double psi = atan2(p50(1), p50(0));
            double p50xy = hypot(p50(1), p50(0));

            if(p50xy < D(3)){
                Eigen::MatrixXd ones(6,1);
                ones.setOnes();
                Th = ones * NAN;
                std::cout << "Position request in the unreachable cylinder" << std::endl;
                return Th;
            }

            double phi1_1 = acos(D(3) / p50xy);
            double phi1_2 = -phi1_1;

            double th1_1 = psi + phi1_1 + M_PI/2;
            double th1_2 = psi + phi1_2 + M_PI/2;

            double p61z_1 = p60(0) * sin(th1_1) - p60(1) * cos(th1_1);
            double p61z_2 = p60(0) * sin(th1_2) - p60(1) * cos(th1_2);

            double th5_1_1 = acos((p61z_1 - D(3)) / D(5));
            double th5_1_2 = -acos((p61z_1 - D(3)) / D(5));
            double th5_2_1 = acos((p61z_2 - D(3)) / D(5));
            double th5_2_2 = -acos((p61z_2 - D(3)) / D(5));

            Eigen::Matrix4d T10_1 = getRotationMatrix(th1_1, ALPHA(0), D(0), A(0));
            Eigen::Matrix4d T10_2 = getRotationMatrix(th1_2, ALPHA(0), D(0), A(0));

            Eigen::Matrix4d T16_1 = (T10_1.inverse()*T60).inverse();
            Eigen::Matrix4d T16_2 = (T10_2.inverse()*T60).inverse();

            double zy_1 = T16_1(1,2);
            double zx_1 = T16_1(0,2);

            double zy_2 = T16_2(1,2);
            double zx_2 = T16_2(0,2);
            double th6_1_1, th6_1_2, th6_2_1, th6_2_2;

            if(almostZero(sin(th5_1_1)) || (almostZero(zy_1) && almostZero(zx_1))){
                std::cout << "singual configuration. Choosing arbitrary th6" << std::endl;
                th6_1_1 = 0;
            } else {
                th6_1_1 = atan2((-zy_1 / sin(th5_1_1)), (zx_1 / sin(th5_1_1)));
            }

            if(almostZero(sin(th5_1_2)) || (almostZero(zy_1) && almostZero(zx_1))){
                std::cout << "singual configuration. Choosing arbitrary th6" << std::endl;
                th6_1_2 = 0;
            } else {
                th6_1_2 = atan2((-zy_1 / sin(th5_1_2)), (zx_1 / sin(th5_1_2)));
            }

            if(almostZero(sin(th5_2_1)) || (almostZero(zy_2) && almostZero(zx_2))){
                std::cout << "singual configuration. Choosing arbitrary th6" << std::endl;
                th6_2_1 = 0;
            } else {
                th6_2_1 = atan2((-zy_2 / sin(th5_2_1)), (zx_2 / sin(th5_2_1)));
            }

            if(almostZero(sin(th5_2_2)) || (almostZero(zy_2) && almostZero(zx_2))){
                std::cout << "singual configuration. Choosing arbitrary th6" << std::endl;
                th6_2_2 = 0;
            } else {
                th6_2_2 = atan2((-zy_2 / sin(th5_2_2)), (zx_2 / sin(th5_2_2)));
            }

            Eigen::Matrix4d T61_1 = T16_1.inverse();
            Eigen::Matrix4d T61_2 = T16_2.inverse();

            Eigen::Matrix4d T54_1_1 = getRotationMatrix(th5_1_1, ALPHA(4), D(4), A(4));
            Eigen::Matrix4d T54_1_2 = getRotationMatrix(th5_1_2, ALPHA(4), D(4), A(4));
            Eigen::Matrix4d T54_2_1 = getRotationMatrix(th5_2_1, ALPHA(4), D(4), A(4));
            Eigen::Matrix4d T54_2_2 = getRotationMatrix(th5_2_2, ALPHA(4), D(4), A(4));

            Eigen::Matrix4d T65_1_1 = getRotationMatrix(th6_1_1, ALPHA(5), D(5), A(5));
            Eigen::Matrix4d T65_1_2 = getRotationMatrix(th6_1_2, ALPHA(5), D(5), A(5));
            Eigen::Matrix4d T65_2_1 = getRotationMatrix(th6_2_1, ALPHA(5), D(5), A(5));
            Eigen::Matrix4d T65_2_2 = getRotationMatrix(th6_2_2, ALPHA(5), D(5), A(5));

            Eigen::Matrix4d T41_1_1 = T61_1 * (T54_1_1 * T65_1_1).inverse();
            Eigen::Matrix4d T41_1_2 = T61_1 * (T54_1_2 * T65_1_2).inverse();
            Eigen::Matrix4d T41_2_1 = T61_2 * (T54_2_1 * T65_2_1).inverse();
            Eigen::Matrix4d T41_2_2 = T61_2 * (T54_2_2 * T65_2_2).inverse();

            data << 0, -D(4), 0, 1;
            Eigen::ArrayXd P = (T41_1_1 * data).topRows(4);
            Eigen::Vector3d P31_1_1 = P.topRows(0).leftCols(3);

            P = (T41_1_2 * data).topRows(4);
            Eigen::Vector3d P31_1_2 = P.topRows(0).leftCols(3);
            
            P = (T41_2_1 * data).topRows(4);
            Eigen::Vector3d P31_2_1 = P.topRows(0).leftCols(3);
            
            P = (T41_2_2 * data).topRows(4);
            Eigen::Vector3d P31_2_2 = P.topRows(0).leftCols(3);
            
            double th3_1_1_1, th3_1_1_2, th3_1_2_1, th3_1_2_2, 
            th3_2_1_1, th3_2_1_2, th3_2_2_1, th3_2_2_2;

            double C = (pow(P31_1_1.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
            if(abs(C) > 1){
                std::cout << "Point out of the work space for th3_1_1\n";
                th3_1_1_1 = NAN;
                th3_1_1_2 = NAN;
            } else {
                th3_1_1_1 = acos(C);
                th3_1_1_2 = -acos(C);
            }

            C = (pow(P31_1_2.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
            if(abs(C) > 1){
                std::cout << "Point out of the work space for th3_1_2\n";
                th3_1_2_1 = NAN;
                th3_1_2_2 = NAN;
            } else {
                th3_1_2_1 = acos(C);
                th3_1_2_2 = -acos(C);
            }


            C = (pow(P31_2_1.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
            if(abs(C) > 1){
                std::cout << "Point out of the work space for th3_2_1\n";
                th3_2_1_1 = NAN;
                th3_2_1_2 = NAN;
            } else {
                th3_2_1_1 = acos(C);
                th3_2_1_2 = -acos(C);
            }


            C = (pow(P31_2_2.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
            if(abs(C) > 1){
                std::cout << "Point out of the work space for th3_2_2\n";
                th3_2_2_1 = NAN;
                th3_2_2_2 = NAN;
            } else {
                th3_2_2_1 = acos(C);
                th3_2_2_2 = -acos(C);
            }


            double th2_1_1_1, th2_1_1_2, th2_1_2_1, th2_1_2_2,
            th2_2_1_1, th2_2_1_2, th2_2_2_1, th2_2_2_2;

            th2_1_1_1 = -atan2(P31_1_1(1), -P31_1_1(0))+asin((A(2)*sin(th3_1_1_1))/P31_1_1.norm());
            th2_1_1_2 = -atan2(P31_1_1(1), -P31_1_1(0))+asin((A(2)*sin(th3_1_1_2))/P31_1_1.norm());
            th2_1_2_1 = -atan2(P31_1_2(1), -P31_1_2(0))+asin((A(2)*sin(th3_1_2_1))/P31_1_2.norm());
            th2_1_2_2 = -atan2(P31_1_2(1), -P31_1_2(0))+asin((A(2)*sin(th3_1_2_2))/P31_1_2.norm());
            th2_2_1_1 = -atan2(P31_2_1(1), -P31_2_1(0))+asin((A(2)*sin(th3_2_1_1))/P31_2_1.norm());
            th2_2_1_2 = -atan2(P31_2_1(1), -P31_2_1(0))+asin((A(2)*sin(th3_2_1_2))/P31_2_1.norm());
            th2_2_2_1 = -atan2(P31_2_2(1), -P31_2_2(0))+asin((A(2)*sin(th3_2_2_1))/P31_2_2.norm());
            th2_2_2_2 = -atan2(P31_2_2(1), -P31_2_2(0))+asin((A(2)*sin(th3_2_2_2))/P31_2_2.norm());
                
            Eigen::MatrixXd T21 = getRotationMatrix(th2_1_1_1, ALPHA(1), D(1), A(1));
            Eigen::MatrixXd T32 = getRotationMatrix(th3_1_1_1, ALPHA(2), D(2), A(2));
            Eigen::MatrixXd T41 = T41_1_1;
            Eigen::MatrixXd T43 = (T21 * T32).inverse() * T41;
            double xy = T43(1, 0);
            double xx = T43(0, 0);
            double th4_1_1_1 = atan2(xy, xx);

            T21 = getRotationMatrix(th2_1_1_2, ALPHA(1), D(1), A(1));
            T32 = getRotationMatrix(th3_1_1_2, ALPHA(2), D(2), A(2));
            T41 = T41_1_1;
            T43 = (T21 * T32).inverse() * T41;
            xy = T43(1, 0);
            xx = T43(0, 0);
            double th4_1_1_2 = atan2(xy, xx);

            T21 = getRotationMatrix(th2_1_2_1, ALPHA(1), D(1), A(1));
            T32 = getRotationMatrix(th3_1_2_1, ALPHA(2), D(2), A(2));
            T41 = T41_1_2;
            T43 = (T21 * T32).inverse() * T41;
            xy = T43(1, 0);
            xx = T43(0, 0);
            double th4_1_2_1 = atan2(xy, xx);

            T21 = getRotationMatrix(th2_1_2_2, ALPHA(1), D(1), A(1));
            T32 = getRotationMatrix(th3_1_2_2, ALPHA(2), D(2), A(2));
            T41 = T41_1_2;
            T43 = (T21 * T32).inverse() * T41;
            xy = T43(1, 0);
            xx = T43(0, 0);
            double th4_1_2_2 = atan2(xy, xx);

            T21 = getRotationMatrix(th2_2_1_1, ALPHA(1), D(1), A(1));
            T32 = getRotationMatrix(th3_2_1_1, ALPHA(2), D(2), A(2));
            T41 = T41_2_1;
            T43 = (T21 * T32).inverse() * T41;
            xy = T43(1, 0);
            xx = T43(0, 0);
            double th4_2_1_1 = atan2(xy, xx);

            T21 = getRotationMatrix(th2_2_1_2, ALPHA(1), D(1), A(1));
            T32 = getRotationMatrix(th3_2_1_2, ALPHA(2), D(2), A(2));
            T41 = T41_2_1;
            T43 = (T21 * T32).inverse() * T41;
            xy = T43(1, 0);
            xx = T43(0, 0);
            double th4_2_1_2 = atan2(xy, xx);

            T21 = getRotationMatrix(th2_2_2_1, ALPHA(1), D(1), A(1));
            T32 = getRotationMatrix(th3_2_2_1, ALPHA(2), D(2), A(2));
            T41 = T41_2_2;
            T43 = (T21 * T32).inverse() * T41;
            xy = T43(1, 0);
            xx = T43(0, 0);
            double th4_2_2_1 = atan2(xy, xx);

            T21 = getRotationMatrix(th2_2_2_2, ALPHA(1), D(1), A(1));
            T32 = getRotationMatrix(th3_2_2_2, ALPHA(2), D(2), A(2));
            T41 = T41_2_2;
            T43 = (T21 * T32).inverse() * T41;
            xy = T43(1, 0);
            xx = T43(0, 0);
            double th4_2_2_2 = atan2(xy, xx);

            Th.resize(6, 8);
            Th.row(0) << th1_1, th1_1, th1_1, th1_1, th1_2, th1_2, th1_2, th1_2;
            Th.row(1) << th2_1_1_1, th2_1_1_2, th2_1_2_1, th2_1_2_2, th2_2_2_1, th2_2_1_2, th2_2_2_1, th2_2_2_2;
            Th.row(2) << th3_1_1_1, th3_1_1_2, th3_1_2_1, th3_1_2_2, th3_2_2_1, th3_2_1_2, th3_2_2_1, th3_2_2_2;
            Th.row(3) << th4_1_1_1, th4_1_1_2, th4_1_2_1, th4_1_2_2, th4_2_2_1, th4_2_1_2, th4_2_2_1, th4_2_2_2;
            Th.row(4) << th5_1_1, th5_1_1, th5_1_2, th5_1_2, th5_2_2, th5_2_1, th5_2_2, th5_2_2;
            Th.row(5) << th6_1_1, th6_1_1, th6_1_2, th6_1_2, th6_2_2, th6_2_1, th6_2_2, th6_2_2;

            return Th;
        }

        /**
         * returns the homogEigen::Matrix4d getRotationMatrix(double th, double alpha, double d, double a){eneus matrix
         */
        Eigen::Matrix4d getRotationMatrix(double th, double alpha, double d, double a){
            Eigen::Matrix4d rotation;
            rotation << cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha), a*cos(th),
                        sin(th), cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th),
                        0, sin(alpha), cos(alpha), d,
                        0, 0, 0, 1;

            return rotation;                                        
        }

        bool almostZero(double value){
            return abs(value)<1e-7;
        }

        /**
         * removes the columns that contains at least 1 Nan
         */
        Eigen::MatrixXd purgeNanColumn(Eigen::MatrixXd matrix){
            Eigen::MatrixXd newMatrix(6, 0);
            int nColumns = 0;

            for (int col = 0; col < matrix.cols(); ++col) {
                bool hasNaN = matrix.col(col).array().isNaN().any();

                if (!hasNaN) {
                    newMatrix.conservativeResize(matrix.rows(), nColumns + 1);
                    newMatrix.col(nColumns) = matrix.col(col);
                    ++nColumns;
                }
            }

            return newMatrix;
        }

        /**
         * a simple functions that cretes the message and publish it through the 
         * topic.
         * note that the angles must be of RADIANT type !!!
         */
        void send_des_jstate(Eigen::ArrayXd q_des){
            std_msgs::Float64MultiArray msg;
            //in case the gripper_sim in params.py is True we need to specify 
            //2 other values to control the gap between the gripper
            //the first value is for the left side
            //the second value is for the right side
            //in this case we let the gripper completly closed
            q_des.conservativeResize(q_des.size() + 2);
            q_des(q_des.size()-2) = 0;
            q_des(q_des.size()-1) = 0;

            //std::cout << q_des << std::endl;

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

        void talker(){
            //std::cout << "pe " << std::endl << pe << std::endl;
            //std::cout << "euler " << std::endl << euler << std::endl;

            ros::init(argc, argv, "ur5_inverse", ros::init_options::AnonymousName);
            ros::NodeHandle n;

            //create and subscribe to topics
            joint_pub = n.advertise<std_msgs::Float64MultiArray>(TOPIC, 1000);
            sub = n.subscribe(TOPIC_SUB, 1000, &InversePublisher::receive_jstate, this);
 
            ros::Duration(1).sleep();//sleep for 1 seconds
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

            Eigen::MatrixXd Th = ur5Inverse(pe, R60);

            std::cout << "Th: " << std::endl << Th << std::endl;

            Th = purgeNanColumn(Th);
            //std::cout << "Purged\n" << Th << std::endl;

            //we can choose an arbitrary solution from Th
            //we know from purge that the remaining solutions are all valid
            //and doesn't contain a Nan value
            //send through topic the new joint values
            if(Th.cols() >= 1){
                std::cout << "Send motion\n";
                send_des_jstate(Th.col(0));
                //std::cout << Th.col(0) << std::endl;;
            } else {
                std::cout << "No possible motion\n";
            }

            //print final q state
            ros::Duration(1).sleep();
            ros::spinOnce();
            std::cout << "final q [ ";
            for(int i=0; i<q.size(); ++i){
                std::cout << q[i] << " ";
            }
            std::cout << "]" << std::endl;
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