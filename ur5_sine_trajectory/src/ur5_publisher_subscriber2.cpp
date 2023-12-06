#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <algorithm>
#include <iterator>
#include <sstream>
#include <string.h>
#include <Eigen/Core>
#include <math.h>

/**
 * the functionality is the same as the ur5_publisher_subscriber but in this case
 * we use a class to manage the reading and comunication with the ur5 manipulator
*/
class JointStatePublisher{

    const static int JOINT_NAMES = 6;
    //the topic to send the new joints configurations
    const std::string TOPIC = std::string("/ur5/joint_group_pos_controller/command");
    //the topic to read the joints states
    const std::string TOPIC_SUB = std::string("/ur5/joint_states");
    const int RATE = 1000; //Hz default 1000

    ros::Publisher joint_pub;
    ros::Subscriber sub;
    std::string joint_names [JOINT_NAMES];

    Eigen::ArrayXd q;
    Eigen::ArrayXd q_des;
    Eigen::ArrayXd qd_des;
    Eigen::ArrayXd tau_ffwd;
    Eigen::ArrayXd filter_1;
    Eigen::ArrayXd filter_2;
    char **argv;
    int argc;
    bool gripper_sim;

    public:
        
        /** 
         * initialize all the data needed to read and generate the new joints configurations
        */
        JointStatePublisher(int argc, char **argv) :
        joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}{
            //initialize all the vectors
            q.resize(6);
            q_des.resize(6);
            qd_des.resize(6);
            tau_ffwd.resize(6);
            filter_1.resize(6);
            filter_2.resize(6);

            q << 0, 0, 0, 0, 0, 0;
            q_des << 0, 0, 0, 0, 0, 0;
            qd_des << 0, 0, 0, 0, 0, 0;
            tau_ffwd << 0, 0, 0, 0, 0, 0;
            filter_1 << 0, 0, 0, 0, 0, 0;
            filter_2 << 0, 0, 0, 0, 0, 0;

            //just copy the argc and argv of main to initialize the nodes in talker function
            argc = argc;
            argv = argv;

            //start talker
            talker();    
        }

    private:
        /** 
         * initialize filter 
        */
        void initFilter(Eigen::ArrayXd& q){
            for(int i=0; i<q.size(); ++i){
                filter_1[i] = q[i];
                filter_2[i] = q[i];
            }
        }
    
        /**
         * initialize second order filter to make a smooth sine trajectory
         */
        Eigen::ArrayXd secondOrderFilter(Eigen::ArrayXd& input, int rate, double settling_time){
            double dt = 1.0 / rate;
            double gain = dt / (0.1 * settling_time + dt);
            
            filter_1 = (1 - gain) * filter_1 + gain * input;
            filter_2 = (1 - gain) * filter_2 + gain * filter_1;

            return filter_2;
        }

        /**
         * a simple functions that cretes the message and publish it through the 
         * topic
         */
        void send_des_jstate(){
            std_msgs::Float64MultiArray msg;

            //in case the also want to control the grip, we need to insert 
            //3 other values to control RPY of end effector
            if(gripper_sim){
                q_des.conservativeResize(3);
                std::vector<double> v3(&q_des[0], q_des.data() + q_des.cols() * q_des.rows());
                msg.data = v3;
            }
            else {
                std::vector<double> v3(&q_des[0], q_des.data() + q_des.cols() * q_des.rows());
                msg.data = v3;
            }

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
         * initialize nodes and loops, reading and sending the new joint states
         */
        void talker(){
            //AnonymousName adds a random suffix to the node name
            ros::init(argc, argv, "custom_joint_pub_node", ros::init_options::AnonymousName);
            ros::NodeHandle n;

            //check if robot is real
            bool real_robot;
            n.getParam("real_robot", real_robot);

            if(real_robot)
                std::cout << "Robot is REAL" << std::endl;

            //create and subscribe to topics
            joint_pub = n.advertise<std_msgs::Float64MultiArray>(TOPIC, 1);
            sub = n.subscribe(TOPIC_SUB, 1, &JointStatePublisher::receive_jstate, this);
            
            ros::Duration(2).sleep();//sleep for 2 seconds
            ros::Rate loop_rate(RATE);
            ros::spinOnce();//IMPORTANT!!! to make sure that the initial configuration is read from the subscriber

            //init variables
            double time = 0;

            //print initial q state
            std::cout << "init q: [ ";
            for(int i=0; i<q.size(); ++i){
                std::cout << q[i] << " ";
            }
            std::cout << "]" << std::endl;

            Eigen::ArrayXd q0(6);
            Eigen::ArrayXd q_des0(6);
            q0 = q;
            initFilter(q0);
            
            //check if gripper is actuated
            gripper_sim;
            n.getParam("/gripper_sim", gripper_sim);
            //gripper_sim = 0 if false (treated as  a rigid body)

            Eigen::ArrayXd amp(6);
            Eigen::ArrayXd freq(6);
            Eigen::ArrayXd increment(6);

            //you can play with the values
            amp << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0;
            freq << 0.2, 0.0, 0.0, 0.0, 0.0, 0.0;
            increment << 0.0, -0.4, 0.0, 0.0, 0.0, 0.0;

            while(ros::ok()){
                if(time < 4.0){
                    //for the first 4 seconds, don't move the arm
                    q_des = q0;
                }
                else {
                    //change position
                    Eigen::ArrayXd input(6);
                    input = q0 + increment;
                    q_des = secondOrderFilter(input, RATE, 5);
                }

                qd_des << 0, 0, 0, 0, 0, 0;
                tau_ffwd << 0, 0, 0, 0, 0, 0;

                //print q destination
                std::cout << "[ ";
                for(int i=0; i<q_des.size(); ++i){
                    std::cout << q_des[i] << " ";
                }
                std::cout << "]" << std::endl;

                send_des_jstate();
                time+=(1.0/RATE);

                //spin and sync
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

};

int main(int argc, char **argv){
    JointStatePublisher myPub(argc, argv);
    return 0;
}