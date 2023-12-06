#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <algorithm>
#include <iterator>
#include <sstream>
#include <string.h>
#include <Eigen/Core>
#include <math.h>

std::string TOPIC("/ur5/joint_group_pos_controller/command");
std::string TOPIC_SUB("/ur5/joint_states");
int JOINT_NAMES = 6;
int RATE = 1000; //Hz default 1000
int argc_copy;
char **argv_copy;
std::string joint_names [] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
Eigen::ArrayXd q(6);

//initialize vectors with default values
void init(Eigen::ArrayXd& q_des, Eigen::ArrayXd& qd_des,
Eigen::ArrayXd& tau_ffwd, Eigen::ArrayXd& filter_1, Eigen::ArrayXd& filter_2){
    q << 0, 0, 0, 0, 0, 0;
    q_des << 0, 0, 0, 0, 0, 0;
    qd_des << 0, 0, 0, 0, 0, 0;
    tau_ffwd << 0, 0, 0, 0, 0, 0;
    filter_1 << 0, 0, 0, 0, 0, 0;
    filter_2 << 0, 0, 0, 0, 0, 0;
}

//initialize filter
void initFilter(Eigen::ArrayXd& filter_1, Eigen::ArrayXd& filter_2, Eigen::ArrayXd& q){
    for(int i=0; i<q.size(); ++i){
        filter_1[i] = q[i];
        filter_2[i] = q[i];
    }
}

//initialize second order filter to make a smooth sine trajectory
Eigen::ArrayXd secondOrderFilter(Eigen::ArrayXd& filter_1, Eigen::ArrayXd& filter_2, Eigen::ArrayXd& input, int rate, double settling_time){
    double dt = 1.0 / rate;
    double gain = dt / (0.1 * settling_time + dt);
    
    filter_1 = (1 - gain) * filter_1 + gain * input;
    filter_2 = (1 - gain) * filter_2 + gain * filter_1;

    return filter_2;
}

//send data
void send_des_jstate(ros::Publisher& joint_pub, bool gripper_sim, Eigen::ArrayXd& q_des){
    std_msgs::Float64MultiArray msg;

    if(gripper_sim){
        q_des.conservativeResize(3);
        std::vector<double> v3(&q_des[0], q_des.data()+q_des.cols()*q_des.rows());
        msg.data = v3;
    }
    else {
        std::vector<double> v3(&q_des[0], q_des.data()+q_des.cols()*q_des.rows());
        msg.data = v3;
    }

    joint_pub.publish(msg);
}

void receive_jstate(const sensor_msgs::JointState::ConstPtr& msg){
    for(int msg_idx=0; msg_idx<msg->name.size(); ++msg_idx){
        for(int joint_idx=0; joint_idx<JOINT_NAMES; ++joint_idx){
            if(joint_names[joint_idx].compare(msg->name[msg_idx]) == 0){
                q[joint_idx] = msg->position[msg_idx];
            }
        }
    }
}

//main function that publish the trajectory to follow
void talker(Eigen::ArrayXd& qd_des, Eigen::ArrayXd& tau_ffwd, 
    Eigen::ArrayXd& filter_1, Eigen::ArrayXd& filter_2){
    ros::init(argc_copy, argv_copy, "custom_joint_pub_node", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    //check if robot is real
    bool real_robot;
    n.getParam("real_robot", real_robot);

    if(real_robot)
        std::cout << "Robot is REAL" << std::endl;

    ros::Publisher joint_pub = n.advertise<std_msgs::Float64MultiArray>(TOPIC, 1);
    ros::Subscriber sub = n.subscribe(TOPIC_SUB, 1, receive_jstate);

    ros::Duration(2).sleep();//sleep for 2 seconds
    ros::Rate loop_rate(RATE);
    ros::spinOnce();//to make sure that the initial configuration is read from the subscriber

    //init variables
    double time = 0;

    std::cout << "init q: [ ";
    for(int i=0; i<q.size(); ++i){
        std::cout << q[i] << " ";
    }
    std::cout << "]" << std::endl;

    Eigen::ArrayXd q0(6);
    Eigen::ArrayXd q_des0(6);
    Eigen::ArrayXd q_des(6);
    q0 = q;
    initFilter(filter_1, filter_2, q0);
    
    //check if gripper is actuated
    bool gripper_sim;
    n.getParam("/gripper_sim", gripper_sim);
    //gripper_sim = 0 if false (treated as  a rigid body)

    Eigen::ArrayXd amp(6);
    Eigen::ArrayXd freq(6);
    Eigen::ArrayXd increment(6);

    amp << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0;
    freq << 0.2, 0.0, 0.0, 0.0, 0.0, 0.0;
    increment << 0.0, -0.4, 0.0, 0.0, 0.0, 0.0;

    while(ros::ok()){
        if(time < 4.0){
            q_des = q0;
        }
        else {
            Eigen::ArrayXd input(6);
            input = q0 + increment;
            q_des = secondOrderFilter(filter_1, filter_2, input, RATE, 5);
        }

        //2 - generate sine reference
        //q_des = q_des0 + amp * sin(2*pi*RATE*time);

        qd_des << 0, 0, 0, 0, 0, 0;
        tau_ffwd << 0, 0, 0, 0, 0, 0;

        //print q destination
        std::cout << "[ ";
        for(int i=0; i<q_des.size(); ++i){
            std::cout << q_des[i] << " ";
        }
        std::cout << "]" << std::endl;

        send_des_jstate(joint_pub, gripper_sim, q_des);
        time+=(1.0/RATE);

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int main(int argc, char **argv){
  
    argc_copy = argc;
    argv_copy = argv;
    Eigen::ArrayXd q_des(6);
    Eigen::ArrayXd qd_des(6);
    Eigen::ArrayXd tau_ffwd(6);
    Eigen::ArrayXd filter_1(6);
    Eigen::ArrayXd filter_2(6);

    init(q_des, qd_des, tau_ffwd, filter_1, filter_2);
    talker(qd_des, tau_ffwd, filter_1, filter_2);

    return 0;
}