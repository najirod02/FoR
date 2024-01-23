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

using namespace Eigen;
using namespace std;

/**
 * translation of the matlab script testUr57b.m 
 * 
 * NOTE
 * the singularities should be controlled by the fact we are using
 * quaternions
 * 
 * all the joint have a 360° mobility and the gripper has an (symetric) opening of 100mm
 * 45mm for each side
 * 
 * example of call:
 * joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
 
 * NOTE
 * the gripper is about millimeters and NOT angles (max opening = 100mm)
 * gripper_sim True -> specify joint values + gripper (2 values, left + right)
 * rosrun ur5_invDiff ur5_invDiff 0.4 -0.35 0.53 0 0 0 45 45
 * 
 * this command should aproach the block X1-Y1-Z1 from tavolo_blocks.world
 * 
 * to grap the object simply indicate the opening of -0.1 -0.1
 * after that, you can move the object where you want
 */
class InverseDifferential{

    const double Tf=10.0;
    const double Tb=0;
    const double deltaT=0.1;

    int sp=0;

    //the topic to send the new joints configurations
    const std::string TOPIC = std::string("/ur5/joint_group_pos_controller/command");
    //the topic to read the joints states
    const std::string TOPIC_SUB = std::string("/ur5/joint_states");

    const static int JOINT_NAMES = 6;
    const double SCALAR_FACTOR = 10.0;
    const double DAMPING_FACTOR = 1e-6;
    const int RATE = 1000;

    Quaterniond q0,qf;
    Vector3d xe0,xef,phie0,phief;
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

        InverseDifferential(int argc_, char **argv_) :
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
            q_des.resize(JOINT_NAMES);

            //set values of the D-H parameters
            //A - D -> distance between joints
            //ALPHA -> rotation about x
            A << 0, -0.425, -0.3922, 0, 0, 0;
            D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
            ALPHA << M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0;
            A *= SCALAR_FACTOR;
            D *= SCALAR_FACTOR;

            //read the position and euler values of final pose
            xef << std::stod(argv_[1]), std::stod(argv_[2]), std::stod(argv_[3]);
            phief << std::stod(argv_[4]), std::stod(argv_[5]), std::stod(argv_[6]);
            gripper << std::stod(argv_[7]), std::stod(argv_[8]);
            
            xef *= SCALAR_FACTOR;

            //start talker
            talker();    
        }

    private:

        /**
         * a simple functions that cretes the message and publish it through the 
         * topic.
         * note that the angles must be of RADIANT type !!!
         */
        void send_des_jstate(ros::Publisher joint_pub, bool gripper_sim, VectorXd q_des){
            std_msgs::Float64MultiArray msg;

            if(gripper_sim){
                q_des.conservativeResize(q_des.size() + 2);
                q_des(q_des.size()-2) = gripper[0];
                q_des(q_des.size()-1) = gripper[1];

                std::vector<double> v3(&q_des[0], q_des.data()+q_des.cols()*q_des.rows());
                msg.data = v3;
            }
            else {
                std::vector<double> v3(&q_des[0], q_des.data()+q_des.cols()*q_des.rows());
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
         * initialize nodes, reading and sending the new joint states
         */
        void talker(){
            //AnonymousName adds a random suffix to the node name
            ros::init(argc, argv, "ur5_invDiff", ros::init_options::AnonymousName);
            ros::NodeHandle n;
            ros::Publisher joint_pub = n.advertise<std_msgs::Float64MultiArray>(TOPIC, 1000);
            ros::Subscriber sub = n.subscribe(TOPIC_SUB, 1000, &InverseDifferential::receive_jstate, this);
            ros::Rate loop_rate(RATE);

            ros::Duration(1).sleep();
            ros::spinOnce();//IMPORTANT!!! to make sure that the initial configuration is read from the subscriber


            //check as first if the final position is possible
            Matrix3d Rf = eul2rotm(phief);
            MatrixXd TH0 = ur5Inverse(xef, Rf);
            cout << "THO" << endl << TH0 << endl << endl;
            MatrixXd M = purgeNanColumn(TH0);

            if(M.cols() == 0) {cout << "Motion not possible\n"; return;}


            //print initial q state
            VectorXd qstart(6);
            std::cout << "initial q [ ";
            for(int i=0; i<q.size(); ++i){
                std::cout << q[i] << " ";
                qstart(i) = q(i);
            }
            std::cout << "]" << std::endl;

            //get initial pose of the robot knowing the initial joint states
            Matrix3d Re;
            //cout << "q\n" << q << endl;
            ur5Direct(xe0, Re, qstart);
            phie0 = rotationMatrixToEulerAngles(Re);

            cout << "qstart\n" << qstart << endl;
            cout << "Re0\n" << Re << endl;
            cout << "phie0\n" << phie0.transpose() << endl;
            cout << "phief\n" << phief.transpose() << endl << endl;
            
            /**
            Quaterniond q_test = eul2quat(phie0);
            Vector3d euler_of_q = q_test.toRotationMatrix().eulerAngles(0,1,2);
            cout << "verify convertion\n" << q_test.coeffs() << endl << euler_of_q << endl << phie0 << endl;
            */
            
            //from the initial configuration
            q0 = eul2quat(phie0).conjugate();
            //from the desired final configuration
            qf = eul2quat(phief).conjugate();
            cout << "q0\n" << q0.coeffs().transpose() << endl << "qf\n" << qf.coeffs().transpose() << endl << endl << endl;

            //calculate the trajectory of robot by a function of time
            //and quaternions
            VectorXd T;
            double L=(Tf-Tb)/deltaT;
            T.resize(L+1);
            double time=0;

            for(int i=0;i<=L;i++){
                T(i)=time;
                time +=0.1;
            } 

            Matrix3d Kp = 10*MatrixXd::Identity(3,3);
            Matrix3d Kq = -10*MatrixXd::Identity(3,3);
            
            list <VectorXd> l;

            l = invDiffKinematicControlSimCompleteQuaternion(q,T, Kp, Kq); 

            int k =0;
            MatrixXd v1;
            v1.resize(JOINT_NAMES, (Tf - Tb) / deltaT);
            auto iterator = l.begin();

            //from list l to matrix of solutions
            for(auto i:l){
                v1.col(k) = i;
                k++;
            }

            //send values to joints
            cout << "Starting motion\n";
            int i=0;
            while(ros::ok()){
                //cout << endl << v1.col(i) << endl;
                send_des_jstate(joint_pub, gripper_sim, v1.col(i++));
                ros::spinOnce();
                loop_rate.sleep();

                if(i >= (Tf - Tb) / deltaT) break;
            }

            //printing list l solutions
            //to print the Th values of matlab
            ///*
            ros::spinOnce();
            k = 1;
            for(auto i:l){
                VectorXd v1(6);
                v1 = i;
                cout<<k<<":      ";
                stampaVector(v1);
                k++;
            }
            //*/

            //print final q state
            ros::Duration(1).sleep();
            ros::spinOnce();
            std::cout << "final q [ ";
            for(int i=0; i<q.size(); ++i){
                std::cout << q[i] << " ";
            }
            std::cout << "]" << std::endl;
        }

        Vector3d rotationMatrixToEulerAngles(const Matrix3d& R)
        {
            Quaterniond quat(R);
            quat.normalize();
            Vector3d euler = quat.toRotationMatrix().eulerAngles(0,1,2);
            for(int i=0; i<euler.size(); ++i){
                if (-1e-10 < euler(i) && euler(i) < 1e-10) {
                    euler(i) =  M_PI * copysign(1.0, euler(i));
                }
            }

            return euler;
        }

        Matrix3d eul2rotm(const Vector3d& euler){

            AngleAxisd rollAngle(euler(0), Vector3d::UnitX());
            AngleAxisd pitchAngle(euler(1), Vector3d::UnitY());
            AngleAxisd yawAngle(euler(2), Vector3d::UnitZ());
                    
            Quaterniond q =  rollAngle * pitchAngle * yawAngle; 
            q.normalize();
            
            Matrix3d R60 = q.matrix();
            return R60;
        }

        Vector3d quaternionToEulerAngles(const Quaterniond& quaternion) {
            return quaternion.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
        }

        //function that transform the euler angles into a quaternion 
        Quaterniond eul2quat(const Vector3d& euler){
            AngleAxisd rollAngle(euler(0), Vector3d::UnitX());
            AngleAxisd pitchAngle(euler(1), Vector3d::UnitY());
            AngleAxisd yawAngle(euler(2), Vector3d::UnitZ());
                    
            Quaterniond q =  rollAngle * pitchAngle * yawAngle ;  
            q.normalize();

            return q;
        }

        Matrix3d quaternionToRotationMatrix(const Quaterniond& quaternion) {
            return quaternion.normalized().toRotationMatrix();
        }

        Quaterniond rotationMatrixToQuaternion(const Matrix3d& rotationMatrix) {
            Quaterniond q(rotationMatrix);
            q.normalize();
            return q;
        }

        void ur5Direct(Vector3d &xe, Matrix3d &Re,const VectorXd q_des){
            Matrix4d t10 = getRotationMatrix(q_des(0), ALPHA(0), D(0), A(0));
            Matrix4d t21 = getRotationMatrix(q_des(1), ALPHA(1), D(1), A(1));
            Matrix4d t32 = getRotationMatrix(q_des(2), ALPHA(2), D(2), A(2));
            Matrix4d t43 = getRotationMatrix(q_des(3), ALPHA(3), D(3), A(3));
            Matrix4d t54 = getRotationMatrix(q_des(4), ALPHA(4), D(4), A(4));
            Matrix4d t65 = getRotationMatrix(q_des(5), ALPHA(5), D(5), A(5));

            Matrix4d t60 = t10*t21*t32*t43*t54*t65;

            xe = t60.block(0, 3, 3, 1);
            Re = t60.block(0, 0, 3, 3);
        }

        MatrixXd ur5Jac(VectorXd Th){
            MatrixXd J;
            J.resize(6,6);

            J.col(0)<< 
                D(4)*(cos(Th(0))*cos(Th(4)) + cos(Th(1) + Th(2) + Th(3))*sin(Th(0))*sin(Th(4))) + D(3)*cos(Th(0)) - A(1)*cos(Th(1))*sin(Th(0)) - D(4)*sin(Th(1) + Th(2) + Th(3))*sin(Th(0)) - A(2)*cos(Th(1))*cos(Th(2))*sin(Th(0)) + A(2)*sin(Th(0))*sin(Th(1))*sin(Th(2)),
                D(4)*(cos(Th(4))*sin(Th(0)) - cos(Th(1) + Th(2) + Th(3))*cos(Th(0))*sin(Th(4))) + D(3)*sin(Th(0)) + A(1)*cos(Th(0))*cos(Th(1)) + D(4)*sin(Th(1) + Th(2) + Th(3))*cos(Th(0)) + A(2)*cos(Th(0))*cos(Th(1))*cos(Th(2)) - A(2)*cos(Th(0))*sin(Th(1))*sin(Th(2)),
                0,
                0,
                0,
                1;

            J.col(1)<< 
                -cos(Th(0))*(A(2)*sin(Th(1) + Th(2)) + A(1)*sin(Th(1)) + D(4)*(sin(Th(1) + Th(2))*sin(Th(3)) - cos(Th(1) + Th(2))*cos(Th(3))) - D(4)*sin(Th(4))*(cos(Th(1) + Th(2))*sin(Th(3)) + sin(Th(1) + Th(2))*cos(Th(3)))),
                -sin(Th(0))*(A(2)*sin(Th(1) + Th(2)) + A(1)*sin(Th(1)) + D(4)*(sin(Th(1) + Th(2))*sin(Th(3)) - cos(Th(1) + Th(2))*cos(Th(3))) - D(4)*sin(Th(4))*(cos(Th(1) + Th(2))*sin(Th(3)) + sin(Th(1) + Th(2))*cos(Th(3)))),
                A(2)*cos(Th(1) + Th(2)) - (D(4)*sin(Th(1) + Th(2) + Th(3) + Th(4)))/2 + A(1)*cos(Th(1)) + (D(4)*sin(Th(1) + Th(2) + Th(3) - Th(4)))/2 + D(4)*sin(Th(1) + Th(2) + Th(3)),
                sin(Th(0)),
                -cos(Th(0)),
                0;


            J.col(2)<<
                cos(Th(0))*(D(4)*cos(Th(1) + Th(2) + Th(3)) - A(2)*sin(Th(1) + Th(2)) + D(4)*sin(Th(1) + Th(2) + Th(3))*sin(Th(4))),
                sin(Th(0))*(D(4)*cos(Th(1) + Th(2) + Th(3)) - A(2)*sin(Th(1) + Th(2)) + D(4)*sin(Th(1) + Th(2) + Th(3))*sin(Th(4))),
                A(2)*cos(Th(1) + Th(2)) - (D(4)*sin(Th(1) + Th(2) + Th(3) + Th(4)))/2 + (D(4)*sin(Th(1) + Th(2) + Th(3) - Th(4)))/2 + D(4)*sin(Th(1) + Th(2) + Th(3)),
                sin(Th(0)),
                -cos(Th(0)),
                0;

            J.col(3)<<
                D(4)*cos(Th(0))*(cos(Th(1) + Th(2) + Th(3)) + sin(Th(1) + Th(2) + Th(3))*sin(Th(4))),
                D(4)*sin(Th(0))*(cos(Th(1) + Th(2) + Th(3)) + sin(Th(1) + Th(2) + Th(3))*sin(Th(4))),
                D(4)*(sin(Th(1) + Th(2) + Th(3) - Th(4))/2 + sin(Th(1) + Th(2) + Th(3)) - sin(Th(1) + Th(2) + Th(3) + Th(4))/2),
                sin(Th(0)),
                -cos(Th(0)),
                0;

            J.col(4)<<
                D(4)*cos(Th(0))*cos(Th(1))*cos(Th(4))*sin(Th(2))*sin(Th(3)) - D(4)*cos(Th(0))*cos(Th(1))*cos(Th(2))*cos(Th(3))*cos(Th(4)) - D(4)*sin(Th(0))*sin(Th(4)) + D(4)*cos(Th(0))*cos(Th(2))*cos(Th(4))*sin(Th(1))*sin(Th(3)) + D(4)*cos(Th(0))*cos(Th(3))*cos(Th(4))*sin(Th(1))*sin(Th(2)),
                D(4)*cos(Th(0))*sin(Th(4)) + D(4)*cos(Th(1))*cos(Th(4))*sin(Th(0))*sin(Th(2))*sin(Th(3)) + D(4)*cos(Th(2))*cos(Th(4))*sin(Th(0))*sin(Th(1))*sin(Th(3)) + D(4)*cos(Th(3))*cos(Th(4))*sin(Th(0))*sin(Th(1))*sin(Th(2)) - D(4)*cos(Th(1))*cos(Th(2))*cos(Th(3))*cos(Th(4))*sin(Th(0)),
                -D(4)*(sin(Th(1) + Th(2) + Th(3) - Th(4))/2 + sin(Th(1) + Th(2) + Th(3) + Th(4))/2),
                sin(Th(1) + Th(2) + Th(3))*cos(Th(0)),
                sin(Th(1) + Th(2) + Th(3))*sin(Th(0)),
                -cos(Th(1) + Th(2) + Th(3));

            J.col(5)<<
                0,
                0,
                0,
                cos(Th(4))*sin(Th(0)) - cos(Th(1) + Th(2) + Th(3))*cos(Th(0))*sin(Th(4)),
                - cos(Th(0))*cos(Th(4)) - cos(Th(1) + Th(2) + Th(3))*sin(Th(0))*sin(Th(4)),
                -sin(Th(1) + Th(2) + Th(3))*sin(Th(4));  

            return J; 
        }

        list<VectorXd> invDiffKinematicControlSimCompleteQuaternion(VectorXd TH0,VectorXd T,Matrix3d Kp,Matrix3d Kphi){
            Vector3d xe;
            Matrix3d Re;
            VectorXd qk(6);
            qk=TH0;
            double t;
            list<VectorXd> q;
            q.push_back(qk);
            
            for(int l=1;l<T.size()-1;++l){
                t=T(l);
                ur5Direct(xe,Re,qk);
                //cout << "Re invDiffKinematicComplete: " << endl << Re << endl;
                //cout << Re << endl << endl;
                Quaterniond qe = rotationMatrixToQuaternion(Re).conjugate();

                //cout << pd(t) << endl << pd(t-deltaT) << endl << endl;
                Vector3d vd=(xd(t)-xd(t-deltaT))/deltaT;
                //cout << "vd:\n";
                //stampaVector(vd);

                Quaterniond work = qd(t+deltaT)*(qd(t).conjugate());
                work.coeffs()*=(2/deltaT);
                //cout << "work:\n" << work.coeffs() << endl << endl;

                Vector3d omegad= work.vec();    
                //cout << "omegad:\n";
                //stampaVector(omegad);

                Vector3d xd_t=xd(t);
                Quaterniond qd_t=qd(t);
                VectorXd dotqk(6);
                //if(l>-1 ){cout<<t<<" vd     "<<xd_t<<endl<<endl<<"om      "<<qd_t.coeffs()<<endl<<endl;}
                
                //cout << l << endl;
                dotqk = invDiffKinematicControlCompleteQuaternion(qk,xe,xd_t,vd,omegad,qe,qd_t,Kp, Kphi); 
                //if(l>-1){cout<<dotqk<<endl<<endl;}

                VectorXd qk1(6);
                qk1= qk + dotqk*deltaT;
                //if(l==2){cout<<qk1<<endl;}
                q.push_back(qk1);
                qk=qk1;    
            }

            return q;
        }

        VectorXd invDiffKinematicControlCompleteQuaternion(VectorXd qk,Vector3d xe,Vector3d xd,Vector3d vd,Vector3d omegad,Quaterniond qe, Quaterniond qd,Matrix3d Kp,Matrix3d Kphi){
            //cout << "qk" << endl;
            //stampaVector(qk);

            MatrixXd J=ur5Jac(qk); 

            //cout<<qk<<endl<<endl<<J<<endl<< endl;
            cout << "determinant\n" << abs(J.determinant()) << endl;
            if(abs(J.determinant()) < 1e-2){
                cout<<"VICINO A SINGOLARITA"<<endl;
                //a possible way to avoid the singularities is to
                //use the damped matrix
                MatrixXd identity = MatrixXd::Identity(6,6);
                J = ur5Jac(qk).transpose()*((ur5Jac(qk) * ur5Jac(qk).transpose() + pow(DAMPING_FACTOR,2)*identity).inverse());
                //exit(1);
            }

            Quaterniond qp = qd*qe.conjugate();
            //cout << "qp: " << endl << qp.coeffs() << endl;
            //cout << "qd: " << endl << qd.coeffs() << endl;
            //cout << "qe: " << endl << qe.coeffs() << endl;
            Vector3d eo=qp.vec();
            //cout << "eo: " << endl << eo << endl;
            Vector3d part1= vd+Kp*(xd-xe);
            Vector3d part2= omegad+Kphi*eo;
            VectorXd idk(6);

            for(int i=0; i<3;i++){
                idk(i)=part1(i);
                idk(i+3)=part2(i);
            }
            
            //cout<<sp<<"     "<<xe<<endl;
            //stampaVector(xe);
            //stampaVector(part2);

            VectorXd dotQ(6);
            dotQ = (J.inverse())*idk;
            //if(sp==8){cout<<qk<<endl<<endl<<J<<endl<<endl<<idk<<endl<< endl<<qp<<endl ;}  
            //sp++;
            //cout << "dotQ:" << endl;
            //stampaVector(dotQ);
            return dotQ;
        }

        Vector3d xd(double ti){
            Vector3d xd;
            double t = ti/Tf;
            if (t > 1){
                xd = xef;
            }
                
            else{
                xd = t*xef + (1-t)*xe0;
            }
            return xd;
        }

        Vector3d phid(double ti){
            Vector3d phid;
            double t = ti/Tf;
            if (t > 1){
                phid = phief;
            }
                
            else{
                phid = t*phief + (1-t)*phie0;
            }
            return phid;
        }

        //Derived quaternion
        Quaterniond qd( double ti){
            double t=ti/Tf;
            Quaterniond qd;
            if(t>1){
                qd=qf;
            }
            else{
                qd=q0.slerp(t,qf);
            }
            return qd;
        }

        MatrixXd ur5Inverse(Vector3d &p60, Matrix3d &Re){
            MatrixXd Th(6, 8);

            Affine3d hmTransf = Affine3d::Identity();
            hmTransf.translation() = p60;
            hmTransf.linear() = Re;
            Matrix4d T60 = hmTransf.matrix();
            //std::cout << "T60\n" << T60 << std::endl;
            
            //finding th1
            Vector4d data(0, 0, -D(5), 1);
            Vector4d p50 = (T60 * data);

            double psi = atan2(p50(1), p50(0));
            double p50xy = hypot(p50(1), p50(0));

            if(p50xy < D(3)){
                MatrixXd ones(6,1);
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

            Matrix4d T10_1 = getRotationMatrix(th1_1, ALPHA(0), D(0), A(0));
            Matrix4d T10_2 = getRotationMatrix(th1_2, ALPHA(0), D(0), A(0));

            Matrix4d T16_1 = (T10_1.inverse()*T60).inverse();
            Matrix4d T16_2 = (T10_2.inverse()*T60).inverse();

            double zy_1 = T16_1(1,2);
            double zx_1 = T16_1(0,2);

            double zy_2 = T16_2(1,2);
            double zx_2 = T16_2(0,2);
            double th6_1_1, th6_1_2, th6_2_1, th6_2_2;

            if(almostZero(sin(th5_1_1)) || (almostZero(zy_1) && almostZero(zx_1))){
                std::cout << "singular configuration. Choosing arbitrary th6" << std::endl;
                th6_1_1 = 0;
            } else {
                th6_1_1 = atan2((-zy_1 / sin(th5_1_1)), (zx_1 / sin(th5_1_1)));
            }

            if(almostZero(sin(th5_1_2)) || (almostZero(zy_1) && almostZero(zx_1))){
                std::cout << "singular configuration. Choosing arbitrary th6" << std::endl;
                th6_1_2 = 0;
            } else {
                th6_1_2 = atan2((-zy_1 / sin(th5_1_2)), (zx_1 / sin(th5_1_2)));
            }

            if(almostZero(sin(th5_2_1)) || (almostZero(zy_2) && almostZero(zx_2))){
                std::cout << "singular configuration. Choosing arbitrary th6" << std::endl;
                th6_2_1 = 0;
            } else {
                th6_2_1 = atan2((-zy_2 / sin(th5_2_1)), (zx_2 / sin(th5_2_1)));
            }

            if(almostZero(sin(th5_2_2)) || (almostZero(zy_2) && almostZero(zx_2))){
                std::cout << "singular configuration. Choosing arbitrary th6" << std::endl;
                th6_2_2 = 0;
            } else {
                th6_2_2 = atan2((-zy_2 / sin(th5_2_2)), (zx_2 / sin(th5_2_2)));
            }

            Matrix4d T61_1 = T16_1.inverse();
            Matrix4d T61_2 = T16_2.inverse();

            Matrix4d T54_1_1 = getRotationMatrix(th5_1_1, ALPHA(4), D(4), A(4));
            Matrix4d T54_1_2 = getRotationMatrix(th5_1_2, ALPHA(4), D(4), A(4));
            Matrix4d T54_2_1 = getRotationMatrix(th5_2_1, ALPHA(4), D(4), A(4));
            Matrix4d T54_2_2 = getRotationMatrix(th5_2_2, ALPHA(4), D(4), A(4));

            Matrix4d T65_1_1 = getRotationMatrix(th6_1_1, ALPHA(5), D(5), A(5));
            Matrix4d T65_1_2 = getRotationMatrix(th6_1_2, ALPHA(5), D(5), A(5));
            Matrix4d T65_2_1 = getRotationMatrix(th6_2_1, ALPHA(5), D(5), A(5));
            Matrix4d T65_2_2 = getRotationMatrix(th6_2_2, ALPHA(5), D(5), A(5));

            Matrix4d T41_1_1 = T61_1 * (T54_1_1 * T65_1_1).inverse();
            Matrix4d T41_1_2 = T61_1 * (T54_1_2 * T65_1_2).inverse();
            Matrix4d T41_2_1 = T61_2 * (T54_2_1 * T65_2_1).inverse();
            Matrix4d T41_2_2 = T61_2 * (T54_2_2 * T65_2_2).inverse();

            data << 0, -D(3), 0, 1;
            Vector4d P = (T41_1_1 * data);
            Vector3d P31_1_1(P(0),P(1),P(2));

            P = (T41_1_2 * data);
            Vector3d P31_1_2 (P(0),P(1),P(2));
            
            P = (T41_2_1 * data);
            Vector3d P31_2_1 (P(0),P(1),P(2));
            
            P = (T41_2_2 * data);
            Vector3d P31_2_2 (P(0),P(1),P(2));
            
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
                
            Matrix4d T21 = getRotationMatrix(th2_1_1_1, ALPHA(1), D(1), A(1));
            Matrix4d T32 = getRotationMatrix(th3_1_1_1, ALPHA(2), D(2), A(2));
            Matrix4d T41 = T41_1_1;
            Matrix4d T43 = (T21 * T32).inverse() * T41;
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

        Matrix4d getRotationMatrix(double th, double alpha, double d, double a){
            Matrix4d rotation;
            rotation << cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha), a*cos(th),
                sin(th), cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th),
                0, sin(alpha), cos(alpha), d,
                0, 0, 0, 1;

            return rotation;                                       
        }

        MatrixXd purgeNanColumn(MatrixXd matrix){
            MatrixXd newMatrix(6, 0);
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

        void stampaVector(VectorXd v){
            for (int cb=0;cb<v.size();cb++){
                cout <<v(cb)<<"  ";
            }
            cout<<endl<<endl;
        }

        bool almostZero(double value){
            return abs(value)<1e-7;
        };

};

int main(int argc, char **argv){
    if(argc < 9) {
        std::cout << "Insert exactly [x, y, z] for position, [alpha, beta, gamma] for rotation and [left, right] for the opening of the gripper" << std::endl;
        return 1;
    }

    InverseDifferential myPub(argc, argv);

    return 0;
}