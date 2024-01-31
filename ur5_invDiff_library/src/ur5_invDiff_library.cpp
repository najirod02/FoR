#include "ur5_invDiff_library/ur5_invDiff_library.h"


InverseDifferential::InverseDifferential(int argc_, char** argv_) :
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
    
    iteration=1;
    intermediate_point_trajectory=1;
    xe_intermediate<<0.1,-0.3,0.6;//in case of error use this point as a safe homing!
    xe_intermediate*=SCALAR_FACTOR;

    WORLD_TO_ROBOT << 1, 0, 0, WORLD_TO_ROBOT_X,
                        0, -1, 0, WORLD_TO_ROBOT_Y,
                        0, 0, -1, WORLD_TO_ROBOT_Z,
                        0, 0, 0, 1;

    //read the position and euler values of final pose
    xef << std::stod(argv_[1]), std::stod(argv_[2]), std::stod(argv_[3]);
    phief << std::stod(argv_[4]), std::stod(argv_[5]), std::stod(argv_[6]);
    gripper << std::stod(argv_[7]), std::stod(argv_[8]);
    
    //convert in robot frame and check if position is inside working area
    worldToRobotFrame(xef, phief);
    xef *= SCALAR_FACTOR;
    if(!checkWorkArea(xef)){cout << "The position is not inside the working area\n"; return;}

    //start talker
    talker();    
}


//ROS functions
/**
 * a simple functions that cretes the message and publish it through the 
 * topic.
 * note that the angles must be of RADIANT type !!!
 */
void InverseDifferential::send_des_jstate(ros::Publisher joint_pub, bool gripper_sim, VectorXd q_des){
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
void InverseDifferential::receive_jstate(const sensor_msgs::JointState::ConstPtr& msg){
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
void InverseDifferential::talker(){
    //AnonymousName adds a random suffix to the node name
    ros::init(argc, argv, "ur5_invDiff", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<std_msgs::Float64MultiArray>(TOPIC, 1000);
    ros::Subscriber sub = n.subscribe(TOPIC_SUB, 1000, &InverseDifferential::receive_jstate, this);
    ros::Rate loop_rate(RATE);

    ros::Duration(1).sleep();
    ros::spinOnce();//IMPORTANT!!! to make sure that the initial configuration is read from the subscriber


    //check first if the final position is possible
    Matrix3d Rf = eulerAnglesToRotationMatrix(phief);
    MatrixXd TH0 = ur5Inverse(xef, Rf);
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
    ur5Direct(xe0, Re, qstart);
    phie0 = rotationMatrixToEulerAngles(Re);
    
    //calculate quaternions
    //from the initial configuration
    q0 = rotationMatrixToQuaternion(Re);
    //from the desired final configuration
    qf = rotationMatrixToQuaternion(Rf);

    //cout << "qstart\n" << qstart << endl;
    cout << "Re0\n" << Re << endl;
    cout << "phie0\n" << phie0.transpose() << endl;
    cout << "Rf\n" << Rf << endl;
    cout << "phief\n" << phief.transpose() << endl << endl;
    cout << "q0\n" << q0.conjugate().coeffs().transpose() << endl << "qf\n" << qf.conjugate().coeffs().transpose() << endl << endl << endl;
    

    //calculate the trajectory of the robot by a function of time
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
    Matrix3d Kq = 10*MatrixXd::Identity(3,3);
    
    list <VectorXd> l;

    l = invDiffKinematicControlSimCompleteQuaternion(qstart,T, Kp, Kq); 

    MatrixXd v1;
    v1.resize(JOINT_NAMES, (Tf - Tb) / deltaT);
    int k =0;
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
    /*
    ros::spinOnce();
    k = 1;
    for(auto i:l){
        VectorXd v1(6);
        v1 = i;
        cout<<k<<":      ";
        printVector(v1);
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
    
    if(checkWorkArea(q)){cout << "The robot's configuration is inside the working area\n\n";}
    else {cout << "The robot's configuration is NOT inside the working area\n";}
}



//Convertion functions
/**
 * function that convert the euler angles in a quaternion
 * using the XYZ convention
 */
Quaterniond InverseDifferential::eulerAnglesToQuaternion(const Vector3d& euler) {
    Quaterniond quaternion;

    quaternion=eulerAnglesToRotationMatrix(euler);

    return quaternion;
}

/**
 * function that gives the euler angles XYZ from the rotation
 * matrix
 */
Vector3d InverseDifferential::rotationMatrixToEulerAngles(const Matrix3d& rotationMatrix) {
    Vector3d euler;

    euler=rotationMatrix.eulerAngles(0,1,2);
    
    for(int j=0;j<3;j++){
        angleCorrection(euler(j));
    }
    return euler;
}

/**
 * function that convert the euler angles XYZ in a quaternion
 */
Matrix3d InverseDifferential::eulerAnglesToRotationMatrix(const Vector3d& euler) {
    Matrix3d rotationMatrix;

    // Rotazione attorno all'asse x (Roll)
    Matrix3d rotationX;
    rotationX << 1, 0, 0,
                  0, cos(euler[0]), -sin(euler[0]),
                  0, sin(euler[0]), cos(euler[0]);

    // Rotazione attorno all'asse y (Pitch)
    Matrix3d rotationY;
    rotationY << cos(euler[1]), 0, sin(euler[1]),
                  0, 1, 0,
                  -sin(euler[1]), 0, cos(euler[1]);

    // Rotazione attorno all'asse z (Yaw)
    Matrix3d rotationZ;
    rotationZ << cos(euler[2]), -sin(euler[2]), 0,
                  sin(euler[2]), cos(euler[2]), 0,
                  0, 0, 1;

    // Combinazione delle rotazioni
    rotationMatrix = rotationX * rotationY * rotationZ;
    
    //Delete precision errors
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            angleCorrection(rotationMatrix(i,j));   
        }
    }


    return rotationMatrix;
}

Quaterniond InverseDifferential::rotationMatrixToQuaternion(const Matrix3d& rotationMatrix) {
    Quaterniond quaternion;

    quaternion=rotationMatrix;
    return quaternion;
}

/**
 * given the pose of a block in the world frame, return the pose of the same
 * block in the robot frame.
 * 
 * The coordinates and orientation has the same unit of gazebo (in meters and radiants)
 */
void InverseDifferential::worldToRobotFrame(Vector3d& coords, Vector3d& euler){
    Vector4d position;
    position << coords[0], coords[1], coords[2], 1;

    coords = (WORLD_TO_ROBOT*position).block(0,0,1,3);
    
    Eigen::Matrix3d R_matrix_base = eulerAnglesToRotationMatrix(euler);

    Eigen::Matrix3d R_matrix_second = WORLD_TO_ROBOT.block<3, 3>(0, 0) * R_matrix_base;

    Eigen::Vector3d rpy_angles_second_frame = rotationMatrixToEulerAngles(R_matrix_second);
    euler(0) = rpy_angles_second_frame(0) - M_PI;
    euler(1) = rpy_angles_second_frame(1);
    euler(2) = rpy_angles_second_frame(2);
}



//Inverse differential functions
/**
 * given the joints values, return the end effector position
 * and the rotation matrix
 */
void InverseDifferential::ur5Direct(Vector3d &xe, Matrix3d &Re,VectorXd q_des){
    
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

/**
 * given the joints values, returns the computet jacobian
 * for the ur5
 */
MatrixXd InverseDifferential::ur5Jac(VectorXd v){
    VectorXd q_des(6);

    q_des<<v(0), v(1) ,  v(2), v(3) ,v(4),v(5) ;

    Matrix4d t10 = getRotationMatrix(q_des(0), ALPHA(0), D(0), A(0));
    Matrix4d t21 = getRotationMatrix(q_des(1), ALPHA(1), D(1), A(1));
    Matrix4d t32 = getRotationMatrix(q_des(2), ALPHA(2), D(2), A(2));
    Matrix4d t43 = getRotationMatrix(q_des(3), ALPHA(3), D(3), A(3));
    Matrix4d t54 = getRotationMatrix(q_des(4), ALPHA(4), D(4), A(4));
    Matrix4d t65 = getRotationMatrix(q_des(5), ALPHA(5), D(5), A(5));

    Matrix4d t20 = t10*t21;
    Matrix4d t30 = t10*t21*t32;
    Matrix4d t40 = t10*t21*t32*t43;
    Matrix4d t50 = t10*t21*t32*t43*t54;
    Matrix4d t60 = t10*t21*t32*t43*t54*t65;
    
    Vector3d z0(0,0,1);
    Vector3d z1=t10.block(0,2,3,1);
    Vector3d z2=t20.block(0,2,3,1);
    Vector3d z3=t30.block(0,2,3,1);
    Vector3d z4=t40.block(0,2,3,1);
    Vector3d z5=t50.block(0,2,3,1);

    Vector3d p0(0,0,0);
    Vector3d p1=t10.block(0,3,3,1);
    Vector3d p2=t20.block(0,3,3,1);
    Vector3d p3=t30.block(0,3,3,1);
    Vector3d p4=t40.block(0,3,3,1);
    Vector3d p5=t50.block(0,3,3,1);
    Vector3d P=t60.block(0,3,3,1);

    MatrixXd J;
    J.resize(6,6);

    J.col(0)<<(z0.cross(P-p0)), z0;
    J.col(1)<<(z1.cross(P-p1)), z1;
    J.col(2)<<(z2.cross(P-p2)), z2;
    J.col(3)<<(z3.cross(P-p3)), z3;
    J.col(4)<<(z4.cross(P-p4)), z4;
    J.col(5)<<(z5.cross(P-p5)), z5;
    
    return J;
    
}

/**
 * first of the two functions that generates the trajectory for the robot
 */
list<VectorXd> InverseDifferential::invDiffKinematicControlSimCompleteQuaternion(VectorXd TH0,VectorXd T,Matrix3d Kp,Matrix3d Kphi){
    Vector3d xe;
    Matrix3d Re;
    VectorXd qk(6);
    qk=TH0;
    double t;
    list <VectorXd> q;
    q.push_back(qk);
    
    for(int l=1;l<T.size()-1;l++){                                     //STRANE DIMENSIONI DEI VETTORI IN MATLAB 
        t=T(l);
        ur5Direct(xe,Re,qk);
        
        //xe[2]> xef[2]; --> metti errore a 2
        Quaterniond qe=rotationMatrixToQuaternion(Re);
        // qe=qe.conjugate();  
        //cout<<Re<<endl<<endl;                                         

        Vector3d vd=(xd(t)-xd(t-deltaT))/deltaT;
        //cout<<vd.transpose()<<endl;

        Quaterniond work=  (qd(t+deltaT)*qd(t).conjugate());          
        work.coeffs()*=(2/ deltaT );                              
        
      
        Vector3d omegad= work.vec();                                 
       
        Vector3d xd_t=xd(t);
        Quaterniond qd_t=qd(t);
        VectorXd dotqk(6);
        //if(l>-1 ){cout<<t<<" vd     "<<xd_t<<endl<<endl<<"om      "<<qd_t<<endl<<endl;}
        
        dotqk=  invDiffKinematicControlCompleteQuaternion(qk,xe,xd_t,vd,omegad,qe,qd_t,Kp, Kphi); 
        //if(l>-1){cout<<dotqk<<endl<<endl;}
        VectorXd qk1(6);
        qk1= qk + dotqk*deltaT;
        //if(l==2){cout<<qk1<<endl;}
        q.push_back(qk1);
        qk=qk1;    
    }
    return q;
}

/**
 * second of the two functions that generates the trajectory for the robot
 */
VectorXd InverseDifferential::invDiffKinematicControlCompleteQuaternion(VectorXd qk,Vector3d xe,Vector3d xd,Vector3d vd,Vector3d omegad,Quaterniond qe, Quaterniond qd,Matrix3d Kp,Matrix3d Kphi){
    MatrixXd J=ur5Jac(qk);     
    
    Quaterniond qp = qd*qe.conjugate();
    
    
    Vector3d eo=qp.vec();
                                                
    Vector3d part1= vd+Kp*(xd-xe);                               //!!!!!!!!!!!!!!!IMPORTANTE CAMBIARE NOME ALLE VARIABILI!!!!!!!!!!!!!
    Vector3d part2= omegad+Kphi*eo;
    VectorXd idk(6);
    for(int i=0; i<3;i++){
        idk(i)=part1(i);
        idk(i+3)=part2(i);
    }
      

    VectorXd dotQ(6);
    if(abs(J.determinant())<1e-3){ 
        cout<<"VICINO A SINGOLARITA"<<endl;   
                                                                    
        MatrixXd identity = MatrixXd::Identity(6,6);
        J = J.transpose()*((J * J.transpose() + pow(DAMPING_FACTOR,2)*identity).inverse());     //damped pseudo inverse
        dotQ=J*idk;
    }
    else{
        dotQ = J.partialPivLu().solve(idk);
        //dotQ=J.inverse()*idk;
    }
    
    return dotQ;
}

/**
 * position as function of time using 
 * interpolation over 4 points
 */
Vector3d InverseDifferential::xd(double ti){
   Vector3d xd;
   //test if the polynomial interpolation is better than the linear one
    if(intermediate_point_trajectory==0){

        if(iteration==0){
            
            
            Vector3d xef_high;
            
            

            xef_high=xef;
            xef_high[2]-=0.2;

            
            double distance_p0_p1=(xe0-xef_high).norm();
            double distance_p1_p2=(xef_high-xef).norm();
            double total_distance=distance_p0_p1+distance_p1_p2;

            double t_p0=0;
            double t_p2=1;
            double t_p1=distance_p0_p1/total_distance;
        

            double t = ti/Tf;
            Matrix4d M;
            MatrixXd coeff(4,3);
            MatrixXd configurations(4,3);

            if(t>=t_p0 && t<t_p1){
                M<< 1,0,0,0,
                    1,pow(t_p1,1),pow(t_p1,2),pow(t_p1,3),
                    0,1,0,0,
                    0,1,2*pow(t_p1,1),3*pow(t_p1,2);
                
                configurations.row(0)<<xe0.transpose();
                configurations.row(1)<<xef_high.transpose();
                configurations.row(2)<<0,0,0;
                configurations.row(3)<<0,0,0;
                    
                coeff=M.partialPivLu().solve(configurations);
                for(int cont=0;cont<3;cont++){
                    xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
                }
            }else{
                M<< 1,pow(t_p1,1),pow(t_p1,2),pow(t_p1,3),
                    1,pow(t_p2,1),pow(t_p2,2),pow(t_p2,3),
                    0,1,2*pow(t_p1,1),3*pow(t_p1,2),
                    0,1,2*pow(t_p2,1),3*pow(t_p2,2);
                    
                configurations.row(0)<<xef_high.transpose();
                configurations.row(1)<<xef.transpose();
                configurations.row(2)<<0,0,0;
                configurations.row(3)<<0,0,0;
                    
                coeff=M.partialPivLu().solve(configurations);
                for(int cont=0;cont<3;cont++){
                    xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
                }

            }

        //    return xd;

        }else{
            //Vector3d xd;
            
            Vector3d xe0_high,xef_high;
            
            xe0_high=xe0;
            xe0_high[2]-=0.2;

            xef_high=xef;
            xef_high[2]-=0.2;

            double distance_p0_p1=(xe0-xe0_high).norm();
            double distance_p1_p2=(xe0_high-xef_high).norm();
            double distance_p2_p3=(xef_high-xef).norm();
            double total_distance=distance_p0_p1+distance_p1_p2+distance_p2_p3;
            

            double t_p0=0;
            double t_p3=1;
            double t_p1=distance_p0_p1/total_distance;
            double t_p2=1-t_p1;

            

            double t = ti/Tf;
            Matrix4d M;
            MatrixXd coeff(4,3);
            MatrixXd configurations(4,3);

            if(t>=t_p0 && t<t_p1){
                M<< 1,0,0,0,
                    1,pow(t_p1,1),pow(t_p1,2),pow(t_p1,3),
                    0,1,0,0,
                    0,1,2*pow(t_p1,1),3*pow(t_p1,2);
                    
                configurations.row(0)<<xe0.transpose();
                configurations.row(1)<<xe0_high.transpose();
                configurations.row(2)<<0,0,0;
                configurations.row(3)<<0,0,0;

                coeff=M.partialPivLu().solve(configurations);
                for(int cont=0;cont<3;cont++){
                    xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
                }
            }else if(t>=t_p1 && t<t_p2){
                M<< 1,pow(t_p1,1),pow(t_p1,2),pow(t_p1,3),
                    1,pow(t_p2,1),pow(t_p2,2),pow(t_p2,3),
                    0,1,2*pow(t_p1,1),3*pow(t_p1,2),
                    0,1,2*pow(t_p2,1),3*pow(t_p2,2);
                    
                configurations.row(0)<<xe0_high.transpose();
                configurations.row(1)<<xef_high.transpose();
                configurations.row(2)<<0,0,0;
                configurations.row(3)<<0,0,0;
                    
                coeff=M.partialPivLu().solve(configurations);
                for(int cont=0;cont<3;cont++){
                    xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
                }

            }
            else{
                M<< 1,pow(t_p2,1),pow(t_p2,2),pow(t_p2,3),
                    1,pow(t_p3,1),pow(t_p3,2),pow(t_p3,3),
                    0,1,2*pow(t_p2,1),3*pow(t_p2,2),
                    0,1,2*pow(t_p3,1),3*pow(t_p3,2);
                    
                configurations.row(0)<<xef_high.transpose();
                configurations.row(1)<<xef.transpose();
                configurations.row(2)<<0,0,0;
                configurations.row(3)<<0,0,0;
                    
                coeff=M.partialPivLu().solve(configurations);
                for(int cont=0;cont<3;cont++){
                    xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
                }
            }
            //return xd;

        }
    }
                //INTERMEDIATE POINT IN CASE OF ERROR IN MOTION
    else{
        //Vector3d xd;
            
        Vector3d xe0_high,xef_high;
            
        xe0_high=xe0;
        xe0_high[2]-=0.2;

        xef_high=xef;
        xef_high[2]-=0.2;

        double distance_p0_p1=(xe0-xe0_high).norm();
        double distance_p1_p2=(xe0_high-xe_intermediate).norm();
        double distance_p2_p3=(xe_intermediate-xef_high).norm();
        double distance_p3_p4=(xef_high-xef).norm();
        double total_distance=distance_p0_p1+distance_p1_p2+distance_p2_p3+distance_p3_p4;
            

        double t_p0=0;
        double t_p4=1;
        double t_p1=distance_p0_p1/total_distance;
        double t_p2=distance_p1_p2/total_distance;
        double t_p3=1-t_p1-t_p2;

            

        double t = ti/Tf;
        Matrix4d M;
        MatrixXd coeff(4,3);
        MatrixXd configurations(4,3);

        if(t>=t_p0 && t<t_p1){
            M<< 1,0,0,0,
            1,pow(t_p1,1),pow(t_p1,2),pow(t_p1,3),
            0,1,0,0,
            0,1,2*pow(t_p1,1),3*pow(t_p1,2);
                    
            configurations.row(0)<<xe0.transpose();
            configurations.row(1)<<xe0_high.transpose();
            configurations.row(2)<<0,0,0;
            configurations.row(3)<<0,0,0;
                    
            coeff=M.partialPivLu().solve(configurations);
            for(int cont=0;cont<3;cont++){
                xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
            }
        }else if(t>=t_p1 && t<t_p2){
            M<< 1,pow(t_p1,1),pow(t_p1,2),pow(t_p1,3),
                1,pow(t_p2,1),pow(t_p2,2),pow(t_p2,3),
                0,1,2*pow(t_p1,1),3*pow(t_p1,2),
                0,1,2*pow(t_p2,1),3*pow(t_p2,2);
                    
            configurations.row(0)<<xe0_high.transpose();
            configurations.row(1)<<xe_intermediate.transpose();
            configurations.row(2)<<0,0,0;
            configurations.row(3)<<0,0,0;
                    
            coeff=M.partialPivLu().solve(configurations);
            for(int cont=0;cont<3;cont++){
                xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
            }

        }
        else if(t>=t_p2 && t<t_p3){
            M<< 1,pow(t_p2,1),pow(t_p2,2),pow(t_p2,3),
                1,pow(t_p3,1),pow(t_p3,2),pow(t_p3,3),
                0,1,2*pow(t_p2,1),3*pow(t_p2,2),
                0,1,2*pow(t_p3,1),3*pow(t_p3,2);
                    
            configurations.row(0)<<xe_intermediate.transpose();
            configurations.row(1)<<xef_high.transpose();
            configurations.row(2)<<0,0,0;
            configurations.row(3)<<0,0,0;
                    
            coeff=M.partialPivLu().solve(configurations);
            for(int cont=0;cont<3;cont++){
                xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
            }
        }else{
            M<< 1,pow(t_p3,1),pow(t_p3,2),pow(t_p3,3),
                1,pow(t_p4,1),pow(t_p4,2),pow(t_p4,3),
                0,1,2*pow(t_p3,1),3*pow(t_p3,2),
                0,1,2*pow(t_p4,1),3*pow(t_p4,2);
                    
            configurations.row(0)<<xef_high.transpose();
            configurations.row(1)<<xef.transpose();
            configurations.row(2)<<0,0,0;
            configurations.row(3)<<0,0,0;
                    
            coeff=M.partialPivLu().solve(configurations);
            for(int cont=0;cont<3;cont++){
                xd(cont)=coeff(0,cont)+coeff(1,cont)*t+coeff(2,cont)*pow(t,2)+coeff(3,cont)*pow(t,3);
        }
        //return xd;

        }
    }
    return xd;
}

/**
 * implement the slerp method
 */
Quaterniond InverseDifferential::qd( double ti){
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
  
/**
 * given position and orientation, returns the possible
 * joints' configurations
 */
MatrixXd InverseDifferential::ur5Inverse(Vector3d &p60, Matrix3d &Re){
    MatrixXd Th(6, 8);
    
    Affine3d hmTransf = Affine3d::Identity();
    hmTransf.translation() = p60;
    hmTransf.linear() = Re;
    Matrix4d T60 = hmTransf.matrix();
  
            
    //finding th1
    Vector4d data(0, 0, -D(5), 1);
    Vector4d p50 = (T60 * data);

    double psi = atan2(p50(1), p50(0));
    // angleCorrection(psi);
    double p50xy = hypot(p50(1), p50(0));
                
                
    //
    if(p50xy < D(3)){
        MatrixXd ones(6,1);
        ones.setOnes();
        Th = ones * NAN;
        cout << "Position request in the unreachable cylinder" << endl;
        return Th;
    }

    double phi1_1 = acos(D(3) / p50xy);
    
    double phi1_2 = -phi1_1;
                
    double th1_1 = psi + phi1_1 + M_PI/2;
    double th1_2 = psi + phi1_2 + M_PI/2;

    double s_th1_1=sin(th1_1);
    double c_th1_1=cos(th1_1);
    double s_th1_2=sin(th1_2);
    double c_th1_2=cos(th1_2);
    

    double p61z_1 = p60(0) * s_th1_1 - p60(1) * c_th1_1;
    double p61z_2 = p60(0) * s_th1_2 - p60(1) * c_th1_2;

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
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_1_1 = 0;
    } else {
        th6_1_1 = atan2((-zy_1 / sin(th5_1_1)), (zx_1 / sin(th5_1_1)));
    }

    if(almostZero(sin(th5_1_2)) || (almostZero(zy_1) && almostZero(zx_1))){
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_1_2 = 0;
    } else {
        th6_1_2 = atan2((-zy_1 / sin(th5_1_2)), (zx_1 / sin(th5_1_2)));
    }

    if(almostZero(sin(th5_2_1)) || (almostZero(zy_2) && almostZero(zx_2))){
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_2_1 = 0;
    } else {
        th6_2_1 = atan2((-zy_2 / sin(th5_2_1)), (zx_2 / sin(th5_2_1)));
    }

    if(almostZero(sin(th5_2_2)) || (almostZero(zy_2) && almostZero(zx_2))){
        cout << "singular configuration. Choosing arbitrary th6" << endl;
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
        cout << "Point out of the work space for th3_1_1\n";
        th3_1_1_1 = NAN;
        th3_1_1_2 = NAN;
    } else {
        th3_1_1_1 = acos(C);
        th3_1_1_2 = -acos(C);
    }

    C = (pow(P31_1_2.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    if(abs(C) > 1){
        cout << "Point out of the work space for th3_1_2\n";
        th3_1_2_1 = NAN;
        th3_1_2_2 = NAN;
    } else {
        th3_1_2_1 = acos(C);
        th3_1_2_2 = -acos(C);
    }


    C = (pow(P31_2_1.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    if(abs(C) > 1){
        cout << "Point out of the work space for th3_2_1\n";
        th3_2_1_1 = NAN;
        th3_2_1_2 = NAN;
    } else {
        th3_2_1_1 = acos(C);
        th3_2_1_2 = -acos(C);
    }


    C = (pow(P31_2_2.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    if(abs(C) > 1){
        cout << "Point out of the work space for th3_2_2\n";
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

/**
 * compute the homogeneous matrix for the computation of the direct
 * kinematic
 */
Matrix4d InverseDifferential::getRotationMatrix(double th, double alpha, double d, double a){
    Matrix4d rotation;
    rotation << cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha), a*cos(th),
        sin(th), cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;
    for(int i=0;i<rotation.rows();i++){
        for(int j=0;j<rotation.cols();j++){
            angleCorrection(rotation(i,j));
        }
    }
    return rotation;                                        
}

/**
 * remove all the NON possible solutions from the inverse problem
 * checking if each columns contains at least a NaN value
 */
MatrixXd InverseDifferential::purgeNanColumn(MatrixXd matrix){
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



//Other functions
/**
 * function that controls if the end effector is inside the working area
 */
bool InverseDifferential::checkWorkArea(const Vector3d& position){
    //approximation to 3 decimal to exlude errors
    double x = floor((position[0]/SCALAR_FACTOR)*100)/100;
    double y = floor((position[1]/SCALAR_FACTOR)*100)/100;
    double z = floor((position[2]/SCALAR_FACTOR)*100)/100;

    if(MIN_X <= x && x <= MAX_X){
        if(MIN_Y <= y && y <= MAX_Y){
            if(MIN_Z <= z && z <= MAX_Z){
                return true;
            }
        }
    }

    return false;
}

/**
 * function that controls if the given joint configuration, the end
 * effector in inside the working area
 */
bool InverseDifferential::checkWorkArea(const VectorXd& joints){
    //use direct kinematic to find end effector position
    Vector3d xe;
    Matrix3d Re;

    ur5Direct(xe, Re, joints);

    return checkWorkArea(xe);
}

/**
 * print the values of a generic vector
 */
void InverseDifferential::printVector(VectorXd v){
    for (int cb=0;cb<v.size();cb++){
        cout <<v(cb)<<"  ";
    }
    cout<<endl<<endl;
}

/**
 * function that checks if a value is close to zero
 */
bool InverseDifferential::almostZero(double value){
    return abs(value)<ALMOST_ZERO;
};

/**
 * function that checks if the angle is close to zero.
 * if so, set the value to zero
 */
void InverseDifferential::angleCorrection(double & angle){
    if(almostZero(angle*pow(10,9))){
        angle=0;
    }
}