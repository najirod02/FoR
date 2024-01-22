#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <list>

using namespace std;
using namespace Eigen;

const static double Tf=10.0;
const static double Tb=0;
const static double deltaT=0.1;

static int sp=0;

const static int JOINT_NAMES = 6;
const double SCALAR_FACTOR = 10.0;
const double DAMPING_FACTOR = 1e-6;
const double K0 = 100.0;
static Quaterniond q0,qf;
static Vector3d xe0,xef,phie0,phief;
static VectorXd A(6),D(6),ALPHA(6);
static Matrix3d K;


bool almostZero(double value);
Vector3d xd(double ti);
Quaterniond qd( double ti);
Vector3d phid(double ti);
Matrix4d getRotationMatrix(double th, double alpha, double d, double a);
MatrixXd ur5Inverse(Vector3d &p60, Matrix3d &Re);
MatrixXd purgeNanColumn(MatrixXd matrix);
void ur5Direct(Vector3d &xe, Matrix3d &Re,VectorXd q_des);
VectorXd invDiffKinematicControlCompleteQuaternion(VectorXd qk,Vector3d xe,Vector3d xd,Vector3d vd,Vector3d omegad,Quaterniond qe, Quaterniond qd,Matrix3d Kp,Matrix3d Kphi);
list<VectorXd> invDiffKinematicControlSimCompleteQuaternion(VectorXd TH0,VectorXd T,Matrix3d Kp,Matrix3d Kphi);
void stampaVector(VectorXd v);
VectorXd qdot(VectorXd q);
MatrixXd ur5Jac(VectorXd Th);
MatrixXd myJac(VectorXd v);
// Vector3d rotationMatrixToEulerAngles2(Matrix3d &R);
// Matrix3d eul2rotm(Vector3d euler);
// Quaterniond eul2quat(Vector3d euler);


//  CONVERSIONI
Vector3d rotationMatrixToEulerAngles(const Matrix3d& rotationMatrix) ;
Matrix3d eulerAnglesToRotationMatrix(const Vector3d& euler) ;
Vector3d quaternionToEulerAngles(const Quaterniond& quaternion); 
Quaterniond eulerAnglesToQuaternion(const Vector3d& euler) ;
Matrix3d quaternionToRotationMatrix(const Quaterniond& quaternion);
Quaterniond rotationMatrixToQuaternion(const Matrix3d& rotationMatrix);  

        
int main(){
    
    A<<0, -0.425, -0.3922, 0, 0, 0;
    D<<0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    ALPHA<<M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0;
    A *= SCALAR_FACTOR;
    D *= SCALAR_FACTOR;
    K = MatrixXd::Identity(3,3);

    VectorXd qstart (6),qfin(6);
    //xe0<< 0.1518 ,-0.1908 ,0.4550;
    //xe0*=SCALAR_FACTOR;
    qstart<< -0.320003, -0.780011, -2.56108, -1.63001, -1.57001, 3.49001;
    
    xef <<0.1816 ,-0.2007 ,0.4837;
    xef*=SCALAR_FACTOR;
    qfin<< 1.7913, -2.1503, 2.4990, -1.9195, 1.5708, -0.2205;
    
    //initial and final angles are 0, 0, 0
    Matrix3d Refin;
    ur5Direct(xef,Refin,qfin);
    
    //cout<<xef.transpose()<<endl << rotationMatrixToEulerAngles(Refin).transpose()<<endl;
    // return 0;
    //0.164246 -0.183591 0.460504 1.62536 2.98626 2.97065;
    Matrix3d Re0;
    // Quaterniond qpr1=Quaterniond(Refin).conjugate();
    // Re0=qpr1.toRotationMatrix();
    // cout<<Re0.eulerAngles(0,1,2)<<endl<<endl<< Refin.eulerAngles(0,1,2)<<endl;

    // return 0;
    
    //NOTE: SCALAR_FACTOR is not needed after ur5Direct because the function itself uses it
    ur5Direct(xe0,Re0,qstart);
    //cout << "xe0\n" << xe0 << endl;
    cout << "qstart\n" << qstart << endl;
    cout << "Re0\n" << Re0 << endl;

    phie0 = rotationMatrixToEulerAngles(Re0);
    phief = rotationMatrixToEulerAngles(Refin);

    cout << "xe0\n" << xe0.transpose() << endl;
    cout << "angles phie0\n";
    stampaVector(phie0);

    //cout << "xef\n" << xef.transpose() << endl;
    cout << endl << "angles phief\n";
    stampaVector(phief);
    
    //stampaVector(qstart); 

    //Matrix3d Re1=eulerAnglesToRotationMatrix(phie0);
    //cout<<Re0<<endl<<Re1<<endl;
    //cout<<endl<< ur5Inverse(xe0,Re0).transpose()<<endl;
    //return 0;
    // Quaterniond q2=eulerAnglesToQuaternion(phie0);
    // Vector3d v2=quaternionToEulerAngles(q2);
    // cout<<v2.transpose()<<endl;
    // stampaVector(v2);
    // stampaVector(phie0);
    // return 0;


    //phief << M_PI/4, M_PI/4, M_PI/4; //Safer orientation
    //phief << M_PI/3, M_PI/2-0.1, M_PI/3; //Critical orientation
    //phief << -M_PI/2+0.1, M_PI/3, 2*M_PI/3; //Safer orientation
    //phief << M_PI/4, M_PI/4 , M_PI/4; //Safer orientation
    
    q0 = eulerAnglesToQuaternion(phie0); 
    qf = eulerAnglesToQuaternion(phief); 
    // cout<<Re0<<endl<<quaternionToRotationMatrix(q0)<<endl<<Refin<<endl<<quaternionToRotationMatrix(qf)<<endl;
    cout <<q0.coeffs()<< endl <<qf.coeffs()<<endl;
    // return 0;
    
    VectorXd T;
    double L=(Tf-Tb)/deltaT;
    T.resize(L+1);
    double time=0;

    for(int ti=0;ti<=L;ti++){
        T(ti)=time;
        
        //cout<<qd(time)<<endl;
        time +=0.1;
    } 
    
    //cout<<qf<<endl;
    //Quaterniond qp(eul2rotm(phief));
    //cout<<qf<<endl<<q_prova<<endl;
    //cout<<qf<<endl<<q_prova<<endl;
    
    //cout<<phid(0)<<endl<< phie0<<endl;
   // Vector3d xd0 = xd(0);
    //Matrix3d R0=eul2rotm(phid(0));
    //Matrix3d R0=eulerAnglesToRotationMatrix(phid(0));
    //cout<<R0<<endl<< Re0<<endl;
    
  
    //Matrix3d Rf=eul2rotm(phief);

//     Matrix3d Rf=eulerAnglesToRotationMatrix(phief);
//     MatrixXd THf = ur5Inverse(xef, Rf );
//    // cout<<TH0<<endl;
   
//     MatrixXd TH0 = ur5Inverse(xd0, R0 );
   // cout<<TH0<<endl;
   
    

    // MatrixXd M = purgeNanColumn(TH0);
    // VectorXd th0(6);
    // th0=M.col(0);                      

    Matrix3d Kp = 10*MatrixXd::Identity(3,3);
    Matrix3d Kq = -10*MatrixXd::Identity(3,3);

    // for(int ki=0;ki<103;ki++){
    //     double ki_d=(double)ki/10;
    //     cout<<ki_d <<":  "<<qd(ki_d)<<endl;
    // }
    // cout<<qf<<endl;
    
    //cout<<Kp<<endl<< Kq<<endl;
    list <VectorXd> l;
    
    l= invDiffKinematicControlSimCompleteQuaternion(qstart,T, Kp, Kq); 
    MatrixXd v1;
    v1.resize(6,100);
    int k =0;
    for(auto si:l){
        
        cout<<(k+1)<<":     ";
        
        v1.col(k)=si;
        stampaVector(si);
        //cout<<v1.col(k)<<"      ";
        k++;
    }
    Vector3d xef1;
    Matrix3d Ref1;
    cout<<v1.col(99).transpose()<<endl <<qfin.transpose()<<endl;

    return 0;
}

void ur5Direct(Vector3d &xe, Matrix3d &Re,VectorXd q_des){
    
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
    list <VectorXd> q;
    q.push_back(qk);
    
    for(int l=1;l<T.size()-1;l++){                                     
        t=T(l);
        ur5Direct(xe,Re,qk);
        
        Quaterniond qe=rotationMatrixToQuaternion(Re);
        //cout<<Re<<endl<<endl;                                         

        Vector3d vd=(xd(t)-xd(t-deltaT))/deltaT;
        //cout<<vd.transpose()<<endl;

        Quaterniond work = qd(t+deltaT)*(qd(t).conjugate());
        work.coeffs()*=(2/deltaT);
        //cout<<work.norm()<<endl;
        //work.normalize();
        Vector3d omegad= work.vec();            
        //cout<<omegad.transpose()<<endl;
        Vector3d xd_t=xd(t);
        Quaterniond qd_t=qd(t);
        VectorXd dotqk(6);
        //if(l>-1 ){cout<<t<<" vd     "<<  xd_t << endl << endl << "om      "<< qd_t.coeffs() << endl << endl;}
        
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
    MatrixXd J=ur5Jac(qk);

    if(abs(J.determinant())<1e-3){ 
        cout<<"VICINO A SINGOLARITA"<<endl;
        //use dumped pseudo inverse matrix
        MatrixXd identity = MatrixXd::Identity(6,6);
        J = ur5Jac(qk).transpose()*((ur5Jac(qk) * ur5Jac(qk).transpose() + DAMPING_FACTOR*identity).inverse());
        //exit(1);
    }
        
    Quaterniond qp = qd*qe.conjugate();
    
    Vector3d eo=qp.vec();
    //cout<<eo.transpose()<<endl;                                             
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
    //J = J.block(0,0,3,6);
    //dotQ = J.completeOrthogonalDecomposition().pseudoInverse() * (vd + K*(xd-xe)) + (MatrixXd::Identity(6,6)-
    //J.completeOrthogonalDecomposition().pseudoInverse()*J) * qdot(qk).transpose();
    dotQ = (J.inverse())*idk;
    //if(sp==8){cout<<qk<<endl<<endl<<J<<endl<<endl<<idk<<endl<< endl<<qp<<endl ;}  
    //sp++;
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

VectorXd qdot(VectorXd q){
    VectorXd qd0;

    qd0 = -K0/6*(q/(2*M_PI));

    return qd0;
}

MatrixXd ur5Inverse(Vector3d &p60, Matrix3d &Re){
            MatrixXd Th(6, 8);
            
           /* // from euler angles to rotation matrix R60                   //We should use the Re not R60 
            AngleAxisd rollAngle(euler(0), Vector3d::UnitX());
            AngleAxisd pitchAngle(euler(1), Vector3d::UnitY());
            AngleAxisd yawAngle(euler(2), Vector3d::UnitZ());
            
            Quaterniond q = yawAngle * pitchAngle * rollAngle;
            R60 = q.matrix();
            //cout << "R60\n " << R60 << endl;*/

            Affine3d hmTransf = Affine3d::Identity();
            hmTransf.translation() = p60;
            hmTransf.linear() = Re;
            Matrix4d T60 = hmTransf.matrix();
            cout << "T60\n" << T60 << endl<<p60.transpose()<<endl;
            
            //finding th1
            Vector4d data(0, 0, -D(5), 1);
            Vector4d p50 = (T60 * data);

            double psi = atan2(p50(1), p50(0));
            double p50xy = hypot(p50(1), p50(0));
            
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

            double p61z_1 = p60(0) * sin(th1_1) - p60(1) * cos(th1_1);
            double p61z_2 = p60(0) * sin(th1_2) - p60(1) * cos(th1_2);
            //cout<<phi1_1<<endl<<phi1_2 <<endl<<th1_1<<endl<<th1_2<<endl << p61z_1<<endl<<p61z_2<<endl;
            double th5_1_1 = acos((p61z_1 - D(3)) / D(5));
            double th5_1_2 = -acos((p61z_1 - D(3)) / D(5));
            double th5_2_1 = acos((p61z_2 - D(3)) / D(5));
            double th5_2_2 = -acos((p61z_2 - D(3)) / D(5));
    //cout<<th5_1_1<<endl<<th5_1_2<<endl<<th5_2_1<<endl<<th5_2_2<<endl;
            Matrix4d T10_1 = getRotationMatrix(th1_1, ALPHA(0), D(0), A(0));
            Matrix4d T10_2 = getRotationMatrix(th1_2, ALPHA(0), D(0), A(0));

            Matrix4d T16_1 = (T10_1.inverse()*T60).inverse();
            Matrix4d T16_2 = (T10_2.inverse()*T60).inverse();

            //cout<<T16_1<<endl <<endl<<T16_2<<endl <<endl <<T10_1<<endl <<endl<<T10_2<<endl <<endl;
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
//
            Matrix4d T41_1_1 = T61_1 * (T54_1_1 * T65_1_1).inverse();
            Matrix4d T41_1_2 = T61_1 * (T54_1_2 * T65_1_2).inverse();
            Matrix4d T41_2_1 = T61_2 * (T54_2_1 * T65_2_1).inverse();
            Matrix4d T41_2_2 = T61_2 * (T54_2_2 * T65_2_2).inverse();
            //
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

//
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
}

MatrixXd myJac(VectorXd v){
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

// Funzione per convertire una matrice di rotazione in angoli di Eulero
Vector3d rotationMatrixToEulerAngles(const Matrix3d& rotationMatrix) {
    Quaterniond quat(rotationMatrix);
    quat.normalize();
    Vector3d euler = quat.toRotationMatrix().eulerAngles(0,1,2);
    for(int i=0; i<euler.size(); ++i){
        if(almostZero(euler(i))) euler(i) = 0;
    }

    return euler;
}

// Funzione per convertire angoli di Eulero in una matrice di rotazione
Matrix3d eulerAnglesToRotationMatrix(const Vector3d& euler) {
    AngleAxisd rollAngle(euler(0), Vector3d::UnitX());
    AngleAxisd pitchAngle(euler(1), Vector3d::UnitY());
    AngleAxisd yawAngle(euler(2), Vector3d::UnitZ());
            
    Quaterniond q =  rollAngle * pitchAngle * yawAngle; 
    q.normalize();
    
    Matrix3d R60 = q.matrix();
    return R60;
}

// Funzione per convertire un quaternione in angoli di Eulero (ZYX)
Vector3d quaternionToEulerAngles(const Quaterniond& quaternion) {
    return quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
}

// Funzione per convertire angoli di Eulero in un quaternione (ZYX)
Quaterniond eulerAnglesToQuaternion(const Vector3d& euler) {
    AngleAxisd rollAngle(euler(0), Vector3d::UnitX());
    AngleAxisd pitchAngle(euler(1), Vector3d::UnitY());
    AngleAxisd yawAngle(euler(2), Vector3d::UnitZ());
            
    Quaterniond q =  rollAngle * pitchAngle * yawAngle ;  

    return q;
}

// Funzione per convertire un quaternione in una matrice di rotazione (ZYX)
Matrix3d quaternionToRotationMatrix(const Quaterniond& quaternion) {
    return quaternion.toRotationMatrix();
}

// Funzione per convertire una matrice di rotazione in un quaternione (ZYX)
Quaterniond rotationMatrixToQuaternion(const Matrix3d& rotationMatrix) {
    Quaterniond quat(rotationMatrix);
    return quat;
}