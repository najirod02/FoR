#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string.h>
#include <vector>

int print_eigen(std::string str, Eigen::MatrixXd m)
{
	// Eigen Matrices do have rule to print them with std::cout
	std::cout << str<< std::endl << m << std::endl<< std::endl;
	return 0;
}


int main()
{


//statically allocated 3x3 matrix
Eigen::Matrix3d A1;
A1 <<    1,2,3,
	4,5,6,
	7,8,9;
print_eigen("A1", A1);

//statically allocated custom size matrix
Eigen::Matrix<double,2,3> A2;
A2 << 1, 2, 4, 
      5, 6, 7;
print_eigen("A2", A2);
	 
//Dynamically allocated matrix
Eigen::MatrixXd A3;
A3.resize(2,3);
A3.row(0) << 1,2,3;
A3.row(1) << 4,5,6;
print_eigen("A3", A3);

Eigen::MatrixXd A4(3, 3);
A4 << 1,2,3,
      4,5,6, 
      7,8,9;
print_eigen("A4", A4);


//Accessing matrix:
std::cout << "The element at 3rd row and 2nd column is " << A4(2, 1) << std::endl;

A4.block(1,0,2,2) << 11, 11, 11, 11;
print_eigen("write 2x2 sublock whose upper left corner is A4(1,0) of A4 ", A4);
print_eigen("Read 2x2 sub-matrix whose upper left corner is A4(0, 0)", A4.block(0, 0, 2, 2));


Eigen::VectorXd a = A4.col(1); 
print_eigen("take the second column of A4", a);
Eigen::VectorXd b = A4.row(0); 
print_eigen("take the first row of A4", b);
Eigen::VectorXd c = b.head(2);
print_eigen("take the first two elements of first row", c);
Eigen::VectorXd d = b.tail(2);
print_eigen("take the last two elements of first row", d);

//Matrix multiplication
Eigen::MatrixXd B1 = 2.0*Eigen::MatrixXd::Identity(5, 5);
Eigen::VectorXd b1(5);
b1 << 1, 4, 6, -2, 0.4;
Eigen::VectorXd B2 = B1 * b1;
print_eigen("The multiplication of B1 * b1 is ", B2);


//Transpose and inverse:
Eigen::MatrixXd A5(3, 2);
A5 << 1, 2, 
     2, 3,
     3, 4;
std::cout << "the transpose of A5 \n"<< A5<<"\nis a 2x3 matrix\n" << A5.transpose() << std::endl<< std::endl;

Eigen::MatrixXd A6(2,2);
A6 << 1, 2, 
     2, 3;
std::cout << "the inverse of A6 \n"<< A6<<"\nis a 2x2 matrix\n" << A6.inverse() << std::endl<< std::endl;

//Dot product and cross product:
Eigen::Vector3d v(1, 2, 3);
Eigen::Vector3d w(0, 1, 2);
double vDotw = v.dot(w); // dot product of two vectors
Eigen::Vector3d vCrossw = v.cross(w); // cross product of two vectors

//useful mappings
//std vector to eigen vector
std::vector<double> std_vec_in{ 10, 20, 30 };
Eigen::Vector3d eigen_vec_out;
eigen_vec_out = Eigen::Map<Eigen::Vector3d>(std_vec_in.data(), std_vec_in.size());

//eigen vector to std vector  
Eigen::Vector3d eigen_vec_in;
std::vector<double> std_vec_out;
std_vec_out.resize(eigen_vec_in.size());
for (unsigned int i = 0; i < eigen_vec_in.size(); i++)
{
    std_vec_out[i] = eigen_vec_in(i);
}

//Quaternion initializations
Eigen::Quaterniond orient_error_quat, camera_quat, q1, q2;
camera_quat = Eigen::Quaterniond::Identity();
camera_quat.setIdentity();
Eigen::Quaterniond q(2, 0, 1, -3); // not recommended need to be normalized!

std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << std::endl << q.vec() << std::endl;
std::cout<<"The rotation matrix associated to the quaternion is: "<<camera_quat.matrix()<<std::endl;
std::cout<<"The coefficients associated to the quaternion are: "<<camera_quat.coeffs()<<std::endl;

// you can directly set the fields (e.g for quaternion multiplication)
orient_error_quat.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
orient_error_quat.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

// spherical interpolation
Eigen::Quaterniond q_intermediate, q_start, q_end;
double time = 0.3; //should be betwen 0 and 1
q_intermediate  = q_start.slerp(time,q_end);

//mapping from angle-axis to quaternion 
double angle_in_radian = M_PI / 4;
Eigen::Vector3d axis;
axis << 0,-1,0; //The axis vector must be normalized!
q = Eigen::AngleAxisd(angle_in_radian, axis);
std::cout<<"the quaternion from angle : "<< angle_in_radian <<" and axis " << axis.transpose() << "  is:   " << q.coeffs().transpose() <<std::endl<< std::endl;

//this is equivalent
double  sinA = std::sin(angle_in_radian / 2);
double cosA = std::cos(angle_in_radian / 2);
q.x() = 0 * sinA;
q.y() = -1 * sinA;
q.z() = 0 * sinA;
q.w() = cosA; 
std::cout<<"the quaternion from angle : "<< angle_in_radian <<" and axis " << axis.transpose() << "  is: " << q.coeffs().transpose() <<std::endl<< std::endl;

// convert a quaternion to a 3x3 rotation matrix w_R_1 that maps vectors from frame 1, whose orientatation is described by  quaterion q1 
Eigen::Matrix3d w_R_1 = q.toRotationMatrix(); 
print_eigen("The rotation matrix associated to the quaternion is", w_R_1);

// from euler angles (ZYX convention - subsequent rotations about moving axes) to rotation matrix w_R_b
Eigen::Matrix3d w_R_b;
w_R_b = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ());
print_eigen("The rotation matrix associated to Euler ancgles 0.1(yaw), 0.2(pitch), 0.3 (roll) is", w_R_b);


//are from Affine3d class: initialize homogeneous trasnform class
Eigen::Affine3d w_T_b = Eigen::Affine3d::Identity();
w_T_b.translation() = Eigen::Vector3d(1,2,3);
w_T_b.linear() = w_R_b;
Eigen::Matrix4d w_T_bm = w_T_b.matrix();
print_eigen("The homogeneous transform can be printed as 4x4 matrix", w_T_bm);

// //Compoisition of homogeneous tranforms
// Eigen::Affine3d w_T_1  = Eigen::Affine3d::Identity();
// Eigen::Affine3d b_T_1 = Eigen::Affine3d::Identity();
// b_T_1.translation() = Eigen::Vector3d(1,0,0);//pure translation
// b_T_1.linear() = Eigen::Matrix3d::Identity();
// w_T_1  = w_T_b*b_T_1;

// print_eigen("The composition of homogeneous transforms w_T_b and b_T_1  is", w_T_1.matrix());

return 0;
}
