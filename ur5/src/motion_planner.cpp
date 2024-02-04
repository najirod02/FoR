#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <list>
#include "ur5/ServiceMessage.h"    
#include "ur5/ur5_invDiff_library.h"

using namespace std;

int main(int argc,char** argv){
    InverseDifferential(argc,argv);
    return 0;
}