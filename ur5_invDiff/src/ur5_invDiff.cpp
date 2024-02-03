/**
 * the main function that starts the motion node
 */

#include "ur5_invDiff_library/ur5_invDiff_library.h"

int main(int argc, char **argv){
    if(argc < 9) {
        std::cout << "Insert [x, y, z] for position in world frame, [alpha, beta, gamma] for rotation in rads and [left, right] for the opening of the gripper" << std::endl;
        return 1;
    }

    //read input values and start node
    double xef[3];
    double phief[3];
    double gripper_left, gripper_right;

    for(int i=1; i<=3; ++i)
        xef[i-1] = stod(argv[i]);

    for(int i=4; i<=6; ++i)
        phief[i-4] = stod(argv[i]); 

    gripper_left = stod(argv[7]);
    gripper_right = stod(argv[7]);

    InverseDifferential myPub(argc, argv, xef, phief, gripper_left, gripper_right);

    return 0;
}