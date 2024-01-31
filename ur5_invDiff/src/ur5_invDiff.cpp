#include "invDiff.h"

int main(int argc, char **argv){
    if(argc < 9) {
        std::cout << "Insert [x, y, z] for position in world frame, [alpha, beta, gamma] for rotation in rads and [left, right] for the opening of the gripper" << std::endl;
        return 1;
    }

    InverseDifferential myPub(argc, argv);

    return 0;
}