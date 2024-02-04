#include "ur5/ur5_task_library.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "task_planner");
    ros::NodeHandle n;

    //set initial state and number of blocks to be moved
    state=start;
    n_classes=5;

    iteration=1;            //used for synchronization of the service
    actual_iteration=0;

    while(state!=no_more_blocks && ros::ok()){
        gestisciStato(state,n); 
    }
    
    return 0;
}