#include "ur5/ur5_task_library.h"

//FIXME: capire perchè non vada il punto intermedio 
int main(int argc, char** argv){
    ros::init(argc, argv, "task_planner");
    ros::NodeHandle n;

    //set initial state and number of blocks to be moved
    state=start;
    n_classes=5;///< set the number of blocks that are positioned on the workspace

    iteration=1;            //used for synchronization of the service
    actual_iteration=0;

    while(next_state!=no_more_blocks && ros::ok()){
        gestisciStato(state,n); 
    }
    
    ROS_INFO("Terminating task planner");

    return 0;
}