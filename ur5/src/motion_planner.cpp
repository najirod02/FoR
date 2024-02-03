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

#include "ur5_invDiff_library/ur5_invDiff_library.h"
#include "ur5/ACK.h"
#include "ur5/tp2mp.h"     

using namespace std;

static int ack=0;           //questi due valori servono per sincronizzare i service dei due nodi
static int ack2=1;

static double gripper;      //variabile che viene assegnata dal service e serve per definire l'apertura del gripper
static int error=0;         //da cambiare nel caso ci sia un errore nella posizione del blocco o nel motion

static int iteration;       //variabile che serve per sincronizzare i due nodi

static double xef[3];

static int final_end=0; //variabile per controllare se si è arrivati all'ultimo blocco

static double phief[3];

bool sendACKToTaskPlanner(ur5::ACK::Request &req,ur5::ACK::Response &res){
    //invio valori al motion planner tra cui se c'è stato un errore e l'ack per sincronizzare i due nodi
    ack2=req.ack;
    res.ack2=0;
    res.error=error;
    
    return true;
}

int main(int argc,char** argv){

    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle n;
    ros::ServiceClient service3 = n.serviceClient<ur5::tp2mp>("send_pose_to_motion_planner");
    while(final_end==0 && ros::ok()){
        error=0;
        ur5::tp2mp srv3;
       
         
        
        
        srv3.request.n_request=iteration++;
        while(!service3.call(srv3) && ros::ok());       //chiamata del service per chiedere i valori da settare


        phief[0]=srv3.response.phief1;
        phief[1]=srv3.response.phief2;
        phief[2]=srv3.response.phief3;
        xef[0]=srv3.response.xef1;
        xef[1]=srv3.response.xef2;
        xef[2]=srv3.response.xef3;
        final_end=srv3.response.end;            //variabile che serve per vedere se non ci sono più blocchi da prendere
        gripper =srv3.response.gripper;         //usare questa variabile per stabilire la stato del gripper all'inizio del codice (aperto o chiuso)
        //TODO: mettere qua  l'implementazione della parte di motion 
        
        InverseDifferential motion(argc, argv, xef, phief, gripper, gripper);
        
        //stampa di prova per vedere se funziona il nodo (si può sostituire con una stampa dei valori ottenuti a fine motion)
        /*
        cout<<"Posizione raggiunta:"<<endl;
        for(int i=0;i<3;i++){
            cout<<"   "<<xef[i]<<" "<<phief[i]<<endl;
        }
        cout<<gripper<<endl;
        */

        //settare la variabile static error per definire se c'è stato un errore durante la parte di motion (va implementato un check sulle posizioni finali attuali e nominali )
        //  error=0 MOTION SUCCESFULL
        //  error=1 OUT OF BOUND
        //  error=2 MOTION ERROR

        //mettere questa parte a fine codice per mandare un ack al task planner per passare al prossimo step del motion
        ros::ServiceServer service4 = n.advertiseService("ack", sendACKToTaskPlanner);
        while(ack2!=ack && ros::ok()){
            ros::spinOnce();
        }
        ack2=1;
        
    }
    return 0;

}

