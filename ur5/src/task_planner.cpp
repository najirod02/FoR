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

using namespace std;
using namespace Eigen;

//stato release: blocco preso e devo posizionarlo nella posizione della classe del blocco
//stato take: blocco rilasciato devo andare a prenderne un altro

enum stato{
    start,                  // ordina alla parte di vision di fare uno scan del tavolo con i blocchi 
    block_take,             //stato in cui il gripper va a prendere il blocco
    high_block_take,        //stato in cui il gripper STA per prendere il blocco (high sta per una traslazione di -0.2 su z)
    high_block_release,     //stato in cui il gripper ha appena preso il blocco e si sposta verso l'alto (high sta per una traslazione di -0.2 su z)
    high_class_take,        //stato in cui trasla verso l'alto per andare a prendere il prossimo blocco 
    high_class_release,      //stato in cui il gripper ha preso il blocco e va nella posizione appena sopra il punto di rilascio
    class_release,          //punto di rilascio del blocco preso
    motion_error,           //gestisce errore nel caso ci sia un problema nel motion mette un punto intermedio nel motion 
    //waitACK,             //aspetta l'ack dal motion planner se arriva entra nel prossimo stato
    block_request,    //chiede il prossimo blocco 
    no_more_blocks     //non ci sono più blocchi termina il programma 
    

};
void gestisciStato(stato &state,ros::NodeHandle n);     //funzione macchina a stati 

//transformation from world to robot frame
const double WORLD_TO_ROBOT_X = -0.5;
const double WORLD_TO_ROBOT_Y = 0.35;
const double WORLD_TO_ROBOT_Z = 1.75;
Vector3d euler,position;       

Vector3d pos[5];
Vector3d phi[5];

const double GRIPPER_CLOSURE=-0.0639;
const double ALTEZZA_GRASPING=1.02;
//const double ALTEZZA_GRASPING=0.925+0.147;     //uncomment if bugged



static double xef[3];              //position end effector to give to the motion
static Vector3d xef_class;         //position vector used for the class


static double gripper;      //valore per l'apertura/chiusura del gripper
static double closed_gripper;

static int n_classes;           //If we wnat to specify the number of blocks/classes
static int class_of_block[5];      //6 values for 6 classes of blocks [1-6]
static int k=0;

static double phief[3];         //phi end effector to give to the motion
static Vector3d phief_class;   //phi vector used for the class



static int error;       //0:successfull motion  -- 1:unreachable block  -- 2:error in motion


const int ack=0;             //variable used for service synchronization
static int actual_ack=0;    //variable used for service synchronization

static int iteration;           //variable used for service synchronization
static int actual_iteration;       //variable used for service synchronization

static stato state;                //current state
static stato next_state;           //next state used after the wait ackstate to proceed to the next step
static int final_end=0;

int main(int argc, char** argv){
    ros::init(argc, argv, "task_planner");
    ros::NodeHandle n;
    state=start;
    n_classes=4;            //modificare questo valore in base a quanti blocchi si vuole mettere 

    iteration=1;            //used for synchronization of the service
    actual_iteration=0;

    while(state!=no_more_blocks && ros::ok()){      //cycle to change the states and different cases of motion
        gestisciStato(state,n);
        
        
    }
    
    return 0;
}
void gestisciStato(stato &state,ros::NodeHandle n){
    switch (state){
        case start:{
            //chiede alla telecamera di vedere le posizioni dei blocchi 
            //In questo caso inizializzo io i valori
            
            //INIZIALIZZO I BLOCCHI CON RISPETTIVE POSIZIONI ED ORIENTAZIONI:

            //X1-Y2-Z1
            pos[0]<< 0.85, 0.7, 0.87;
            phi[0]<< 0, 0, 0;
            //X1-Y2-Z2-TWINFILLET
            pos[1]<< 0.7, 0.7, 0.87;
            phi[1]<< 0, 0, -0.785398;
            //X1-Y3-Z2-FILLET
            pos[2]<< 0.85, 0.5, 0.87;
            phi[2]<< 0, 0, 0;
            //X1-Y4-Z1
            pos[3]<< 0.7, 0.5, 0.87;
            phi[3]<< 0, 0, -1.57;
            //X1-Y4-Z2
            pos[4]<< 0.9, 0.3, 0.87;
            phi[4]<< 0, 0, -1.57;
            
            //scegliere la classe adeguata per il blocco (mettere un numero da 1 a 5) il numero 6 è per un blocco che non funziona il grasping
            class_of_block[0]=1;
            class_of_block[1]=2;
            class_of_block[2]=3;
            class_of_block[3]=4;
            class_of_block[4]=5;
            
            ROS_INFO("passaggio a block_request");
            state=block_request;

            break;
        }

        case block_request:{
            //richiesta della posizione/ORIENTAZIONE/CLASSE di un blocco e controllo sulla variabile end 
            //in questo caso inizializzo manualmente le posizioni  di rilascio del blocco in base alla classe del blocco
            
            
            
            //Metto dentro i vettori double i valori del "k" blocco da prendere
            for(int i=0;i<3;i++){
                xef[i]=pos[k](i);
                phief[i]=phi[k](i);
            }

            switch (class_of_block[k]){        //in base alla classe del blocco imposto una posizione di rilascio e una larghezza del gripper
                case 1:{
                    //X1-Y2-Z1
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.32,ALTEZZA_GRASPING;

                    closed_gripper=GRIPPER_CLOSURE;       //impostare il valore in base alla dimensione del blocco, questa variabile verrà poi passata al motion planner per chiudere il gripper
                                       
                    break;
                }
                case 2:{
                    //X1-Y2-Z2-TWINFILLET
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.40,ALTEZZA_GRASPING;

                    closed_gripper=GRIPPER_CLOSURE; 
                   
                    break;
                }
                case 3:{
                    //X1-Y3-Z2-FILLET
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.48,ALTEZZA_GRASPING;

                    closed_gripper=GRIPPER_CLOSURE; 
                    
                    break;
                }
                case 4:{
                    //X1-Y4-Z1
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.56,ALTEZZA_GRASPING;

                    closed_gripper=GRIPPER_CLOSURE; 
                    
                    break;
                }
                case 5:{
                    //X1-Y4-Z2
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.64,ALTEZZA_GRASPING;

                    closed_gripper=GRIPPER_CLOSURE;                   

                    break;
                }
                case 6:{
                    //X2-Y2-Z2  non si può usare
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.72,ALTEZZA_GRASPING;

                    closed_gripper=GRIPPER_CLOSURE; 

                    break;
                }
            }

            if(final_end==0){   //se ci sono ancora blocchi procedi alla parte di motion
                
                state=high_block_take;
                
            }   
            else{                //nel caso in cui non ci siano più blocchi entra in questo stato 
                state=no_more_blocks;
            }
            break;
        }


        case high_block_take:{
            gripper=0.3;                //gripper aperto
            xef[2]+=0.3;                //sposto la z in alto in modo da evitare gli altri blocchi 
            // ros::ServiceServer service1 = n.advertiseService("send_pose_to_motion_planner", sendPoseToMotionPlanner);   
            // while(actual_iteration!=iteration && ros::ok()){        //fino a quando non mi arriva una service request continuo a mettere una service response
                    
            //     ros::spinOnce();
            // }
            
            // state=waitACK;           //entra nello stato di wait ack fino a quando il node motion non finisce
            // next_state=block_take;  //mette quale sarà lo stato futuro nella motion
            // actual_iteration++;     //serve per sync

           

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{                                        //gestione dell'errore (spiegazione ad inizio file sui vari valori)
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        state=block_take;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        state=block_request;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        next_state=block_take;
                        state=motion_error;
                        break;

                    }
                }         

            break;

        }


        case block_take:{           //gripper aperto 
            //manda una service response con la posizione finale=posizione blocco e gripper aperto 
            
            
            xef[2]=ALTEZZA_GRASPING;
            // ros::ServiceServer service1 = n.advertiseService("send_pose_to_motion_planner", sendPoseToMotionPlanner);
            // while(actual_iteration!=iteration && ros::ok()){

            //     ros::spinOnce();
            // }

            // state=waitACK;
            // next_state=high_block_release;
            // actual_iteration++; 
            // break;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{                                        //gestione dell'errore (spiegazione ad inizio file sui vari valori)
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        state=high_block_release;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        state=block_request;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        next_state=high_block_release;
                        state=motion_error;
                        break;

                    }
                }         

            break;
        }
        
        case high_block_release:{   //gripper chiuso
            gripper=closed_gripper;
            xef[2]+=0.3;
            // ros::ServiceServer service1 = n.advertiseService("send_pose_to_motion_planner", sendPoseToMotionPlanner);
            // while(actual_iteration!=iteration && ros::ok()){
                    
            //     ros::spinOnce();
            // }
            
            // state=waitACK;
            // next_state=high_class_release;
            // actual_iteration++; 
            // break;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{                                        //gestione dell'errore (spiegazione ad inizio file sui vari valori)
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        state=high_class_release;
                        break;
                    }
                    case 1:{srv.request.ack=0;
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        state=block_request;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        next_state=high_class_release;
                        state=motion_error;
                        break;

                    }
                }         

            break;

        }

        case high_class_release:{       //gripper chiuso

            
            for(int i=0;i<3;i++){       //metto come posizione finale la posizione della corrispettiva classe
                phief[i]=phief_class(i);
                xef[i]=xef_class(i);
                
            }
            xef[2]+=0.3;                // vado alla posizione sopraelevata 
            // ros::ServiceServer service1 = n.advertiseService("send_pose_to_motion_planner", sendPoseToMotionPlanner);
            // while(actual_iteration!=iteration && ros::ok()){
                    
            //     ros::spinOnce();
            // }
            
            // state=waitACK;
            // next_state=class_release;
            // actual_iteration++; 
            // break;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{                                        //gestione dell'errore (spiegazione ad inizio file sui vari valori)
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        state=class_release;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        state=block_request;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        next_state=class_release;
                        state=motion_error;
                        break;

                    }
                }         

            break;

        }
        case class_release:{        //gripper chiuso
            xef[2]=ALTEZZA_GRASPING;

            
            // ros::ServiceServer service1 = n.advertiseService("send_pose_to_motion_planner", sendPoseToMotionPlanner);
            // while(actual_iteration!=iteration && ros::ok()){
                    
            //     ros::spinOnce();
            // }
            
            // state=waitACK;
            // next_state=high_class_take;
            // actual_iteration++; 
            // break;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{                                        //gestione dell'errore (spiegazione ad inizio file sui vari valori)
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        state=high_class_take;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        state=block_request;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        next_state=high_class_take;
                        state=motion_error;
                        break;

                    }
                }         

            break;
        }

        case high_class_take:{      //gripper aperto
            gripper=0.3;
            xef[2]+=0.3;

            if(k++ == n_classes-1) final_end=1;        //nel caso in cui sia arrivato all'ultimo blocco faccio finire sia il task planner che il motion planner
            // ros::ServiceServer service1 = n.advertiseService("send_pose_to_motion_planner", sendPoseToMotionPlanner);
            // while(actual_iteration!=iteration && ros::ok()){
                    
            //     ros::spinOnce();
            // }
            
            // state=waitACK;
            // next_state=block_request;
            // actual_iteration++; 
            // break;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{                                        //gestione dell'errore (spiegazione ad inizio file sui vari valori)
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        if(final_end!=1)state=block_request;
                        else state=no_more_blocks;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        state=block_request;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        if(final_end!=1)next_state=block_request;
                        else next_state=no_more_blocks;
                        state=motion_error;
                        break;

                    }
                }         

            break;
        }
        
        case motion_error:{
            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;
            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=0;
            srv.request.xef2=0.38;
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;

            
            
                
            while(!service.call(srv)); 
            error=srv.response.error;
            if(error==2)exit (1);
            state=next_state;
             
            break;
        }
        
        case no_more_blocks:
            {//stato fantoccio in quanto appena arriva a questo stato si esce dal ciclo while e non viene nemmeno "invocato" 
                  
                break;
            }
    }
}