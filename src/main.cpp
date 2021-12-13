
#include <ros/ros.h>
#include "optiTrack_tf.h"
#include <vector>
#include <mavros_msgs/ParamSet.h>
#include <iostream>






using namespace std;


int main(int argc, char** argv)
{

    ///////////////////////////////////////////////////////////

    //ROS INITIALIZATION 

    ///////////////////////////////////////////////////////////
    //INITIATE ROS NODE
    std::cout<<"starting"<<std::endl; 
    ros::init(argc, argv, "control_node"); 

    ros::NodeHandle nh; 



    
    optiTrack_tf baseOptiTrack_tf(nh); 

   
     ros::spin(); 


    return 0; 


}