#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointClear.h>
#include <sensor_msgs/Imu.h>
#include <list>
#include <chrono>
#include <cmath>
#include <fstream>
#include <time.h>


#include <vector>


//#include "Iir.h"

using namespace std; 


class optiTrack_tf 
{
    public: 

        //constructor
        optiTrack_tf(const ros::NodeHandle& nodeHandle); 

   
    
    private: 

        //node handler
        ros::NodeHandle nh; 

        //logfile stream
        std::ofstream logFile;

        //create ros sub object for pose data
        ros::Subscriber pose_sub; 

        //define publishers
        ros::Publisher mocap_pub;
        

        //callback node
        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); 

        //current x,y,z
        double mocap_x; 
        double mocap_y; 
        double mocap_z; 

        //orientation x,y,z,w
        double mocap_orientation_x; 
        double mocap_orientation_y; 
        double mocap_orientation_z; 
        double mocap_orientation_w; 
       
        





};