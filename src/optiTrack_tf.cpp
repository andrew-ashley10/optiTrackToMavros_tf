
#include "optiTrack_tf.h"

using namespace std; 
using namespace std::chrono; 

optiTrack_tf::optiTrack_tf(const ros::NodeHandle& nodeHandle)
{
    //initialize nodeHandle
    this->nh = nodeHandle; 

    //start logfile
    logFile.open("logfile.csv"); 


    
    //initialize ros subscribers
    this->pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody/pose", 10, &optiTrack_tf::pose_cb, this);

    //initialize ros publishers
    ///////////////////////////
    //trying to use pose not tf
    ///////////////////////////
    this->mocap_pub = this->nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1000);




    ros::spinOnce();

}


//TODO should pub be broken out to new msg
void optiTrack_tf::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    std::cout<<"received mocap pose"<<std::endl;

    //put msg data in mocap vars.. not just doing transform here incase this needs manipulated more
    this->mocap_x = msg->pose.position.x; 
    this->mocap_y = msg->pose.position.y; 
    this->mocap_z = msg->pose.position.z; 

    this->mocap_orientation_x = msg->pose.orientation.x; 
    this->mocap_orientation_y = msg->pose.orientation.y; 
    this->mocap_orientation_z = msg->pose.orientation.z; 
    this->mocap_orientation_w = msg->pose.orientation.w; 



    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //SHOULD TIME STAMP BE CURRENT TIME OR SHOULD IT BE TIME THAT THE ORIGINAL MESSAGE WAS SENT
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    geometry_msgs::PoseStamped newPubMsg; 
    newPubMsg.header.stamp = ros::Time::now(); 

    //pose tf
    //x mocap = x mav
    //z mocap = y mav
    //y mocap = z mav
    newPubMsg.pose.position.x = mocap_x; 
    newPubMsg.pose.position.y = mocap_z; 
    newPubMsg.pose.position.z = mocap_y; 


    //orientation tf
    newPubMsg.pose.orientation.x = mocap_orientation_x;
    newPubMsg.pose.orientation.y = mocap_orientation_y;
    newPubMsg.pose.orientation.z = mocap_orientation_z;
    newPubMsg.pose.orientation.w = mocap_orientation_w;



    //publish message
    this->mocap_pub.publish(newPubMsg); 


}



// //establish connection to FCU
// bool optiTrack_tf::connect()
// {
//     std::cout<<"creating optiTrack_tf object"<<std::endl; 

//         //subscribe to state
//         this->state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &optiTrack_tf::state_cb, this);
//         this->mission_state_sub = nh.subscribe<mavros_msgs::WaypointReached>("mavros/mission/reached", 100, &optiTrack_tf::mission_state_cb, this);
//         this->imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 1000, &optiTrack_tf::imu_cb, this);


//         //the setpoint publishing rate MUST be faster than 2Hz
//         ros::Rate rate(20.0);

//         // wait for FCU connection
//         while(ros::ok() && !current_state.connected){
//             ros::spinOnce();
//             rate.sleep();
//         }

//         std::cout<<"Successfully connected to FCU"<<std::endl; 

// }

// //Change PX4 flight mode 
// optiTrack_tf::modeSetting()
// {
//     set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


//     //for now just always in offboard, can change to take a string with desired mode
//     if( current_state.mode != "OFFBOARD")
//     {
//         std::cout<<"trying to command offboard"<<std::endl; 
//         mavros_msgs::SetMode offb_set_mode;
//         offb_set_mode.request.custom_mode = "OFFBOARD";

//         //attempt to change to offboard
//         // while(current_state.mode != "OFFBOARD")
//         // {
//             if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
//                 {
//                     ROS_INFO("Offboard enabled");
//                 }
//         //}   

//     }else{
//         std::cout<<"Already in offboard mode"<<std::endl; 
//     }


//     //std::cout<<current_state.mode<<std::endl; 

// }


// //vehicle takeoff command
// optiTrack_tf::takeoff()
// {

//     ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff"); 

//     mavros_msgs::CommandTOL takeoff_cmd; 
//     takeoff_cmd.request.min_pitch = 0; 
//     takeoff_cmd.request.yaw = 0; 
//     takeoff_cmd.request.latitude = 0; 
//     takeoff_cmd.request.longitude = 0; 
//     takeoff_cmd.request.altitude = 20; 

//     if(takeoff_client.call(takeoff_cmd) &&takeoff_cmd.response.success)
//     {
//         ROS_INFO("Vehicle taking off"); 
//     }

// }

// //vehicle arm command
// optiTrack_tf::arm()
// {

//     arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//     set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    



//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     arming_client.call(arm_cmd);

//     ROS_INFO("ARMING..."); 
    

// }


// //clear previous mission, since this method also resets currentWaypointFinished, it
// //is assumed that this method only will be called at the start of a new mission 
// optiTrack_tf::clearWaypoints()
// {
//     ros::ServiceClient wp_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
//     mavros_msgs::WaypointClear wp_clear_srv;

//     //send clear command 
//     if (wp_clear_client.call(wp_clear_srv))
//     {
//         ROS_INFO("Waypoint list was cleared");
//         //set current waypoint back to 0, assuming this method is only being called at start of new mission
//         currentWaypointFinished = 0; 
//     }
//     else
//     {
//         ROS_ERROR("Waypoint list couldn't been cleared");
//     }
// }


// //send waypoints to fcu
// optiTrack_tf::sendWaypoints(mavros_msgs::WaypointPush wp_push_srv)
// {
//     wp_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
//     set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


//     mavros_msgs::SetMode auto_set_mode;
//     auto_set_mode.request.custom_mode = "AUTO.MISSION";






//      // Send WPs to Vehicle
//     if (wp_client.call(wp_push_srv)) {
//         ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
//         if (current_state.mode != "AUTO.MISSION") {
//             if( set_mode_client.call(auto_set_mode) &&
//                 auto_set_mode.response.mode_sent){
//                 ROS_INFO("AUTO.MISSION enabled");
//             }
//         }
//     }
//     else
//         ROS_ERROR("Send waypoints FAILED.");


// }


// vector<vector<double>> optiTrack_tf::plan()
// {
//     vector<vector<double>> solution; 


//     //each vector<double> in solution should be: 
//     //c,x,y,z,theta,roll,pitch
//     //where c = command type (0 for waypoint, 1 for roll command, 2 for both)

//     if(inContact){

//     }else{

//     }

//     return solution; 
// }


// bool optiTrack_tf::executeMission(mavros_msgs::WaypointPush wp_push_srv, double missionSize)
// {

//     //send arm/takeoff command
//     this->arm(); 
//     usleep(500000); 
//     this->takeoff(); 

//     //once in air send waypoint mission
//     this->sendWaypoints(wp_push_srv);

//     //monitor mission progress
// //////////////
// //TODO MAKE SURE NUMBERING IS RIGHT
// //////////////



// }







// ///////////////////////////////////////////////////
// //for cog estimation
// ///////////////////////////////////////////////////


// MatrixXd optiTrack_tf::estimate_cog(MatrixXd angular_velocity, MatrixXd angular_accel, MatrixXd linear_accel)
// {

//     //cog calculation
//     MatrixXd G1(3,3); 
//     MatrixXd G2(3,3); 

//     //////
//     //G1//
//     //////
//     //0
//     G1(0,0) = 0; 
//     //-w'z
//     G1(0,1) = -angular_accel(0,2); 
//     //w'y
//     G1(0,2) = angular_accel(0,1); 
//     //w'z
//     G1(1,0) = angular_accel(0,2); 
//     //0
//     G1(1,1) = 0; 
//     //-w'x
//     G1(1,2) = -angular_accel(0,0); 
//     //-w'y
//     G1(2,0) = -angular_accel(0,1);
//     //w'x 
//     G1(2,1) = angular_accel(0,0); 
//     //0
//     G1(2,2) = 0; 


//     //////
//     //G2//
//     //////
//     //-(wy^2+wz^2)
//     G2(0,0) = -(pow(linear_accel(1,0),2)+pow(linear_accel(2,0),2)); 
//     //wx*wy
//     G2(0,1) = linear_accel(0,0)*linear_accel(1,0); 
//     //wx*wz
//     G2(0,2) = linear_accel(0,0)*linear_accel(2,0); 
//     //wx*wy
//     G2(1,0) = linear_accel(0,0)*linear_accel(1,0); 
//     //-(wx^2+wz^2)
//     G2(1,1) = -(pow(linear_accel(0,0),2)+pow(linear_accel(2,0),2));
//     //wy*wz
//     G2(1,2) = linear_accel(1,0)*linear_accel(2,0); 
//     //wx*wz
//     G2(2,0) = linear_accel(0,0)*linear_accel(2,0); 
//     //wy*wz
//     G2(2,1) = linear_accel(1,0)*linear_accel(2,0); 
//     //-(wy^2+wz^2)
//     G2(2,2) = -(pow(linear_accel(1,0),2)+pow(linear_accel(2,0),2));

//     MatrixXd G = G1+G2; 

//     G = G.inverse(); 

//     MatrixXd cog = G*linear_accel; 
//     //MatrixXd cog = linear_accel*G;

//     //std::cout<<"\n\n"<<cog<<std::endl; 
//     //std::cout<<"i\n"<<std::endl; 
//     return cog; 

    
// }


// void optiTrack_tf::state_cb(const mavros_msgs::State::ConstPtr& msg)
// {
//         current_state = *msg;



//         // std::cout<<"State: "<<current_state.mode<<std::endl;

//     //    std::cout<<"HeartBeat:"<<std::endl; 
// }

// void optiTrack_tf::mission_state_cb(const mavros_msgs::WaypointReached::ConstPtr& msg)
// {
//     currentWaypointFinished = msg->wp_seq;       
// }

// void optiTrack_tf::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
// {

//     // //setup matricies
//     // MatrixXd angular_velocity(1,3); 
//     // MatrixXd angular_accel(1,3); 
//     // MatrixXd linear_accel(3,1);
    


//     // //get angular velocity
//     // //current_angular_velocity = msg.angular_velocity
//     // double current_veloc_x = msg->angular_velocity.x; 
//     // double current_veloc_y = msg->angular_velocity.y; 
//     // double current_veloc_z = msg->angular_velocity.z; 


//     // ///////////////////////
//     // //FILTER HERE
//     // ///////////////////////

//     // angular_velocity(0,0) = last_filtered_angl_veloc_x+0.05*(last_veloc_x-last_filtered_angl_veloc_x);
//     // angular_velocity(0,1) = last_filtered_angl_veloc_y+0.05*(last_veloc_y-last_filtered_angl_veloc_y);
//     // angular_velocity(0,2) = last_filtered_angl_veloc_z+0.05*(last_veloc_z-last_filtered_angl_veloc_z);


//     // //get linear acceleration
//     // double current_linear_accel_x = msg->linear_acceleration.x; 
//     // double current_linear_accel_y = msg->linear_acceleration.y; 
//     // double current_linear_accel_z = 9.81 - msg->linear_acceleration.z; //account for gravitational acceleration



//     // //std::cout<<msg->angular_velocity.y;
//     // //std::cout<<"    "<<angular_velocity(0,1)<<std::endl; 

//     // ///////////////////////////
//     // //FILTER HERE
//     // ///////////////////////////

//     // linear_accel(0,0) = last_filtered_linear_accel_x+0.05*(last_linear_accel_x-last_filtered_linear_accel_x); 
//     // linear_accel(1,0) = last_filtered_linear_accel_y+0.05*(last_linear_accel_y-last_filtered_linear_accel_y);
//     // linear_accel(2,0) = last_filtered_linear_accel_z+0.05*(last_linear_accel_z-last_filtered_linear_accel_z);

//     // //get current time


//     // auto current_time = msg->header.stamp.nsec;

//     // auto duration = current_time - last_time; 

//     // //std::cout<<"\n\n\nTIME ns: "<<duration<<std::endl; 


//     // //calculate angular acceleration
//     // //CALCULATING WITH UNFILTERED VALUES
//     // // angular_accel(0,0) = 1000000000*((current_veloc_x - last_veloc_x)/(duration)); 
//     // // angular_accel(0,1) = 1000000000*((current_veloc_y - last_veloc_y)/(duration)); 
//     // // angular_accel(0,2) = 1000000000*((current_veloc_z - last_veloc_z)/(duration)); 

//     // //CALCULATING WITH FILTERED VALS 
//     // angular_accel(0,0) = 1000000000*((angular_velocity(0,0) - last_filtered_angl_veloc_x)/(duration)); 
//     // angular_accel(0,1) = 1000000000*((angular_velocity(0,1) - last_filtered_angl_veloc_y)/(duration)); 
//     // angular_accel(0,2) = 1000000000*((angular_velocity(0,2) - last_filtered_angl_veloc_z)/(duration)); 


//     // //SET UNFILTERED VALS HISTORY
//     // //set current to last angular velocity
//     // //set current to last time
//     // last_veloc_x = current_veloc_x;
//     // last_veloc_y = current_veloc_y; 
//     // last_veloc_z = current_veloc_z; 
//     // last_time = current_time; 

//     // //set last to current linear accel
//     // last_linear_accel_x = current_linear_accel_x;
//     // last_linear_accel_y = current_linear_accel_y; 
//     // last_linear_accel_z = current_linear_accel_z;

//     // //SET FILTERED VALS HISTORY
//     // //set last filter values
//     // last_filtered_angl_veloc_x = angular_velocity(0,0); 
//     // last_filtered_angl_veloc_y = angular_velocity(0,1); 
//     // last_filtered_angl_veloc_z = angular_velocity(0,2);

//     // last_filtered_linear_accel_x = linear_accel(0,0);  
//     // last_filtered_linear_accel_y = linear_accel(1,0);  
//     // last_filtered_linear_accel_z = linear_accel(2,0);  


    
    
//     // //Estimate COG
//     // MatrixXd result = estimate_cog(angular_velocity,angular_accel, linear_accel);


//     // //Update tables for COG Calc per iteration, only 2000 samples right now
//     // if(itercount < 2000)
//     // {
//     //     cogEstimationsX(itercount,0) = itercount;
//     //     cogEstimationsX(itercount,1) = result(0,0);

        
//     //     // cogEstimationsY(itercount,1) = result(1,0);
//     //     // cogEstimationsZ(itercount,1) = result(2,0);
//     //     itercount += 1; 
//     // }else if(itercount == 2000)
//     // {   //at 2000 samples, LMS estimation
//     //     LMS_estimation test(cogEstimationsX);
//     //     // LMS_estimation test(cogEstimationsY);
//     //     // LMS_estimation test(cogEstimationsZ);
//     //     itercount += 1; 
//     // }else
//     // {   
//     //     itercount +=1; 
//     // }
    
//     // //Log COG Estimation Results
//     // logFile<<result(0,0)<<','<<result(1,0)<<','<<result(2,0); 
//     // logFile<<std::endl;
   


// }



// int optiTrack_tf::filterGyro()
// {


// }
