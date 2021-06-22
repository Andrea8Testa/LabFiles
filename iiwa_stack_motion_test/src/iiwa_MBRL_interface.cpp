
#include <iiwa_ros/iiwa_ros.h>
#include <iiwa_ros/conversions.h>
#include <ros/console.h>

#include <math.h> 
#include <random>
#include <iostream>
#include <fstream>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointPosition.h>

std::ofstream log_file_KDW;

std_msgs::Bool execute_exp;
std_msgs::Float32 component_weight;

iiwa_msgs::JointPosition iiwa_joint_position;           
iiwa_msgs::JointPosition command_joint_position;
iiwa_msgs::JointPosition msr_joint_position;

geometry_msgs::PoseStamped command_cartesian_position;
geometry_msgs::PoseStamped goal_pose;
geometry_msgs::WrenchStamped iiwa_cartesian_wrench;
geometry_msgs::WrenchStamped iiwa_old_wrench;
geometry_msgs::Transform iiwa_pose;

iiwa_ros::iiwaRos my_iiwa;




int K_uplim  = 5000;
int K_lowlim = 500;
int K_mean   = (K_uplim + K_lowlim)/2; 


float D_uplim  = 0.9;
float D_lowlim = 0.1;
float D_mean   = (D_uplim + D_lowlim)/2; 

// float Kp_uplim  = 0.1;
// float Kp_lowlim = 0.005;
// float Kp_mean   = (Kp_uplim + Kp_lowlim)/2;



std::vector<int> stiffness(3, K_mean);   // creating a vector with 6 element all initialized with K_uplim value
std::vector<float> damping(3, D_mean);

std::vector<int> temp_stiffness(3, 0);  
std::vector<float> temp_damping(3, 0);

int num_trials = 1;
int i_ = 0;
int j_ = 0;
bool impedance_mode = false;
bool change_impedance = false;

int K_z;
float D_z;
float Kp_pose;



float dt         = 0.003;
float speed      = 0.0;
float speed_lim  = 0.07; 
float speed_temp = 0.0;

std::vector<float> old_pose(3, 0); 
std::vector<float> moving_direction(3, 0);
bool moving_bool = false;
bool move_       = false;
float weight_    = 0;



float human_force     = 0;
float human_force_lim = 5; // (N)
std::vector<float> human_force_vec(3, 0);

ros::Publisher execute_exp_MBRL_pub;


///////////////////////////////////////////////////// Callback functions ////////////////////////////////////////////////////////////

void iiwa_pose_callback ( const geometry_msgs::PoseStamped::ConstPtr& msg )
{
//   ROS_INFO_STREAM("inside pose callback");
  iiwa_pose.translation.x = msg->pose.position.x;
  iiwa_pose.translation.y = msg->pose.position.y;
  iiwa_pose.translation.z = msg->pose.position.z;
  iiwa_pose.rotation.x    = msg->pose.orientation.x;
  iiwa_pose.rotation.y    = msg->pose.orientation.y;
  iiwa_pose.rotation.z    = msg->pose.orientation.z;
  iiwa_pose.rotation.w    = msg->pose.orientation.w;
  
  int iter_ = 10;
  
  // Calculating the average speed (not used)
  if (i_ < iter_)
  {
     speed_temp += sqrt( pow( (iiwa_pose.translation.x - old_pose[0]), 2) + pow( (iiwa_pose.translation.y - old_pose[1]), 2) + pow( (iiwa_pose.translation.z - old_pose[2]), 2) )/dt;
     i_ += 1;
  }
  else
  {
    speed = speed_temp/iter_;
    speed_temp = 0;
    i_= 0;
  }

 
  old_pose[0] = iiwa_pose.translation.x;
  old_pose[1] = iiwa_pose.translation.y;
  old_pose[2] = iiwa_pose.translation.z;
  
}






void iiwa_KD_callback ( const geometry_msgs::PoseStamped::ConstPtr& msg )
{

  // Getting the impedance values online
  
  if (change_impedance)
  {
    temp_stiffness[2] += msg->pose.position.x;
    temp_stiffness[1] += msg->pose.position.y;
    temp_stiffness[0] += msg->pose.position.z;
    
    temp_damping[2]   += msg->pose.orientation.x;
    temp_damping[1]   += msg->pose.orientation.y;
    temp_damping[0]   += msg->pose.orientation.z;
    j_ +=1;
    
  }


}


void iiwa_joint_position_callback ( const iiwa_msgs::JointPosition::ConstPtr& msg )
{
//   ROS_INFO_STREAM("inside jointPosition callback");
  iiwa_joint_position.position.a1 = msg->position.a1;
  iiwa_joint_position.position.a2 = msg->position.a2;
  iiwa_joint_position.position.a3 = msg->position.a3;
  iiwa_joint_position.position.a4 = msg->position.a4;
  iiwa_joint_position.position.a5 = msg->position.a5;
  iiwa_joint_position.position.a6 = msg->position.a6;
  iiwa_joint_position.position.a7 = msg->position.a7;
  

}




void iiwa_wrench_callback ( const geometry_msgs::WrenchStamped::ConstPtr& msg )
{
//   ROS_INFO_STREAM("inside wrench callback");
  iiwa_cartesian_wrench.wrench.force.x = msg->wrench.force.x;
  iiwa_cartesian_wrench.wrench.force.y = msg->wrench.force.y;
  iiwa_cartesian_wrench.wrench.force.z = msg->wrench.force.z;
  

  // wrench is in Body-Frame need to transform to Space-Frame for consistency. (To Do)
  human_force_vec[0] = msg->wrench.force.x - weight_;
  human_force_vec[1] = msg->wrench.force.y;
  human_force_vec[2] = msg->wrench.force.z;
  
  human_force = sqrt( pow(human_force_vec[0], 2) + pow(human_force_vec[1], 2) + pow(human_force_vec[2], 2) );
   
    
  
  if ( impedance_mode && ( fabs(human_force_vec[0]) >= human_force_lim || fabs(human_force_vec[1]) >= human_force_lim || fabs(human_force_vec[2]) >= human_force_lim ))
  {
    
 
    if (execute_exp.data == false) 
    {
      ROS_WARN_STREAM("Detected Human input force");
//       change_impedance = true;
//       ros::Duration(1.0).sleep();
//       change_impedance = false;
      execute_exp.data = true;  		// flag for recording the data
      execute_exp_MBRL_pub.publish(execute_exp);
      
    }
    
  }
  else if (impedance_mode && ( fabs(human_force_vec[0]) < human_force_lim && fabs(human_force_vec[1]) < human_force_lim && fabs(human_force_vec[2]) < human_force_lim ))
  {
    
    if (execute_exp.data == true)
    {
      ROS_WARN_STREAM("No Human input force");
      execute_exp.data = false;
      execute_exp_MBRL_pub.publish(execute_exp);
      
    }
    
  }
  
  
  iiwa_old_wrench.wrench.force.x = iiwa_cartesian_wrench.wrench.force.x;
  iiwa_old_wrench.wrench.force.y = iiwa_cartesian_wrench.wrench.force.y;
  iiwa_old_wrench.wrench.force.z = iiwa_cartesian_wrench.wrench.force.z;

}





//////////////////////////////////////////////////////////////////// Main Function /////////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv) 
{

  // Initialize ROS
  ros::init(argc, argv, "iiwa_MBRL_interface");
  ros::NodeHandle nh("~");
  ros::Rate loopRate(90);         // control frequency Hz
//   component_weight.data = 0.0;

  
  //// Initializing some values ////
  
  // average impedance values calculated offline using the predictions of NN models
  K_z = 691;
  D_z = 0.87;

   
  stiffness[0] = K_z;
  damping[0]   = D_z;
  
  iiwa_old_wrench.wrench.force.x = 0;
  iiwa_old_wrench.wrench.force.y = 0;
  iiwa_old_wrench.wrench.force.z = 0;

  ////////////iiwa////////////
  my_iiwa.init();

  ros::Subscriber iiwa_pose_MBRL_sub           = nh.subscribe ("/iiwa/state/CartesianPose",   1, iiwa_pose_callback);
  ros::Subscriber iiwa_wrench_MBRL_sub         = nh.subscribe ("/iiwa/state/CartesianWrench", 1, iiwa_wrench_callback);
  ros::Subscriber iiwa_joint_position_MBRL_sub = nh.subscribe ("/iiwa/state/JointPosition",   1, iiwa_joint_position_callback);
  ros::Subscriber iiwa_KD_MBRL_sub             = nh.subscribe ("/iiwa/state/KD",              1, iiwa_KD_callback);
  
  execute_exp_MBRL_pub = nh.advertise <std_msgs::Bool> ("/iiwa/execute_exp_topic",1); 
  
  log_file_KDW.open("/home/jey/iiwa_KDW_data_MBRL.txt", std::ios_base::app);
  
  execute_exp.data = false;
  execute_exp_MBRL_pub.publish(execute_exp);
  ros::spinOnce();
  
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  
  ROS_WARN_STREAM("Setting to Position Control Mode...");
  my_iiwa.getServoMotion().setPositionControlMode();
  ros::spinOnce();
  

  
  /////////////////////////////////////////Initial Joint Position////////////////////////
    
  command_joint_position.position.a1 = 3.14/180. * 0;
  command_joint_position.position.a2 = 3.14/180. * 45;
  command_joint_position.position.a3 = 3.14/180. * 0;
  command_joint_position.position.a4 = 3.14/180. * -90;
  command_joint_position.position.a5 = 3.14/180. * 0;
  command_joint_position.position.a6 = 3.14/180. * -45;
  command_joint_position.position.a7 = 3.14/180. * 0;
  
  
  /// Goal Position and Orientation 
  goal_pose.pose.position.x    = 0.64743;
  goal_pose.pose.position.y    = 0.00641;
  goal_pose.pose.position.z    = 0.7;
  goal_pose.pose.orientation.x = -0.0227191;
  goal_pose.pose.orientation.y = 0.611744;
  goal_pose.pose.orientation.z = 0.216091;
  goal_pose.pose.orientation.w = 0.76063;

  
  bool connection_flag = false;
  
  std::cout << "Initializing the robot position ..." << std::endl;
  
  while (!connection_flag){
  
    if (my_iiwa.getRobotIsConnected()) {
    
      connection_flag = true;
      
      while (!my_iiwa.getJointPosition(msr_joint_position)) {}   // ?
      
      my_iiwa.setJointPosition(command_joint_position);

      ros::Duration(5.0).sleep(); // 5 seconds
      
    }
    else
    {
    
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
      
    }
    
    
    ros::spinOnce();

  }
  
  ros::spinOnce();
  // storing the orientation of the initial pose:
  command_cartesian_position.pose.position.x    = iiwa_pose.translation.x;
  command_cartesian_position.pose.position.y    = iiwa_pose.translation.y;
  command_cartesian_position.pose.position.z    = iiwa_pose.translation.z;
  command_cartesian_position.pose.orientation.x = iiwa_pose.rotation.x ;
  command_cartesian_position.pose.orientation.y = iiwa_pose.rotation.y ;
  command_cartesian_position.pose.orientation.z = iiwa_pose.rotation.z ;
  command_cartesian_position.pose.orientation.w = iiwa_pose.rotation.w ;
  
    
  ///////////////////////////////////// Running some trials  ////////////////////////////////////
      
  for (int i=0; i<num_trials; i++)
  {

    // log_file_KDKp<< stiffness[0] << "," << stiffness[1] << "," << stiffness[2] << "," << damping[0] << "," << damping[1] << "," << damping[2] << "," << Kp_pose << std::endl;
     
    my_iiwa.getServoMotion().setCartesianImpedanceMode( iiwa_ros::CartesianQuantityFromDouble(K_uplim,K_uplim,K_uplim, 300, 300, 300), 
							iiwa_ros::CartesianQuantityFromDouble(D_uplim, D_uplim, D_uplim, D_uplim, D_uplim, D_uplim) );
    
    ROS_WARN_STREAM("Impedance Parameters Maxed!...");
    ros::Duration(2.0).sleep();
//     ROS_WARN_STREAM("Five seconds of idle...");
//     ros::Duration(5.0).sleep();
//     
//     ROS_WARN_STREAM("Estimating the weight...");
    
    
    int iter_ = 0;
//     while(iter_ < 50) 
//     {
//       weight_ += iiwa_cartesian_wrench.wrench.force.x;
//       iter_   += 1;
//     }
//     
//     weight_ = weight_/iter_;          // averaging
//     std::cout << "Estimated Weight is : " << weight_ << " (N)" << std::endl;
//     ros::spinOnce();
    
    
    
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // In case want to calculate the average impedance parameters online and the set before the movement - Also can be done offline (in this case --> K_z, D_z) ///////////
    
//     change_impedance = true;
//     
//     iter_ = 0;
//     j_ = 0;
//     
//     while (iter_ < 10 )
//     {
// //      ROS_WARN_STREAM("Waiting to change the impedance...");
//      ros::Duration(0.5).sleep();
//      ros::spinOnce();
//      iter_ +=1;
//     }
//     
//     change_impedance = false;
//     
//     stiffness[2] = temp_stiffness[2]/j_;
//     stiffness[1] = temp_stiffness[1]/j_;
//     stiffness[0] = temp_stiffness[0]/j_;
//     
//     damping[2] = temp_damping[2]/j_;
//     damping[1] = temp_damping[1]/j_;
//     damping[0] = temp_damping[0]/j_;
//     
//     temp_stiffness = std::vector<int>(3,0) ;
//     temp_damping   = std::vector<float>(3,0);
//     std::cout << "K_x: " << stiffness[2] << "  D_x: " << damping[2] << std::endl;
//     std::cout << "K_y: " << stiffness[1] << "  D_y: " << damping[1] << std::endl;
//     std::cout << "K_z: " << stiffness[0] << "  D_z: " << damping[0] << std::endl;
//     j_ = 0;
    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    my_iiwa.getServoMotion().setCartesianImpedanceMode( iiwa_ros::CartesianQuantityFromDouble(stiffness[0],stiffness[1],stiffness[2], 300, 300, 300), 
							iiwa_ros::CartesianQuantityFromDouble(damping[0], damping[1], damping[2], 0.9, 0.9, 0.9) );
    
    int x = 0;
    while (x!=1)
    {
      ROS_WARN_STREAM("Put the weight and Enter 1 to start ...");
      std::cin >> x;	
      ros::Duration(0.1).sleep();
    }

    ROS_WARN_STREAM("Seconds of idle...");
    ros::Duration(1.0).sleep();
    
    ROS_WARN_STREAM("Estimating the weight...");
    
    
    iter_ = 0;
    
    while(iter_ < 50) 
    {
      weight_ += iiwa_cartesian_wrench.wrench.force.x;
      iter_   += 1;
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
    
    weight_ = weight_/iter_;          // averaging
    std::cout << "Estimated Weight is : " << weight_ << " (N)" << std::endl;
    ros::spinOnce();
    

    
    std::cout << "K_x: " << stiffness[2] << "  D_x: " << damping[2] << std::endl;
    std::cout << "K_y: " << stiffness[1] << "  D_y: " << damping[1] << std::endl;
    std::cout << "K_z: " << stiffness[0] << "  D_z: " << damping[0] << std::endl;

    log_file_KDW << stiffness[0] << "," << damping[0] << "," << weight_ << std::endl;

    ros::Duration(1).sleep();
    ROS_WARN_STREAM("Ready for Collaboration...");
    
    float distanc_to_goal = 0.2;     //initializing
    impedance_mode   = true;
    float dx = weight_/stiffness[0];
    std::cout << "dx is: " << dx << std::endl;  
    

    
    while(distanc_to_goal >= 0.05) 
    {
      ros::spinOnce();
      
      if ( execute_exp.data == true)
      {
	command_cartesian_position.pose.position.x = iiwa_pose.translation.x;
	command_cartesian_position.pose.position.y = iiwa_pose.translation.y;
	
	// compensating for the weight
	if (fabs( human_force_vec[0]) >= human_force_lim)
	{
// 	  ROS_INFO_STREAM("Inside If_z");
  	  command_cartesian_position.pose.position.z = iiwa_pose.translation.z - dx*(human_force_vec[0]/fabs(human_force_vec[0]));
// 	  std::cout << "Command_z is : " << command_cartesian_position.pose.position.z << std::endl;
	} 

      }
      

      distanc_to_goal = fabs(iiwa_pose.translation.z - goal_pose.pose.position.z);
      my_iiwa.setCartesianPose(command_cartesian_position);
      loopRate.sleep();
      
    }
    
    
    impedance_mode   = false;
    log_file_KDW.close();
    ROS_WARN_STREAM("Current Trial is finished...");
    execute_exp.data = false;
    execute_exp_MBRL_pub.publish(execute_exp);
    ros::spinOnce();
    ros::Duration(2.0).sleep();

        
  }
  
  
  ROS_WARN_STREAM("Current Session is finished...");
  
  return 0;
  
};
