
#include <iiwa_ros/iiwa_ros.h>
#include <ros/console.h>
#include <math.h> 
#include <geometry_msgs/Transform.h>
#include <random>
#include <iostream>
#include <iiwa_ros/conversions.h>
#include <std_msgs/Bool.h>
#include <fstream>

std::ofstream log_file_KDW;

geometry_msgs::Transform iiwa_pose;
iiwa_msgs::JointPosition iiwa_joint_position;
geometry_msgs::WrenchStamped iiwa_cartesian_wrench;
geometry_msgs::WrenchStamped iiwa_delta_old_wrench;
geometry_msgs::WrenchStamped iiwa_default_wrench;
geometry_msgs::PoseStamped iiwa_cartesian_stiffness_damping_Kp;
std_msgs::Bool execute_exp;

iiwa_msgs::JointPosition command_joint_position;
geometry_msgs::PoseStamped goal_pose;
iiwa_msgs::JointPosition msr_joint_position;
geometry_msgs::PoseStamped command_cartesian_position;

int K_uplim  = 5000;
int K_lowlim = 500;
int K_mean   = (K_uplim + K_lowlim)/2; 


float D_uplim  = 0.9;
float D_lowlim = 0.1;
float D_mean   = (D_uplim + D_lowlim)/2; 

// float Kp_uplim  = 0.01;
// float Kp_lowlim = 0.001;
// float Kp_mean   = (Kp_uplim + Kp_lowlim)/2;



std::vector<int> stiffness(6, K_mean);   // creating a vector with 6 element all initialized with K_uplim value
std::vector<float> damping(6, D_mean);

int num_trials = 1;
float trial_duration = 30;    // in seconds



bool measure_default_wrench = false;

int K_z;     
float D_z; 

// float K_z     = K_mean;
// float D_z     = D_mean;
// float Kp_pose = 0.07;


float human_force     = 0;
float human_force_lim = 5; // (N)
std::vector<float> human_force_vec(3, 0);

float weight_    = 0;
ros::Publisher execute_exp_pub;
bool impedance_mode = false;
iiwa_ros::iiwaRos my_iiwa;
int i_ = 0;

///////////////////////////////////////////////////// Callback functions ////////////////////////////////////////////////////////////

void iiwa_pose_callback ( const geometry_msgs::PoseStampedConstPtr& msg )
{
  
  iiwa_pose.translation.x = msg->pose.position.x;
  iiwa_pose.translation.y = msg->pose.position.y;
  iiwa_pose.translation.z = msg->pose.position.z;
  iiwa_pose.rotation.x    = msg->pose.orientation.x;
  iiwa_pose.rotation.y    = msg->pose.orientation.y;
  iiwa_pose.rotation.z    = msg->pose.orientation.z;
  iiwa_pose.rotation.w    = msg->pose.orientation.w;

}



void iiwa_joint_position_callback ( const iiwa_msgs::JointPositionConstPtr& msg )
{
  
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
      execute_exp.data = true;
      execute_exp_pub.publish(execute_exp);
      
    }
    
  }
  else if (impedance_mode && ( fabs(human_force_vec[0]) < human_force_lim && fabs(human_force_vec[1]) < human_force_lim && fabs(human_force_vec[2]) < human_force_lim ))
  {
    
    if (execute_exp.data == true)
    {
      ROS_WARN_STREAM("No Human input force");
      execute_exp.data = false;
      execute_exp_pub.publish(execute_exp);
      
    }
    
  }
  

}





///////////////////////////////////////////////////////////////// some helper functions ///////////////////////////////////////////////////////////////

void random_impedance_generator()
{
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(-1.0, 1.001);
  
  
  
  for (int i = 0; i < 3; ++i) 
  {
      //Use dis to transform the random unsigned int generated by gen into a double in [1, 2)
      //Each call to dis(gen) generates a new random double
       stiffness[i] = dis(gen)*(K_uplim - K_mean) + K_mean;    
       damping[i]   = dis(gen)*(D_uplim - D_mean) + D_mean; 
//        Kp_pose      = dis(gen)*(Kp_uplim - Kp_mean) + Kp_mean;
  }
  
  
  iiwa_cartesian_stiffness_damping_Kp.pose.position.x = stiffness[0];
  iiwa_cartesian_stiffness_damping_Kp.pose.position.y = stiffness[1];
  iiwa_cartesian_stiffness_damping_Kp.pose.position.z = stiffness[2];
  std::cout << "Random Stiffness values are: " << stiffness[0] << ", " << stiffness[1] << ", " << stiffness[2] << std::endl;
  
  iiwa_cartesian_stiffness_damping_Kp.pose.orientation.x = damping[0];
  iiwa_cartesian_stiffness_damping_Kp.pose.orientation.y = damping[1];
  iiwa_cartesian_stiffness_damping_Kp.pose.orientation.z = damping[2];
  std::cout << "Random Damping values are: " << damping[0]<< ", " << damping[1] << ", " << damping[2] << std::endl;
  
//   iiwa_cartesian_stiffness_damping_Kp.pose.orientation.w = Kp_pose;
//   std::cout << "Random Kp_Pose value is: " << Kp_pose << std::endl;
 
}




/////////////////////////////////////////////////////////////////////////////// Main Function /////////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv) 
{

  // Initialize ROS
  ros::init(argc, argv, "iiwa_initial_random_impedance_commander");
  ros::NodeHandle nh("~");
  ros::Rate loopRate(60);         // control frequency Hz
  std::vector<float> direction_to_move;

  K_z = 2500;
  D_z = 0.5;
  
  stiffness[0] = K_z;
  damping[0] = D_z;

  ////////////iiwa////////////

  my_iiwa.init();
  
  ros::Subscriber iiwa_pose_sub           = nh.subscribe ("/iiwa/state/CartesianPose", 1, iiwa_pose_callback);
  ros::Subscriber iiwa_joint_position_sub = nh.subscribe ("/iiwa/state/JointPosition", 1, iiwa_joint_position_callback);
  
  ros::Publisher stiffness_damping_Kp_pub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/state/CartesianStiffnessDampingKp", 1);
  execute_exp_pub          = nh.advertise<std_msgs::Bool>("/iiwa/execute_exp_topic",1); 
  
  log_file_KDW.open("/home/jey/iiwa_KDW_data_random.txt", std::ios_base::app);
  execute_exp.data = false;
  execute_exp_pub.publish(execute_exp);
  
  ros::spinOnce();
  
  ros::Duration(2.0).sleep();
  
  ros::spinOnce();
  my_iiwa.getServoMotion().setPositionControlMode();
  
  ros::spinOnce();
  
  
  
  
  
  /////////////////////////////////////////Initial Joint Position////////////////////////
  //// Becareful that the robot should be in configuration that it is possible to go this joint-config wihout planning!
  command_joint_position.position.a1 = 3.14/180. * 0;
  command_joint_position.position.a2 = 3.14/180. * 45;
  command_joint_position.position.a3 = 3.14/180. * 0;
  command_joint_position.position.a4 = 3.14/180. * -90;
  command_joint_position.position.a5 = 3.14/180. * 0;
  command_joint_position.position.a6 = 3.14/180. * -45;
  command_joint_position.position.a7 = 3.14/180. * 0;
  
// 0.506967,0.272312,0.829498,-0.0227191,0.611744,0.216091,0.76063
  goal_pose.pose.position.x = 0.707;
  goal_pose.pose.position.y = 0.0;
  goal_pose.pose.position.z = 0.700;
  goal_pose.pose.orientation.x = 0;
  goal_pose.pose.orientation.y = 0.707;
  goal_pose.pose.orientation.z = 0;
  goal_pose.pose.orientation.w = 0.707;
  
//   command_joint_position.position.a1 = 3.14/180. * 0;
//   command_joint_position.position.a2 = 3.14/180. * 45;
//   command_joint_position.position.a3 = 3.14/180. * 0;
//   command_joint_position.position.a4 = 3.14/180. * -45;
//   command_joint_position.position.a5 = 3.14/180. * 0;
//   command_joint_position.position.a6 = 3.14/180. * 0;
//   command_joint_position.position.a7 = 3.14/180. * 0;
  
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
  command_cartesian_position.pose.position.x    = iiwa_pose.translation.x;
  command_cartesian_position.pose.position.y    = iiwa_pose.translation.y;
  command_cartesian_position.pose.position.z    = iiwa_pose.translation.z;
  command_cartesian_position.pose.orientation.x = iiwa_pose.rotation.x;
  command_cartesian_position.pose.orientation.y = iiwa_pose.rotation.y;
  command_cartesian_position.pose.orientation.z = iiwa_pose.rotation.z;
  command_cartesian_position.pose.orientation.w = iiwa_pose.rotation.w;
  
  ///////////////////////////////////// Running some trials to collect some data ////////////////////////////////////
      
  for (int i=0; i<num_trials; i++)
  {

    random_impedance_generator();
//     log_file_KDKp<< stiffness[0] << "," << stiffness[1] << "," << stiffness[2] << "," << damping[0] << "," << damping[1] << "," << damping[2] << ","  << std::endl;

    
    my_iiwa.getServoMotion().setCartesianImpedanceMode( iiwa_ros::CartesianQuantityFromDouble(K_uplim,K_uplim,K_uplim, 300, 300, 300), 
							iiwa_ros::CartesianQuantityFromDouble(D_uplim, D_uplim, D_uplim, D_uplim, D_uplim, D_uplim) );
    
    ROS_WARN_STREAM("Impedance Parameters Maxed!...");
    ros::Duration(2.0).sleep();
    ROS_WARN_STREAM("Five seconds of idle...");
    ros::Duration(5.0).sleep();
    
    ROS_WARN_STREAM("Estimating the weight...");
    
    
    int iter_ = 0;
    while(iter_ < 50) 
    {
      weight_ += iiwa_cartesian_wrench.wrench.force.x;
      iter_   += 1;
    }
    
    weight_ = weight_/iter_;          // averaging
    std::cout << "Estimated Weight is : " << weight_ << " (N)" << std::endl;
    log_file_KDW  << stiffness[0] << "," << damping[0] << "," << weight_ << std::endl;
    log_file_KDW.close();
    ros::spinOnce();
    
    
    ROS_WARN_STREAM("Ready for Collaboration...");
    
    float distanc_to_goal = 0.2;     //initializing
    impedance_mode   = true;
    float dx = weight_/K_z;
    std::cout << "dx is: " << dx << std::endl;  
    
    my_iiwa.getServoMotion().setCartesianImpedanceMode( iiwa_ros::CartesianQuantityFromDouble(stiffness[0], K_mean, K_mean, 300, 300, 300), 
							iiwa_ros::CartesianQuantityFromDouble(damping[0], D_mean, D_mean, D_uplim, D_uplim, D_uplim) );

    
    while(distanc_to_goal >= 0.05) 
    {
      ros::spinOnce();
      
      if ( execute_exp.data == true)
      {
	command_cartesian_position.pose.position.x = iiwa_pose.translation.x;
	command_cartesian_position.pose.position.y = iiwa_pose.translation.y;
	
	
	if (fabs( human_force_vec[0]) >= human_force_lim)
	{
// 	  ROS_INFO_STREAM("Inside If_z");
  	  command_cartesian_position.pose.position.z = iiwa_pose.translation.z - dx*(human_force_vec[0]/fabs(human_force_vec[0]));
	  std::cout << "Command_z is : " << command_cartesian_position.pose.position.z << std::endl;
	} 

      }
      

      distanc_to_goal = fabs(iiwa_pose.translation.z - goal_pose.pose.position.z);
      my_iiwa.setCartesianPose(command_cartesian_position);
      loopRate.sleep();
      
    }
    
    ROS_WARN_STREAM("Current Trial is finished...");
    impedance_mode   = false;
    execute_exp.data = false;
    execute_exp_pub.publish(execute_exp);
    ros::spinOnce();
    ros::Duration(2.0).sleep();

        
  }
  
  
  ROS_WARN_STREAM("Current Session is finished...");
  
  return 0;
  
};
