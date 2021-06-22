#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iiwa_ros/iiwa_ros.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <math.h> 
#include <fstream>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iiwa_msgs/JointPositionVelocity.h>


geometry_msgs::WrenchStamped iiwa_delta_old_wrench;
bool execute_exp;

std::ofstream log_file;


void iiwa_sync_callback (const geometry_msgs::WrenchStamped::ConstPtr& wrench_, const iiwa_msgs::JointPositionVelocity::ConstPtr& joint_position_velocity_, 
			 const geometry_msgs::PoseStamped::ConstPtr& pose_)


{
  
  // calculating delta value between current cartesian forces and previous time step cartesian forces
  iiwa_delta_old_wrench.wrench.force.x = wrench_->wrench.force.x - iiwa_delta_old_wrench.wrench.force.x;
  iiwa_delta_old_wrench.wrench.force.y = wrench_->wrench.force.y - iiwa_delta_old_wrench.wrench.force.y;
  iiwa_delta_old_wrench.wrench.force.z = wrench_->wrench.force.z - iiwa_delta_old_wrench.wrench.force.z;

  // logging the data whenever there's HRC
    if (execute_exp)
    {
      
        log_file << wrench_->wrench.force.x    << "," << wrench_->wrench.force.y               << "," << wrench_->wrench.force.z               << "," << 
	 iiwa_delta_old_wrench.wrench.force.x  << "," << iiwa_delta_old_wrench.wrench.force.y  << "," << iiwa_delta_old_wrench.wrench.force.z  << "," <<                     
         joint_position_velocity_->position.a1 << "," << joint_position_velocity_->position.a2 << "," << joint_position_velocity_->position.a3 << "," << 
         joint_position_velocity_->position.a4 << "," << joint_position_velocity_->position.a5 << "," << joint_position_velocity_->position.a6 << "," <<
         joint_position_velocity_->position.a7 << "," << joint_position_velocity_->velocity.a1 << "," << joint_position_velocity_->velocity.a2 << "," <<
         joint_position_velocity_->velocity.a3 << "," << joint_position_velocity_->velocity.a4 << "," << joint_position_velocity_->velocity.a5 << "," <<
         joint_position_velocity_->velocity.a6 << "," << joint_position_velocity_->velocity.a7 << "," << pose_->pose.position.x                << "," << pose_->pose.position.y    << "," << 
         pose_->pose.position.z                << "," << pose_->pose.orientation.x             << "," << pose_->pose.orientation.y             << "," << pose_->pose.orientation.z << "," << 
         pose_->pose.orientation.w             << "," << pose_->header.stamp                   << std::endl;
          
//         ROS_WARN_STREAM("recording the data...");
    } 

  

// saving current cartesian forces to be used in next time step to calculate the difference     
  iiwa_delta_old_wrench.wrench.force.x = wrench_->wrench.force.x;   
  iiwa_delta_old_wrench.wrench.force.y = wrench_->wrench.force.y;
  iiwa_delta_old_wrench.wrench.force.z = wrench_->wrench.force.z;
  
  
 
  
}


void iiwa_bool_cb(const std_msgs::Bool::ConstPtr& msg)
{
  execute_exp = msg->data;

}






int main(int argc, char** argv)
{

  
  ros::init(argc, argv, "iiwa_data_logger");
  ros::Time::init();
  ros::NodeHandle nh;
  ros::Subscriber bool_sub;
  
  
    // saving the data to a .csv file
  log_file.open("/home/jey/iiwa_training_data.txt", std::ios_base::app);
  
  iiwa_delta_old_wrench.wrench.force.x  = 0; iiwa_delta_old_wrench.wrench.force.y  = 0; iiwa_delta_old_wrench.wrench.force.z  = 0;
  iiwa_delta_old_wrench.wrench.torque.x = 0; iiwa_delta_old_wrench.wrench.torque.y = 0; iiwa_delta_old_wrench.wrench.torque.z = 0;
  
//   ROS_INFO("start message filter");
  message_filters::Subscriber<geometry_msgs::WrenchStamped>     cartesian_wrench_logger_sub(nh, "/iiwa/state/CartesianWrench",       1);
  message_filters::Subscriber<iiwa_msgs::JointPositionVelocity> joint_pos_vel_logger_sub   (nh, "/iiwa/state/JointPositionVelocity", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped>       cartesian_pose_logger_sub  (nh, "/iiwa/state/CartesianPose",         1);


  
  typedef message_filters::sync_policies::ApproximateTime< geometry_msgs::WrenchStamped, iiwa_msgs::JointPositionVelocity, geometry_msgs::PoseStamped> MySyncPolicy;
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), cartesian_wrench_logger_sub, joint_pos_vel_logger_sub, cartesian_pose_logger_sub);
  
  sync.registerCallback(boost::bind(&iiwa_sync_callback, _1, _2, _3));
  
  bool_sub             = nh.subscribe("/iiwa/execute_exp_topic", 1,  &iiwa_bool_cb);


  ros::spin();

  
  log_file.close();
//   ROS_WARN_STREAM("log file closed2...");
  
  return 0;
  
}



