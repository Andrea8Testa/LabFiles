
#include <iiwa_ros/iiwa_ros.h>
#include <ros/console.h>
#include <geometry_msgs/Transform.h>
#include <math.h> 
#include <iiwa_ros/conversions.h>

geometry_msgs::Transform iiwa_pose;

void iiwa_pose_sub_cb ( const geometry_msgs::PoseStampedConstPtr msg )
{
  
  iiwa_pose.translation.x=msg->pose.position.x;
  iiwa_pose.translation.y=msg->pose.position.y;
  iiwa_pose.translation.z=msg->pose.position.z;
  iiwa_pose.rotation.x=msg->pose.orientation.x;
  iiwa_pose.rotation.y=msg->pose.orientation.y;
  iiwa_pose.rotation.z=msg->pose.orientation.z;
  iiwa_pose.rotation.w=msg->pose.orientation.w;

}

int main (int argc, char **argv) {
	
  // Initialize ROS
  ros::init(argc, argv, "iiwa_stack_motion_test");
  ros::NodeHandle nh("~");

  // ROS spinner.
  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  
  ////////////iiwa////////////
  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init();
  
  ros::Subscriber iiwa_pose_sub=nh.subscribe ("/iiwa/state/CartesianPose",1,iiwa_pose_sub_cb);
  
  ros::spinOnce();
  
  usleep ( 500 );
  
  ros::spinOnce();
  
  ///////////////////////Load Parameters////////////////////////
  
  const std::string& name_param = "/iiwa_stack_test_param";
  
  double control_freq = 200;
  
  if ( !nh.getParam ( name_param+"/control_freq", control_freq) )
    ROS_ERROR ( "control_freq parameters not found \n" );
  
  double st = 1./control_freq;
  ros::WallRate loop_rate (1./st);
  
  double delta_p = 0.01;
  
  if ( !nh.getParam ( name_param+"/delta_p", delta_p) )
    ROS_ERROR ( "delta_p parameters not found \n" );
  
  double uplim = 0.8;
  
  if ( !nh.getParam ( name_param+"/uplim", uplim) )
    ROS_ERROR ( "uplim parameters not found \n" );
  
  double lowlim = 0.5;
  
  if ( !nh.getParam ( name_param+"/lowlim", lowlim) )
    ROS_ERROR ( "lowlim parameters not found \n" );
  
  int max_motions = 5;
  
  if ( !nh.getParam ( name_param+"/max_motions", max_motions) )
    ROS_ERROR ( "max_motions parameters not found \n" );
  
  std::vector<double> stiffness;
  
  if ( !nh.getParam ( name_param+"/Kr", stiffness) )
    ROS_ERROR ( "Stiffness parameters not found \n" );
    
  if ( stiffness.size() != 6 )
    ROS_ERROR ( "Stiffness parameters with wrong dimensions \n" );
    
  double damping;
  
  if ( !nh.getParam ( name_param+"/Dr", damping) )
  {
    
    ROS_ERROR ( "Damping parameters not found \n" );
   
    damping = 0.9;
    
  }
  
  ros::spinOnce();
  
  ros::Duration(2.0).sleep();
  
  ros::spinOnce();
  
  my_iiwa.getServoMotion().setPositionControlMode();
  
  ros::spinOnce();
  
  ///////////////////////Initial Joint Position////////////////////////
  
  iiwa_msgs::JointPosition command_joint_position;
  iiwa_msgs::JointPosition msr_joint_position;
  geometry_msgs::PoseStamped command_cartesian_position;
  
  command_joint_position.position.a1 = 3.14/180. * -60.30;
  command_joint_position.position.a2 = 3.14/180. * 51.93;
  command_joint_position.position.a3 = 3.14/180. * 142.43;
  command_joint_position.position.a4 = 3.14/180. * 101.11;
  command_joint_position.position.a5 = 3.14/180. * -132.82;
  command_joint_position.position.a6 = 3.14/180. * 40.51;
  command_joint_position.position.a7 = 3.14/180. * -28.01;
  
  bool connection_flag = false;
  
  std::cout << "positioning robot..." << std::endl;
  
  while (!connection_flag){
  
    if (my_iiwa.getRobotIsConnected()) {
    
      connection_flag = true;
      
      while (!my_iiwa.getJointPosition(msr_joint_position)) {}
      
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
  
  command_cartesian_position.pose.position.x = iiwa_pose.translation.x;
  command_cartesian_position.pose.position.y = iiwa_pose.translation.y;
  command_cartesian_position.pose.position.z = iiwa_pose.translation.z;
  command_cartesian_position.pose.orientation.x = iiwa_pose.rotation.x;
  command_cartesian_position.pose.orientation.y = iiwa_pose.rotation.y;
  command_cartesian_position.pose.orientation.z = iiwa_pose.rotation.z;
  command_cartesian_position.pose.orientation.w = iiwa_pose.rotation.w;
  
  command_cartesian_position.pose.position.z = command_cartesian_position.pose.position.z + 0.3;
  
  my_iiwa.setCartesianPose(command_cartesian_position);
  
  ros::Duration(5.0).sleep();
  
  std::cout << "robot positioned!" << std::endl;
  
  ros::spinOnce();
  
  ros::Duration(2.0).sleep();
  
  ros::spinOnce();
  
  connection_flag = false;
  
  std::cout << "going to start impedance..." << std::endl;
  
  while (!connection_flag){
  
    if (my_iiwa.getRobotIsConnected()) {
    
      connection_flag = true;
      
      while (!my_iiwa.getJointPosition(msr_joint_position)) {}
      
      my_iiwa.getServoMotion().setCartesianImpedanceMode(iiwa_ros::CartesianQuantityFromDouble(stiffness[0],stiffness[1],stiffness[2],stiffness[3],stiffness[4],stiffness[5]), 
							 iiwa_ros::CartesianQuantityFromDouble(damping));
      
      command_cartesian_position.pose.position.x = iiwa_pose.translation.x;
      command_cartesian_position.pose.position.y = iiwa_pose.translation.y;
      command_cartesian_position.pose.position.z = iiwa_pose.translation.z;
      command_cartesian_position.pose.orientation.x = iiwa_pose.rotation.x;
      command_cartesian_position.pose.orientation.y = iiwa_pose.rotation.y;
      command_cartesian_position.pose.orientation.z = iiwa_pose.rotation.z;
      command_cartesian_position.pose.orientation.w = iiwa_pose.rotation.w;
      
    }
    else
    {
    
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
      
    }
    
    ros::spinOnce();
  
  }
  
  ros::Duration(2.0).sleep();
  
  bool app_end = false;
  int cont_app = 0;
  int direction = 1;
  
  command_cartesian_position.pose.position.z = command_cartesian_position.pose.position.z - 0.3;
  
  my_iiwa.setCartesianPose(command_cartesian_position);
  
  ros::Duration(5.0).sleep();
  
  std::cout << "starting motion: " << std::endl;
  
  while (ros::ok() && !app_end) {
	
    command_cartesian_position.pose.position.z = iiwa_pose.translation.z + direction * delta_p;
    
    if ( command_cartesian_position.pose.position.z >= uplim )
    {
      
      command_cartesian_position.pose.position.z = uplim;
      direction = -1;
      
    } else if ( command_cartesian_position.pose.position.z <= lowlim )
    {
      
      command_cartesian_position.pose.position.z = lowlim;
      direction = 1;
      cont_app++;
      
    }
    
    my_iiwa.setCartesianPose(command_cartesian_position);
    
    if ( cont_app >= max_motions )
      app_end = true;
  
    if ( !ros::ok() ){
	  
      return 0;

    }
    
    loop_rate.sleep();
    ros::spinOnce();
    
  }
  
  ROS_WARN_STREAM("Motion Ended");
    
  return 0;
    
};

