// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/Torque_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

#include <fstream>
#include <unistd.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

namespace franka_example_controllers {

bool Torque_controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  
 // franka_EE_pose_pub = node_handle.advertise<geometry_msgs::Pose>("/franka_ee_pose", 1000);
 time_pub = node_handle.advertise<std_msgs::Float64>("/Tempo", 1000);


//ros::NodeHandle n;
 coppia_sub0 = node_handle.subscribe("/Coppia0", 1000, &Torque_controller::CoppiaCallback0, this,
      ros::TransportHints().reliable().tcpNoDelay());
 coppia_sub1 = node_handle.subscribe("/Coppia1", 1000, &Torque_controller::CoppiaCallback1, this,
      ros::TransportHints().reliable().tcpNoDelay());
 coppia_sub2 = node_handle.subscribe("/Coppia2", 1000, &Torque_controller::CoppiaCallback2, this,
      ros::TransportHints().reliable().tcpNoDelay());
 coppia_sub3 = node_handle.subscribe("/Coppia3", 1000, &Torque_controller::CoppiaCallback3, this,
      ros::TransportHints().reliable().tcpNoDelay());
 coppia_sub4 = node_handle.subscribe("/Coppia4", 1000, &Torque_controller::CoppiaCallback4, this,
      ros::TransportHints().reliable().tcpNoDelay());
 coppia_sub5 = node_handle.subscribe("/Coppia5", 1000, &Torque_controller::CoppiaCallback5, this,
      ros::TransportHints().reliable().tcpNoDelay());
 coppia_sub6 = node_handle.subscribe("/Coppia6", 1000, &Torque_controller::CoppiaCallback6, this,
      ros::TransportHints().reliable().tcpNoDelay());
  /*sub_equilibrium_pose_ = node_handle.subscribe(
      "/DMP_pose", 20, &Torque_controller::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());*/

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("Torque_controller: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "Torque_controller: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "Torque_controller: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "Torque_controller: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "Torque_controller: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "Torque_controller: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "Torque_controller: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "Torque_controller: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  return true;
}

void Torque_controller::starting(const ros::Time& time) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  initial_state = state_handle_->getRobotState();
  
  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_0(initial_state.q.data());
  
  cont_task_setpoint = 0;
  
  for (int j = 0; j < 7; ++j)
  {
    joint_pos_d_vec_[j] = q_0(j);
    dq_filt[j] = 0.;
  }
  
  for (int j = 0; j < 6; ++j)
    velocity_d_vec_[j] = 0.;
  
  /*msrTimestep = 0.001;
  filtTime = msrTimestep*2.;
  digfilt = 0.;
	
  if (filtTime>0.)
	digfilt = exp(-msrTimestep/filtTime);*/
}

double tempo=0;
double coppia_importata0, coppia_importata1, coppia_importata2, coppia_importata3, coppia_importata4, coppia_importata5, coppia_importata6;

void Torque_controller::update(const ros::Time& time, const ros::Duration& period) {

  tempo += period.toSec();

  std::array<double, 49> inertia_array = model_handle_->getMass();
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7> > inertia(inertia_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data()); //PRIME ERA: robot_state.tau_J_d.data());
  /*
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = position(0);
  pose_msg.position.y = position(1);
  pose_msg.position.z = position(2);
  pose_msg.orientation.x = orientation.x();
  pose_msg.orientation.y = orientation.y();
  pose_msg.orientation.z = orientation.z();
  pose_msg.orientation.w = orientation.w();
  
  franka_EE_pose_pub.publish(pose_msg);
  
  velocity_d_vec_[0] = ( ( filter_params_ * position_d_target_(0) + (1.0 - filter_params_) * position_d_(0) ) - position_d_(0) ) / 0.001;
  
  velocity_d_vec_[1] = ( ( filter_params_ * position_d_target_(1) + (1.0 - filter_params_) * position_d_(1) ) - position_d_(1) ) / 0.001;
  
  Eigen::VectorXd velocity_d_(6);
  
  for (int j = 0; j < 6; ++j)
    velocity_d_(j) = velocity_d_vec_[j];
  
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
//   orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  
  Eigen::VectorXd joint_velocity_d(7), joint_pos_d(7), joint_pos_d_old(7);
  
  Eigen::MatrixXd Jpinv(7,6);
  Jpinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
  
  for (int j = 0; j < 7; ++j)
    dq_filt[j] = dq_filt[j] * digfilt + (1-digfilt) * dq(j);
  
  Eigen::VectorXd dq_filt_Eigen(7);
  for (int j = 0; j < 7; ++j)
      dq_filt_Eigen(j) = dq_filt[j];
  
  */


std_msgs::Float64 time_msg;
time_msg.data = tempo;
time_pub.publish(time_msg);



 Eigen::VectorXd tau_d(7);
 tau_d(0) = coppia_importata0; //coppia_importata;//0;//3*sin(tempo*3.14);
 tau_d(1) = coppia_importata1;
 tau_d(2) = coppia_importata2;
 tau_d(3) = coppia_importata3;
 tau_d(4) = coppia_importata4;//1*sin(tempo*3.14);
 tau_d(5) = coppia_importata5;
 tau_d(6) = coppia_importata6;

  // compute control
  //tau_d << tau_ctrl + coriolis;
  
  //for (int j = 0; j < 7; ++j)
  //  joint_pos_d_vec_[j] = joint_pos_d(j);
  
  cont_task_setpoint++;
  
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
    printf("%f \n",  tau_d[i]);
  }
    printf("\n");

printf("tempo: %f \n",  tempo);
printf("\n");
//printf("coppia importata: %f \n",  coppia_importata);
//printf("\n");
//printf("coppia eseguita: %f \n",  tau_d(0));
//printf("\n");


}

Eigen::Matrix<double, 7, 1> Torque_controller::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void Torque_controller::CoppiaCallback0(const std_msgs::Float64::ConstPtr& msg)
{
  coppia_importata0 = msg->data;
}

void Torque_controller::CoppiaCallback1(const std_msgs::Float64::ConstPtr& msg)
{
  coppia_importata1 = msg->data;
}

void Torque_controller::CoppiaCallback2(const std_msgs::Float64::ConstPtr& msg)
{
  coppia_importata2 = msg->data;
}

void Torque_controller::CoppiaCallback3(const std_msgs::Float64::ConstPtr& msg)
{
  coppia_importata3 = msg->data;
}

void Torque_controller::CoppiaCallback4(const std_msgs::Float64::ConstPtr& msg)
{
  coppia_importata4 = msg->data;
}

void Torque_controller::CoppiaCallback5(const std_msgs::Float64::ConstPtr& msg)
{
  coppia_importata5 = msg->data;
}

void Torque_controller::CoppiaCallback6(const std_msgs::Float64::ConstPtr& msg)
{
  coppia_importata6 = msg->data;
}
/*
void Torque_controller::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}
*/
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::Torque_controller,
                       controller_interface::ControllerBase)
