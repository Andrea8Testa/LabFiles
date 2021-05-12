// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/DMP_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

#include <fstream>
#include <unistd.h>

namespace franka_example_controllers {

bool DMP_controller::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  
  franka_EE_pose_pub = node_handle.advertise<geometry_msgs::Pose>("/franka_ee_pose", 1000);

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/DMP_pose", 20, &DMP_controller::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("DMP_controller: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "DMP_controller: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DMP_controller: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DMP_controller: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DMP_controller: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DMP_controller: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DMP_controller: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DMP_controller: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  return true;
}

void DMP_controller::starting(const ros::Time& /*time*/) {
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
  
  msrTimestep = 0.001;
  filtTime = msrTimestep*2.;
  digfilt = 0.;
	
  if (filtTime>0.)
	digfilt = exp(-msrTimestep/filtTime);
}

void DMP_controller::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
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
      robot_state.tau_J_d.data());
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
  
  // compute control
  Eigen::VectorXd tau_ctrl(7), tau_d(7), ctrl_velocity(6), position_err(6), int_jnt_pos_err(7), int_jnt_pos_err_old(7);
  
  position_err.setZero();
  position_err(0) = position_d_(0) - position(0);
  position_err(1) = position_d_(1) - position(1);
  position_err(2) = position_d_(2) - position(2);
  
  Eigen::MatrixXd Kpos(6,6);
  Kpos.setZero();
  for (int j = 0; j < 6; ++j)
    Kpos(j,j) = 0.5;
  
  ctrl_velocity = velocity_d_ + Kpos*position_err;
  
  for (int j = 0; j < 7; ++j)
    joint_pos_d_old(j) = joint_pos_d_vec_[j];
  
  joint_velocity_d = Jpinv*ctrl_velocity;
  joint_pos_d = joint_pos_d_old + joint_velocity_d*0.001;
  
  int_jnt_pos_err_old.setZero();
  for (int j = 0; j < 7; ++j)
      int_jnt_pos_err_old(j) = int_jnt_pos_err_vec_[j];
  
  int_jnt_pos_err.setZero();
  int_jnt_pos_err = int_jnt_pos_err_old + (joint_pos_d - q)*0.001;
  
  for (int j = 0; j < 7; ++j)
      int_jnt_pos_err_vec_[j] = int_jnt_pos_err(j);
  
  Eigen::MatrixXd Kp(7,7), Kd(7,7), Ki(7,7);
  Kp.setZero();
  Kd.setZero();
  Ki.setZero();
  
  Kp(0,0) = 1000.;
  Kp(1,1) = 1500.;
  Kp(2,2) = 3000.;
  Kp(3,3) = 2500.; // 3200.
  Kp(4,4) = 800.;
  Kp(5,5) = 600.;
  Kp(6,6) = 300.;
  
  Kd(0,0) = 32.;
  Kd(1,1) = 32.;
  Kd(2,2) = 30.;
  Kd(3,3) = 45.;
  Kd(4,4) = 20.;
  Kd(5,5) = 20.;
  Kd(6,6) = 10.;
  
  Ki(0,0) = 17.;
  Ki(1,1) = 12.;
  Ki(2,2) = 25.;
  Ki(3,3) = 59.;
  Ki(4,4) = 50.;
  Ki(5,5) = 32.;
  Ki(6,6) = 53.;
  
  tau_ctrl << Kp*(joint_pos_d - q) + 0.95*Kd*(joint_velocity_d - dq_filt_Eigen) + Ki*int_jnt_pos_err;
  tau_d << tau_ctrl + coriolis;
  
  for (int j = 0; j < 7; ++j)
    joint_pos_d_vec_[j] = joint_pos_d(j);
  
//   if (cont_task_setpoint%1000 == 0) //&& cont_task_setpoint<10
//   {D
//     std::cout << "tau_ctrl: " << tau_ctrl << std::endl;
//     std::cout << "joint_velocity_d: " << joint_velocity_d << std::endl;
//     std::cout << "velocity_d_: " << velocity_d_ << std::endl;
//     std::cout << "position_d_: " << position_d_ << std::endl;
//     std::cout << "dq_filt_Eigen: " << dq_filt_Eigen << std::endl;
//     std::cout << "joint_pos_d: " << joint_pos_d << std::endl;
//     std::cout << "q: " << q << std::endl;
//     std::cout << "int_jnt_pos_err:" << int_jnt_pos_err << std::endl;
//     std::cout << "----------" << std::endl;
//   }
  
  cont_task_setpoint++;
  
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

}

Eigen::Matrix<double, 7, 1> DMP_controller::saturateTorqueRate(
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

void DMP_controller::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DMP_controller,
                       controller_interface::ControllerBase)
