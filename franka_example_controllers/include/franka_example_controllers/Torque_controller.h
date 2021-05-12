// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class Torque_controller : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.9999/*0.005*/};
  const double delta_tau_max_{5.0};
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  
  ros::Publisher  time_pub;

  int cont_task_setpoint;
  
  double msrTimestep;
  double filtTime;
  double digfilt;
  std::array<double, 7> dq_filt;
  std::array<double, 7> joint_pos_d_vec_;
  std::array<double, 7> int_jnt_pos_err_vec_;
  std::array<double, 6> velocity_d_vec_;
  
  franka::RobotState initial_state;
  
  // Equilibrium pose subscriber
  ros::Subscriber coppia_sub0;
  void CoppiaCallback0(const std_msgs::Float64::ConstPtr& msg);
  ros::Subscriber coppia_sub1;
  void CoppiaCallback1(const std_msgs::Float64::ConstPtr& msg);
  ros::Subscriber coppia_sub2;
  void CoppiaCallback2(const std_msgs::Float64::ConstPtr& msg);
  ros::Subscriber coppia_sub3;
  void CoppiaCallback3(const std_msgs::Float64::ConstPtr& msg);
  ros::Subscriber coppia_sub4;
  void CoppiaCallback4(const std_msgs::Float64::ConstPtr& msg);
  ros::Subscriber coppia_sub5;
  void CoppiaCallback5(const std_msgs::Float64::ConstPtr& msg);
  ros::Subscriber coppia_sub6;
  void CoppiaCallback6(const std_msgs::Float64::ConstPtr& msg);
  ros::Subscriber coppia_sub7;
  void CoppiaCallback7(const std_msgs::Float64::ConstPtr& msg);
};

}  // namespace franka_example_controllers
