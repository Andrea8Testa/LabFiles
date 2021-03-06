// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_QLMPC_controller.h>

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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>


namespace franka_example_controllers {

    bool CartesianImpedanceQLMPCController::init(hardware_interface::RobotHW* robot_hw,
        ros::NodeHandle& node_handle) {
        std::vector<double> cartesian_stiffness_vector;
        std::vector<double> cartesian_damping_vector;

        franka_EE_pose_pub = node_handle.advertise<geometry_msgs::Pose>("/franka_ee_pose", 1000);
        franka_EE_velocity_pub = node_handle.advertise<geometry_msgs::Twist>("/franka_ee_velocity", 1000);
        franka_EE_wrench_pub = node_handle.advertise<geometry_msgs::Wrench>("/franka_ee_wrench", 1000);

        sub_equilibrium_pose_ = node_handle.subscribe(
            "/QLMPC_pose", 20, &CartesianImpedanceQLMPCController::equilibriumPoseCallback, this,
            ros::TransportHints().reliable().tcpNoDelay());
        sub_damping_ = node_handle.subscribe("/D_information", 20, &CartesianImpedanceQLMPCController::dampingCallback, this,
            ros::TransportHints().reliable().tcpNoDelay());

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR_STREAM("CartesianImpedanceQLMPCController: Could not read parameter arm_id");
            return false;
        }
        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
            ROS_ERROR(
                "CartesianImpedanceQLMPCController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
            return false;
        }

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceQLMPCController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceQLMPCController: Exception getting model handle from interface: "
                << ex.what());
            return false;
        }

        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceQLMPCController: Error getting state interface from hardware");
            return false;
        }
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle(arm_id + "_robot"));
        }
        catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceQLMPCController: Exception getting state handle from interface: "
                << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceQLMPCController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                    "CartesianImpedanceQLMPCController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }
        /*
        dynamic_reconfigure_compliance_param_node_ =
            ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

        dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<
            franka_example_controllers::compliance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
        dynamic_server_compliance_param_->setCallback(
            boost::bind(&CartesianImpedanceQLMPCController::complianceParamCallback, this, _1, _2));
        */
        // definition
        position_d_.setZero();
        orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        position_d_target_.setZero();
        orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

        cartesian_stiffness_.setZero();
        cartesian_damping_.setZero();

        return true;
    }

    void CartesianImpedanceQLMPCController::starting(const ros::Time& /*time*/) {
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

        cont_task_setpoint = 0;

        for (int j = 0; j < 3; ++j)
        {
            dw_psp_world(j) = 0.;
            drpy_cmd(j) = 0.;
            rpy_cmd(j) = 0.;
            drpy(j) = 0.;
            rpy_old(j) = 0.;
            dposition(j) = 0.;
            position_old(j) = 0.;
            dposition_filt(j) = 0.;
            drpy_filt(j) = 0.;
        }

        for (int j = 0; j < 7; ++j)
            dq_filt[j] = 0.;

        h_damp_t = 0.5; //0.9;                  // last "Roveda" value = 0.75
        h_damp_r = 0.5; //0.9;                  // last "Roveda" value = 0.75
        mass_imp = 1.; //5.
        inertia_imp = 1.; //5.
        translational_stiffness = 1000;//3000.; // last "Roveda" value = 7500
        rotational_stiffness = 1000;//10000.;   // last "Roveda" value = 15000

        msrTimestep = 0.001;
        filtTime = msrTimestep * 2.;
        digfilt = 0.;

        if (filtTime > 0.)
            digfilt = exp(-msrTimestep / filtTime);
    }

    void CartesianImpedanceQLMPCController::update(const ros::Time& /*time*/,
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
        // load from the robot the updated state
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());
        // load from the robot the updated wrench
        Eigen::Map<Eigen::Matrix<double, 6, 1>> wrench(robot_state.O_F_ext_hat_K.data());

        Eigen::MatrixXd stiffness(6, 6), damping(6, 6), mass(6, 6), inv_mass(6, 6);
        mass.setZero();
        mass.topLeftCorner(3, 3) << mass_imp * Eigen::MatrixXd::Identity(3, 3);
        mass.bottomRightCorner(3, 3) << inertia_imp * Eigen::MatrixXd::Identity(3, 3);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * mass_imp * h_damp_t * sqrt(translational_stiffness / mass_imp) *
            Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * inertia_imp * h_damp_r * sqrt(rotational_stiffness / inertia_imp) *
            Eigen::MatrixXd::Identity(3, 3);
        damping(3, 3) = damping_importato * 2.0 * mass_imp * sqrt(translational_stiffness / mass_imp);
        // pose publisher
        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = position(0);
        pose_msg.position.y = position(1);
        pose_msg.position.z = position(2);
        pose_msg.orientation.x = orientation.x();
        pose_msg.orientation.y = orientation.y();
        pose_msg.orientation.z = orientation.z();
        pose_msg.orientation.w = orientation.w();

        franka_EE_pose_pub.publish(pose_msg);

        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
        orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

        if (cont_task_setpoint % 1 == 0 && cont_task_setpoint < 10)
        {
            // print
            std::cout << position_d_target_ << std::endl;
            std::cout << "----------" << std::endl;
        }

        cont_task_setpoint++;

        Eigen::Affine3d transform_cmd(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Matrix3d R_msr(transform.rotation());
        Eigen::Matrix3d R_cmd(transform_cmd.rotation());
        Eigen::Matrix3d R_comp = R_msr.inverse() * R_cmd;
        Eigen::Matrix3d T_dangles_to_w_cd;
        Eigen::Matrix3d dT_dangles_to_w_cd;
        Eigen::Vector3d rpy;

        rpy(2) = atan2(R_comp(1, 0), R_comp(0, 0));
        rpy(1) = atan2(-R_comp(2, 0), pow((pow(R_comp(2, 1), 2) + pow(R_comp(2, 2), 2)), 0.5));
        rpy(0) = atan2(R_comp(2, 1), R_comp(2, 2));

        for (int j = 0; j < 7; ++j)
            dq_filt[j] = dq_filt[j] * digfilt + (1 - digfilt) * dq(j);
        // cartesian velocity calculation
        for (int j = 0; j < 3; ++j)
        {
            dposition(j) = (position(j) - position_old(j)) / 0.001;
            dposition_filt(j) = dposition_filt(j) * digfilt + (1 - digfilt) * dposition(j);
            drpy(j) = (rpy(j) - rpy_old(j)) / 0.001;
            drpy_filt(j) = drpy_filt(j) * digfilt + (1 - digfilt) * drpy(j);
        }
        // velocity publisher
        geometry_msgs::Twist velocity_msg;
        velocity_msg.linear.x = dposition_filt(0);
        velocity_msg.linear.y = dposition_filt(1);
        velocity_msg.linear.z = dposition_filt(2);
        velocity_msg.angular.x = drpy_filt(0);
        velocity_msg.angular.y = drpy_filt(1);
        velocity_msg.angular.z = drpy_filt(2);

        franka_EE_velocity_pub.publish(velocity_msg);

        // wrench publisher
        geometry_msgs::Wrench wrench_msg;
        wrench_msg.force.x = wrench(0);
        wrench_msg.force.y = wrench(1);
        wrench_msg.force.z = wrench(2);
        wrench_msg.torque.x = wrench(3);
        wrench_msg.torque.y = wrench(4);
        wrench_msg.torque.z = wrench(5);

        franka_EE_wrench_pub.publish(wrench_msg);

        inv_mass = mass.inverse();

        position_old = position;
        rpy_old = rpy;

        // compute error to desired equilibrium pose
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d_;
        // orientation error
        // "difference" quaternion
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.linear() * error.tail(3);
        // compute control
        Eigen::VectorXd tau_task(7), tau_d(7), tau_nullspace(7);
        // Spring damper system with damping ratio=1
        tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));

        Eigen::MatrixXd NullSpace(7, 7);
        Eigen::MatrixXd Jpinv(7, 6);
        Jpinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

        for (int j = 0; j < 7; ++j)
            tau_nullspace(j) = 0.;

        NullSpace = Eigen::MatrixXd::Identity(7, 7) - Jpinv * jacobian;

        Eigen::MatrixXd M_nullspace(7, 7), D_nullspace(7, 7);

        double mns = 1.;
        double kns = 1000.; // last "Roveda" value = 1000
        double hns = 5.;
        double dns = 2 * hns * mns * pow((kns / mns), 0.5);

        M_nullspace = mns * Eigen::MatrixXd::Identity(7, 7);
        D_nullspace = dns * Eigen::MatrixXd::Identity(7, 7);

        Eigen::VectorXd dq_filt_Eigen(7);
        for (int j = 0; j < 7; ++j)
            dq_filt_Eigen(j) = dq_filt[j];
        tau_nullspace = inertia * (NullSpace * (-M_nullspace.inverse() * D_nullspace * dq_filt_Eigen));

        //tau_imp << inertia * (Jpinv * acc_cmd); // once we get the required acceleration, we can obtain the torques 
        tau_d << tau_task + coriolis + tau_nullspace;

        // Saturate torque rate to avoid discontinuities
        tau_d << saturateTorqueRate(tau_d, tau_J_d);
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i));
        }

        // update parameters changed online either through dynamic reconfigure or through the interactive target by filtering
        cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    }

    Eigen::Matrix<double, 7, 1> CartesianImpedanceQLMPCController::saturateTorqueRate(
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
    /*
    void CartesianImpedanceQLMPCController::complianceParamCallback(
        franka_example_controllers::compliance_paramConfig& config,
        uint32_t) {

        double t_stiff = 1000.;
        double r_stiff = 10.;
        double nsp_stiff = 10.;

        cartesian_stiffness_target_.setIdentity();
        cartesian_stiffness_target_.topLeftCorner(3, 3)
            <<t_stiff * Eigen::Matrix3d::Identity();
        cartesian_stiffness_target_.bottomRightCorner(3, 3)
            << r_stiff * Eigen::Matrix3d::Identity();
        cartesian_damping_target_.setIdentity();
        // Damping ratio = 1
        cartesian_damping_target_.topLeftCorner(3, 3)
            << 2.0 * sqrt(t_stiff) * Eigen::Matrix3d::Identity();
        cartesian_damping_target_.bottomRightCorner(3, 3)
            << 2.0 * sqrt(r_stiff) * Eigen::Matrix3d::Identity();
        nullspace_stiffness_target_ = nsp_stiff;
    }
    */

    void CartesianImpedanceQLMPCController::equilibriumPoseCallback(
        const geometry_msgs::PoseConstPtr& msg) {
        position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
        orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
            msg->pose.orientation.z, msg->pose.orientation.w;
        if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
            orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
        }
    }

    void CartesianImpedanceQLMPCController::dampingCallback(const std_msgs::Float64::ConstPtr& msg) {
        damping_importato = msg->data;
    }

} 

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceQLMPCController,
    controller_interface::ControllerBase)